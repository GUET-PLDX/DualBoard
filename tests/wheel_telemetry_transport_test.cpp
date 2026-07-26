#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace {

constexpr uint8_t ONLINE = 1U << 0;
constexpr uint8_t FRESH = 1U << 1;
constexpr uint8_t ENCODING_SATURATED = 1U << 4;
constexpr uint8_t CHASSIS_POWER_ON = 1U << 0;
constexpr uint8_t POWER_STATE_VALID = 1U << 1;
constexpr uint8_t DIAGNOSTIC_VALID = 1U << 2;
constexpr uint8_t TRANSPORT_VALID = 1U << 4;
constexpr uint8_t TIME_SYNC_VALID = 1U << 5;
constexpr uint8_t STALE = 1U << 6;
constexpr uint8_t VERSION = 1U;

struct __attribute__((packed)) Meta {
  uint8_t version;
  uint16_t sequence;
  uint32_t sample_time_us;
  uint8_t global_status;
};
struct __attribute__((packed)) Pair {
  uint16_t sequence;
  int16_t first_q;
  int16_t second_q;
  uint8_t first_status;
  uint8_t second_status;
};
struct __attribute__((packed)) Diagnostic {
  uint16_t sequence;
  int16_t vx_mm_s;
  int16_t vy_mm_s;
  int16_t wz_mrad_s;
};
static_assert(sizeof(Meta) == 8U);
static_assert(sizeof(Pair) == 8U);
static_assert(sizeof(Diagnostic) == 8U);

struct Sample {
  uint64_t time_us = 0U;
  uint16_t sequence = 0U;
  float wheel[4]{};
  uint8_t status[4]{};
  uint8_t global = 0U;
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
};

int16_t EncodeQ(float value, uint8_t& status) {
  const float scaled = value * 256.0f;
  if (!std::isfinite(scaled) || scaled > 32767.0f) {
    status = static_cast<uint8_t>((status | ENCODING_SATURATED) & ~FRESH);
    return 32767;
  }
  if (scaled < -32768.0f) {
    status = static_cast<uint8_t>((status | ENCODING_SATURATED) & ~FRESH);
    return -32768;
  }
  return static_cast<int16_t>(std::lround(scaled));
}

struct Frames {
  Meta meta{};
  Pair pair01{};
  Diagnostic diagnostic{};
  Pair pair23{};
};

Frames Encode(const Sample& sample) {
  Frames frames{};
  frames.meta = {VERSION, sample.sequence,
                 static_cast<uint32_t>(sample.time_us), sample.global};
  frames.pair01.sequence = sample.sequence;
  frames.pair23.sequence = sample.sequence;
  frames.pair01.first_status = sample.status[0];
  frames.pair01.second_status = sample.status[1];
  frames.pair23.first_status = sample.status[2];
  frames.pair23.second_status = sample.status[3];
  frames.pair01.first_q = EncodeQ(sample.wheel[0], frames.pair01.first_status);
  frames.pair01.second_q =
      EncodeQ(sample.wheel[1], frames.pair01.second_status);
  frames.pair23.first_q = EncodeQ(sample.wheel[2], frames.pair23.first_status);
  frames.pair23.second_q =
      EncodeQ(sample.wheel[3], frames.pair23.second_status);
  frames.diagnostic = {sample.sequence, static_cast<int16_t>(sample.vx * 1000),
                       static_cast<int16_t>(sample.vy * 1000),
                       static_cast<int16_t>(sample.wz * 1000)};
  return frames;
}

class Assembler {
 public:
  bool PushMeta(const Meta& frame, uint32_t now, Sample& output) {
    if (frame.version != VERSION || !Prepare(frame.sequence, 1U, now)) {
      return false;
    }
    frames_.meta = frame;
    return Complete(output);
  }
  bool PushPair01(const Pair& frame, uint32_t now, Sample& output) {
    if (!Prepare(frame.sequence, 2U, now)) return false;
    frames_.pair01 = frame;
    return Complete(output);
  }
  bool PushDiagnostic(const Diagnostic& frame, uint32_t now, Sample& output) {
    if (!Prepare(frame.sequence, 4U, now)) return false;
    frames_.diagnostic = frame;
    return Complete(output);
  }
  bool PushPair23(const Pair& frame, uint32_t now, Sample& output) {
    if (!Prepare(frame.sequence, 8U, now)) return false;
    frames_.pair23 = frame;
    return Complete(output);
  }
  bool Expire(uint32_t now) {
    if (!active_ || now - started_ <= 20U) return false;
    ResetStorage();
    return true;
  }

 private:
  static bool Newer(uint16_t candidate, uint16_t reference) {
    return static_cast<int16_t>(candidate - reference) > 0;
  }
  void Start(uint16_t sequence, uint32_t now) {
    frames_ = {};
    sequence_ = sequence;
    started_ = now;
    mask_ = 0U;
    active_ = true;
  }
  void ResetStorage() {
    frames_ = {};
    mask_ = 0U;
    active_ = false;
  }
  bool Prepare(uint16_t sequence, uint8_t bit, uint32_t now) {
    Expire(now);
    if (completed_ && sequence == last_completed_) return false;
    if (!active_) {
      Start(sequence, now);
    } else if (sequence != sequence_) {
      if (!Newer(sequence, sequence_)) return false;
      Start(sequence, now);
    }
    if ((mask_ & bit) != 0U) return false;
    mask_ |= bit;
    return true;
  }
  bool Complete(Sample& output) {
    if ((mask_ & 0x0BU) != 0x0BU) return false;
    output.time_us = frames_.meta.sample_time_us;
    output.sequence = sequence_;
    output.wheel[0] = frames_.pair01.first_q / 256.0f;
    output.wheel[1] = frames_.pair01.second_q / 256.0f;
    output.wheel[2] = frames_.pair23.first_q / 256.0f;
    output.wheel[3] = frames_.pair23.second_q / 256.0f;
    output.status[0] = frames_.pair01.first_status;
    output.status[1] = frames_.pair01.second_status;
    output.status[2] = frames_.pair23.first_status;
    output.status[3] = frames_.pair23.second_status;
    output.global = static_cast<uint8_t>(
        (frames_.meta.global_status | TRANSPORT_VALID) & ~TIME_SYNC_VALID);
    if ((mask_ & 4U) != 0U) {
      output.vx = frames_.diagnostic.vx_mm_s / 1000.0f;
      output.vy = frames_.diagnostic.vy_mm_s / 1000.0f;
      output.wz = frames_.diagnostic.wz_mrad_s / 1000.0f;
    } else {
      output.global &= static_cast<uint8_t>(~DIAGNOSTIC_VALID);
    }
    last_completed_ = sequence_;
    completed_ = true;
    ResetStorage();
    return true;
  }

  Frames frames_{};
  uint32_t started_ = 0U;
  uint16_t sequence_ = 0U;
  uint16_t last_completed_ = 0U;
  uint8_t mask_ = 0U;
  bool active_ = false;
  bool completed_ = false;
};

class Watchdog {
 public:
  void Receive(uint32_t now, const Sample& sample) {
    last_ = sample;
    last_rx_ = now;
    received_ = true;
    stale_ = false;
  }
  bool Tick(uint32_t now, Sample& output) {
    const uint32_t reference = received_ ? last_rx_ : start_;
    if (now - reference <= 50U) return false;
    if (stale_ && now - last_publish_ < 100U) return false;
    output = last_;
    output.global = static_cast<uint8_t>(
        (output.global &
         ~(TRANSPORT_VALID | TIME_SYNC_VALID | DIAGNOSTIC_VALID)) |
        STALE);
    for (auto& state : output.status) state &= static_cast<uint8_t>(~FRESH);
    output.vx = output.vy = output.wz = 0.0f;
    stale_ = true;
    last_publish_ = now;
    return true;
  }

 private:
  Sample last_{};
  uint32_t start_ = 0U;
  uint32_t last_rx_ = 0U;
  uint32_t last_publish_ = 0U;
  bool received_ = false;
  bool stale_ = false;
};

Sample Nominal(uint16_t sequence = 7U) {
  Sample sample{};
  sample.time_us = 0xFFFFFFF0ULL;
  sample.sequence = sequence;
  sample.wheel[0] = -1.25f;
  sample.wheel[1] = 2.5f;
  sample.wheel[2] = -3.75f;
  sample.wheel[3] = 4.0f;
  for (auto& state : sample.status) state = ONLINE | FRESH;
  sample.global =
      CHASSIS_POWER_ON | POWER_STATE_VALID | DIAGNOSTIC_VALID | TRANSPORT_VALID;
  sample.vx = 1.2f;
  sample.vy = -0.4f;
  sample.wz = 0.75f;
  return sample;
}

void TestQBoundaries() {
  uint8_t state = FRESH;
  assert(EncodeQ(32767.0f / 256.0f, state) == 32767);
  assert((state & ENCODING_SATURATED) == 0U);
  state = FRESH;
  assert(EncodeQ(128.0f, state) == 32767);
  assert((state & ENCODING_SATURATED) != 0U && (state & FRESH) == 0U);
  state = FRESH;
  assert(EncodeQ(-128.0f, state) == -32768);
  assert((state & ENCODING_SATURATED) == 0U);
  state = FRESH;
  assert(EncodeQ(-128.01f, state) == -32768);
  assert((state & ENCODING_SATURATED) != 0U && (state & FRESH) == 0U);
}

void TestRoundTripOutOfOrderAndTimestampWrap() {
  const auto frames = Encode(Nominal());
  Assembler assembler;
  Sample output{};
  assert(!assembler.PushPair23(frames.pair23, 1U, output));
  assert(!assembler.PushMeta(frames.meta, 2U, output));
  assert(!assembler.PushDiagnostic(frames.diagnostic, 3U, output));
  assert(assembler.PushPair01(frames.pair01, 4U, output));
  assert(output.sequence == 7U);
  assert(output.time_us == 0xFFFFFFF0ULL);
  assert(std::fabs(output.wheel[2] + 3.75f) < 1.0e-6f);
  assert(std::fabs(output.vx - 1.2f) < 1.0e-6f);
  assert(!assembler.PushPair01(frames.pair01, 5U, output));
}

void TestMixedMissingAndExpiry() {
  const auto first = Encode(Nominal(10U));
  const auto second = Encode(Nominal(11U));
  Assembler assembler;
  Sample output{};
  assert(!assembler.PushMeta(first.meta, 100U, output));
  assert(!assembler.PushPair01(second.pair01, 101U, output));
  assert(!assembler.PushPair23(first.pair23, 102U, output));
  assert(!assembler.PushMeta(second.meta, 103U, output));
  assert(assembler.PushPair23(second.pair23, 104U, output));
  assert(output.sequence == 11U);

  Assembler missing;
  assert(!missing.PushMeta(first.meta, 200U, output));
  assert(!missing.PushPair01(first.pair01, 201U, output));
  assert(!missing.Expire(220U));
  assert(missing.Expire(221U));
  assert(!missing.PushPair23(first.pair23, 222U, output));
}

void TestSequenceWrapPowerOffAndWatchdog() {
  const auto old_frames = Encode(Nominal(65535U));
  const auto wrap_frames = Encode(Nominal(0U));
  Assembler assembler;
  Sample output{};
  assert(!assembler.PushMeta(old_frames.meta, 1U, output));
  assert(!assembler.PushPair01(wrap_frames.pair01, 2U, output));
  assert(!assembler.PushMeta(wrap_frames.meta, 3U, output));
  assert(assembler.PushPair23(wrap_frames.pair23, 4U, output));
  assert(output.sequence == 0U);

  auto power_off = Nominal();
  power_off.global = POWER_STATE_VALID | TRANSPORT_VALID;
  for (auto& state : power_off.status) state &= static_cast<uint8_t>(~FRESH);
  const auto power_frames = Encode(power_off);
  Assembler power_assembler;
  assert(!power_assembler.PushMeta(power_frames.meta, 5U, output));
  assert(!power_assembler.PushPair01(power_frames.pair01, 6U, output));
  assert(power_assembler.PushPair23(power_frames.pair23, 7U, output));
  assert((output.global & POWER_STATE_VALID) != 0U);
  assert((output.global & CHASSIS_POWER_ON) == 0U);
  for (const auto state : output.status) assert((state & FRESH) == 0U);

  Watchdog watchdog;
  watchdog.Receive(10U, Nominal());
  assert(!watchdog.Tick(60U, output));
  assert(watchdog.Tick(61U, output));
  assert((output.global & STALE) != 0U);
  assert((output.global & (TRANSPORT_VALID | TIME_SYNC_VALID)) == 0U);
  assert(!watchdog.Tick(160U, output));
  assert(watchdog.Tick(161U, output));
  watchdog.Receive(170U, Nominal(8U));
  assert(!watchdog.Tick(220U, output));
}

}  // namespace

int main() {
  TestQBoundaries();
  TestRoundTripOutOfOrderAndTimestampWrap();
  TestMixedMissingAndExpiry();
  TestSequenceWrapPowerOffAndWatchdog();
}
