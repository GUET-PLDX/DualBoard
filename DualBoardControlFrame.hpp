#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace Pldx::DualBoardControl {

inline constexpr float NORMALIZED_SCALE = 32767.0F;
inline constexpr float SI_SCALE = 1000.0F;
inline constexpr float MAX_VX_MPS = 2.5F;
inline constexpr float MAX_VY_MPS = 2.5F;
inline constexpr float MAX_WZ_RAD_S = 1.8F;
inline constexpr float FORCE_SCALE = 100.0F;
inline constexpr float MAX_FORCE_N = 200.0F;
inline constexpr float MAX_TORQUE_NM = 100.0F;
inline constexpr uint32_t REARM_ZERO_DURATION_MS = 100U;
inline constexpr uint32_t REARM_MAX_HEARTBEAT_GAP_MS = 50U;
inline constexpr uint8_t SI_BIT = 1U << 7U;
inline constexpr uint8_t FORCE_BIT = 1U << 6U;
inline constexpr uint8_t RESERVED_MASK = 0x30U;
inline constexpr uint8_t MODE_MASK = 0x0FU;

enum class Mode : uint8_t {
  RELAX = 0U,
  INDEPENDENT = 1U,
  ROTOR = 2U,
  FOLLOW = 3U,
};

enum class Admission : uint8_t {
  INVALID,
  PROBATION,
  ACCEPTED,
};

struct Command {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  int8_t self_define = 0;
  Mode mode = Mode::RELAX;
  bool si_units = false;
  float force_x_global_n = 0.0F;
  float force_y_global_n = 0.0F;
  float torque_z_global_nm = 0.0F;
  bool force_control = false;
};

using Frame = std::array<uint8_t, 8U>;
using ForceFrame = std::array<uint8_t, 8U>;

struct ProcessResult {
  Command command{};
  Admission admission = Admission::INVALID;
  bool accepted = false;
};

inline bool IsSupportedMode(Mode mode) {
  return static_cast<uint8_t>(mode) <= static_cast<uint8_t>(Mode::FOLLOW);
}

inline bool IsSupportedSelfDefine(int8_t value) {
  return value >= 0 && value <= 2;
}

inline bool IsValidSi(const Command& command) {
  return std::isfinite(command.x) && std::isfinite(command.y) &&
         std::isfinite(command.z) && std::fabs(command.x) <= MAX_VX_MPS &&
         std::fabs(command.y) <= MAX_VY_MPS &&
         std::fabs(command.z) <= MAX_WZ_RAD_S;
}

inline bool IsZero(const Command& command) {
  return command.x == 0.0F && command.y == 0.0F && command.z == 0.0F;
}

inline bool IsValidForce(const Command& command) {
  return std::isfinite(command.force_x_global_n) &&
         std::isfinite(command.force_y_global_n) &&
         std::isfinite(command.torque_z_global_nm) &&
         std::fabs(command.force_x_global_n) <= MAX_FORCE_N &&
         std::fabs(command.force_y_global_n) <= MAX_FORCE_N &&
         std::fabs(command.torque_z_global_nm) <= MAX_TORQUE_NM;
}

inline void StoreSigned(Frame& frame, size_t offset, int16_t value) {
  const auto RAW = static_cast<uint16_t>(value);
  frame[offset] = static_cast<uint8_t>(RAW & 0xFFU);
  frame[offset + 1U] = static_cast<uint8_t>(RAW >> 8U);
}

inline int16_t LoadSigned(const Frame& frame, size_t offset) {
  const uint16_t RAW = static_cast<uint16_t>(frame[offset]) |
                       (static_cast<uint16_t>(frame[offset + 1U]) << 8U);
  return static_cast<int16_t>(RAW);
}

inline bool EncodeForce(const Command& command, ForceFrame& frame,
                        uint8_t sequence) {
  frame = {};
  if (!IsValidForce(command)) return false;
  const float values[3] = {command.force_x_global_n, command.force_y_global_n,
                           command.torque_z_global_nm};
  int16_t encoded[3]{};
  for (size_t index = 0U; index < 3U; ++index) {
    const float scaled = std::round(values[index] * FORCE_SCALE);
    if (scaled < -32767.0F || scaled > 32767.0F) return false;
    encoded[index] = static_cast<int16_t>(scaled);
    const uint16_t raw = static_cast<uint16_t>(encoded[index]);
    frame[index * 2U] = static_cast<uint8_t>(raw & 0xFFU);
    frame[index * 2U + 1U] = static_cast<uint8_t>(raw >> 8U);
  }
  frame[6] = command.force_control ? 1U : 0U;
  frame[7] = sequence;
  return true;
}

inline bool DecodeForce(const ForceFrame& frame, Command& command) {
  const auto load = [&frame](size_t offset) {
    const uint16_t raw = static_cast<uint16_t>(frame[offset]) |
                         (static_cast<uint16_t>(frame[offset + 1U]) << 8U);
    return static_cast<int16_t>(raw);
  };
  if ((frame[6] & 0xFEU) != 0U) return false;
  command.force_x_global_n = static_cast<float>(load(0U)) / FORCE_SCALE;
  command.force_y_global_n = static_cast<float>(load(2U)) / FORCE_SCALE;
  command.torque_z_global_nm = static_cast<float>(load(4U)) / FORCE_SCALE;
  command.force_control = (frame[6] & 1U) != 0U;
  return IsValidForce(command);
}

inline Frame SafeFrame() { return {}; }

inline bool Encode(const Command& command, Frame& frame) {
  frame = SafeFrame();
  if (!IsSupportedMode(command.mode) ||
      !IsSupportedSelfDefine(command.self_define) ||
      !std::isfinite(command.x) || !std::isfinite(command.y) ||
      !std::isfinite(command.z)) {
    return false;
  }

  std::array<int16_t, 3U> encoded{};
  if (command.si_units) {
    if (!IsValidSi(command)) {
      return false;
    }
    const std::array<float, 3U> VALUES{command.x, command.y, command.z};
    for (size_t index = 0U; index < VALUES.size(); ++index) {
      encoded[index] =
          static_cast<int16_t>(std::round(VALUES[index] * SI_SCALE));
    }
  } else {
    const std::array<float, 3U> VALUES{command.x, command.y, command.z};
    for (size_t index = 0U; index < VALUES.size(); ++index) {
      const float CLAMPED = std::clamp(VALUES[index], -1.0F, 1.0F);
      encoded[index] = static_cast<int16_t>(CLAMPED * NORMALIZED_SCALE);
    }
  }

  StoreSigned(frame, 0U, encoded[0]);
  StoreSigned(frame, 2U, encoded[1]);
  StoreSigned(frame, 4U, encoded[2]);
  frame[6] = static_cast<uint8_t>(command.self_define);
  frame[7] = static_cast<uint8_t>(command.mode) |
             (command.si_units ? SI_BIT : 0U) |
             (command.force_control ? FORCE_BIT : 0U);
  return true;
}

inline bool Decode(const Frame& frame, Command& command) {
  command = {};
  const uint8_t MODE_RAW = frame[7];
  if ((MODE_RAW & RESERVED_MASK) != 0U) {
    return false;
  }
  const auto MODE = static_cast<Mode>(MODE_RAW & MODE_MASK);
  const auto SELF_DEFINE = static_cast<int8_t>(frame[6]);
  if (!IsSupportedMode(MODE) || !IsSupportedSelfDefine(SELF_DEFINE)) {
    return false;
  }

  const std::array<int16_t, 3U> RAW{
      LoadSigned(frame, 0U), LoadSigned(frame, 2U), LoadSigned(frame, 4U)};
  for (const auto VALUE : RAW) {
    if (VALUE == std::numeric_limits<int16_t>::min()) {
      return false;
    }
  }

  command.x = static_cast<float>(RAW[0]);
  command.y = static_cast<float>(RAW[1]);
  command.z = static_cast<float>(RAW[2]);
  command.self_define = SELF_DEFINE;
  command.mode = MODE;
  command.si_units = (MODE_RAW & SI_BIT) != 0U;
  command.force_control = (MODE_RAW & FORCE_BIT) != 0U;
  const float SCALE = command.si_units ? SI_SCALE : NORMALIZED_SCALE;
  command.x /= SCALE;
  command.y /= SCALE;
  command.z /= SCALE;
  return !command.si_units || IsValidSi(command);
}

class AdmissionState {
 public:
  ProcessResult Process(const Frame& frame, uint32_t now_ms) {
    Command command{};
    if (!Decode(frame, command)) {
      Reset();
      return {};
    }

    if (admission_ == Admission::ACCEPTED) {
      return {command, admission_, true};
    }

    if (!IsZero(command)) {
      Reset();
      return {};
    }

    if (admission_ != Admission::PROBATION ||
        now_ms - last_zero_ms_ > REARM_MAX_HEARTBEAT_GAP_MS) {
      admission_ = Admission::PROBATION;
      probation_start_ms_ = now_ms;
      last_zero_ms_ = now_ms;
      return {{}, admission_, false};
    }

    last_zero_ms_ = now_ms;
    if (now_ms - probation_start_ms_ < REARM_ZERO_DURATION_MS) {
      return {{}, admission_, false};
    }

    admission_ = Admission::ACCEPTED;
    return {command, admission_, true};
  }

  void Reset() {
    admission_ = Admission::INVALID;
    probation_start_ms_ = 0U;
    last_zero_ms_ = 0U;
  }

  [[nodiscard]] Admission State() const { return admission_; }

 private:
  Admission admission_ = Admission::INVALID;
  uint32_t probation_start_ms_ = 0U;
  uint32_t last_zero_ms_ = 0U;
};

}  // namespace Pldx::DualBoardControl
