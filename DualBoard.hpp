#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 双板定频 CAN 业务帧模块，通过 LibXR Topic 保持上层接口并使用 Classic CAN 固定帧同步双板状态
depends:
- id: GUET-PLDX/CMD
  ref: d9254a5957d641c0dbceda88eea14c001648c554
- id: GUET-PLDX/Chassis
  ref: 1225795a0b621c9293f8817a7a0cc2795c74ffa9
- id: GUET-PLDX/PowerControl
  ref: 5d06d27928970a736283d3f6273383534517c4f9
- id: GUET-PLDX/Referee
  ref: 4eb967e28d76bfb3004ddb03a565b10741f1d888
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <type_traits>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "PowerControl.hpp"
#include "Referee.hpp"
#include "can.hpp"
#include "flag.hpp"
#include "libxr.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "queue/spsc_queue.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

namespace Pldx::DualBoardControl {

inline constexpr float NORMALIZED_SCALE = 32767.0F;
inline constexpr float CHASSIS_VELOCITY_SCALE = 1000.0F;
inline constexpr uint8_t RESERVED_MASK = 0xF0U;
inline constexpr uint8_t MODE_MASK = 0x0FU;
inline constexpr uint32_t USE_CAPACITOR_ID_OFFSET = 0x11U;
inline constexpr char NAV_CHASSIS_MODE_TOPIC[] = "nav_chassis_mode";
inline constexpr char USE_CAPACITOR_TOPIC[] = "use_capacitor";

enum class Mode : uint8_t {
  RELAX = 0U,
  INDEPENDENT = 1U,
  ROTOR = 2U,
  FOLLOW = 3U,
  NAVIGATION = 4U,
  ROTOR_VARIABLE = 5U,
  NAVIGATION_ROTOR = 6U,
  HOLE = 7U,
};

struct PlanarVelocity {
  float vx_mps = 0.0F;
  float vy_mps = 0.0F;
  float wz_rad_s = 0.0F;
};

struct Command {
  float vx = 0.0F;
  float vy = 0.0F;
  float wz = 0.0F;
  int8_t self_define = 0;
  Mode mode = Mode::RELAX;
};

using Frame = std::array<uint8_t, 8U>;

struct __attribute__((packed)) UseCapacitorCommand {
  uint8_t enabled = 1U;
  uint8_t sequence = 0U;
  uint8_t reserved[6]{};
};

static_assert(sizeof(UseCapacitorCommand) == 8U);

inline bool IsSupportedMode(Mode mode) {
  /* 上界比较依赖"新模式追加尾部"纪律；追加枚举时须同步此处与 static_assert 组
   */
  return static_cast<uint8_t>(mode) <= static_cast<uint8_t>(Mode::HOLE);
}

inline bool UsesChassisVelocity(Mode mode) {
  return mode == Mode::NAVIGATION || mode == Mode::NAVIGATION_ROTOR ||
         mode == Mode::HOLE;
}

inline Mode SelectRequestedMode(const CMD::ChassisControl& control,
                                Mode legacy_mode) {
  if (!control.explicit_requests) {
    return legacy_mode;
  }
  switch (control.motion) {
    case CMD::ChassisMotion::FOLLOW:
      return Mode::FOLLOW;
    case CMD::ChassisMotion::ROTOR:
      return Mode::ROTOR;
    default:
      return Mode::RELAX;
  }
}

inline Mode SelectOutputMode(Mode rc_mode, bool nav_valid, Mode nav_mode,
                             bool auto_ctrl, bool ai_chassis_online,
                             bool rc_lost, bool explicit_requests = false) {
  if (rc_lost || (explicit_requests && rc_mode == Mode::RELAX)) {
    return Mode::RELAX;
  }
  if (auto_ctrl && ai_chassis_online && nav_valid) {
    return nav_mode == Mode::HOLE ? Mode::RELAX : nav_mode;
  }
  if (auto_ctrl && explicit_requests) {
    return Mode::RELAX;
  }
  if (auto_ctrl && rc_mode == Mode::HOLE) {
    return Mode::RELAX;
  }
  return rc_mode;
}

struct ConfigurationInput {
  Mode request = Mode::RELAX;
  Mode actual_mode = Mode::RELAX;
  LifterPosition position = LifterPosition::MIDDLE;
  float yaw_rad = 0.0F;
  bool feedback_valid = false;
  uint64_t yaw_sample_us = 0U;
  bool explicit_requests = false;
  bool folded_request = false;
};

struct ConfigurationOutput {
  Mode mode = Mode::RELAX;
  FoldAction fold = FoldAction::HOLD;
  bool folded = false;
};

class ConfigurationControl {
 public:
  ConfigurationOutput Step(const ConfigurationInput& input) {
    const bool supported_request =
        input.explicit_requests
            ? (input.request == Mode::FOLLOW || input.request == Mode::ROTOR)
            : (input.request == Mode::HOLE || input.request == Mode::ROTOR);
    if (!input.feedback_valid || !std::isfinite(input.yaw_rad) ||
        input.yaw_sample_us == 0U || !supported_request) {
      Reset();
      return {};
    }

    const bool folded_request = input.explicit_requests
                                    ? input.folded_request
                                    : input.request == Mode::HOLE;
    if (folded_request || input.position != LifterPosition::TOP) {
      rotor_allowed_ = false;
      const bool aligned = IsSettled(input, Mode::HOLE, HOLE_ANGLE_RAD);
      if (folded_request) {
        return {Mode::HOLE, aligned ? FoldAction::BOTTOM : FoldAction::HOLD,
                aligned && input.position == LifterPosition::BOTTOM};
      }
      return {Mode::HOLE, aligned ? FoldAction::TOP : FoldAction::HOLD};
    }

    if (input.request == Mode::FOLLOW || !rotor_allowed_) {
      rotor_allowed_ = IsSettled(input, Mode::FOLLOW, 0.0F);
    }
    return {rotor_allowed_ ? input.request : Mode::FOLLOW, FoldAction::TOP};
  }

  void Reset() { *this = {}; }

 private:
  static constexpr float HOLE_ANGLE_RAD = std::numbers::pi_v<float> / 4.0F;
  static constexpr float ALIGNMENT_TOLERANCE_RAD = 0.05F;
  static constexpr uint64_t ALIGNMENT_SETTLE_US = 50'000U;

  bool IsSettled(const ConfigurationInput& input, Mode mode, float target_rad) {
    const float error = std::remainder(input.yaw_rad - target_rad,
                                       2.0F * std::numbers::pi_v<float>);
    if (input.actual_mode != mode ||
        std::abs(error) > ALIGNMENT_TOLERANCE_RAD) {
      alignment_mode_ = Mode::RELAX;
      return false;
    }
    if (alignment_mode_ != mode || input.yaw_sample_us < last_yaw_sample_us_ ||
        input.yaw_sample_us - last_yaw_sample_us_ > ALIGNMENT_SETTLE_US) {
      alignment_mode_ = mode;
      alignment_since_us_ = input.yaw_sample_us;
    }
    last_yaw_sample_us_ = input.yaw_sample_us;
    return input.yaw_sample_us - alignment_since_us_ >= ALIGNMENT_SETTLE_US;
  }

  Mode alignment_mode_ = Mode::RELAX;
  uint64_t alignment_since_us_ = 0U;
  uint64_t last_yaw_sample_us_ = 0U;
  bool rotor_allowed_ = false;
};

inline void MapManualHoleVelocity(CMD::ChassisCMD& command,
                                  float max_speed_mps) {
  constexpr float COS_45 = std::numbers::sqrt2_v<float> / 2.0F;
  const float SCALE = max_speed_mps * COS_45;
  const float RIGHT = command.vx;
  const float FORWARD = command.vy;
  command.vx = SCALE * (RIGHT + FORWARD);
  command.vy = SCALE * (FORWARD - RIGHT);
  command.wz = 0.0F;
}

inline Command MakeControlCommand(CMD::ChassisCMD command, Mode mode,
                                  float manual_hole_max_speed_mps) {
  if (mode == Mode::HOLE) {
    MapManualHoleVelocity(command, manual_hole_max_speed_mps);
  }
  return {command.vx, command.vy, command.wz,
          static_cast<int8_t>(command.self_define), mode};
}

inline bool IsSupportedSelfDefine(int8_t value) {
  return value >= 0 && value <= 2;
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

inline bool EncodeUseCapacitor(bool enabled, uint8_t sequence,
                               UseCapacitorCommand& frame) {
  frame = {};
  frame.enabled = enabled ? 1U : 0U;
  frame.sequence = sequence;
  return true;
}

inline bool DecodeUseCapacitor(const UseCapacitorCommand& frame,
                               bool& enabled) {
  if (frame.enabled > 1U) {
    return false;
  }
  for (uint8_t byte : frame.reserved) {
    if (byte != 0U) {
      return false;
    }
  }
  enabled = frame.enabled == 1U;
  return true;
}

inline Frame SafeFrame() { return {}; }

inline bool Encode(const Command& command, Frame& frame) {
  frame = SafeFrame();
  if (!IsSupportedMode(command.mode) ||
      !IsSupportedSelfDefine(command.self_define)) {
    return false;
  }

  const float SCALE = UsesChassisVelocity(command.mode) ? CHASSIS_VELOCITY_SCALE
                                                        : NORMALIZED_SCALE;
  const float LIMIT = 32767.0F / SCALE;
  std::array<int16_t, 3U> encoded{};
  const std::array<float, 3U> VALUES{command.vx, command.vy, command.wz};
  for (size_t index = 0U; index < VALUES.size(); ++index) {
    if (!std::isfinite(VALUES[index])) return false;
    encoded[index] =
        static_cast<int16_t>(std::clamp(VALUES[index], -LIMIT, LIMIT) * SCALE);
  }

  StoreSigned(frame, 0U, encoded[0]);
  StoreSigned(frame, 2U, encoded[1]);
  StoreSigned(frame, 4U, encoded[2]);
  frame[6] = static_cast<uint8_t>(command.self_define);
  frame[7] = static_cast<uint8_t>(command.mode);
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

  command.self_define = SELF_DEFINE;
  command.mode = MODE;
  const float SCALE =
      UsesChassisVelocity(MODE) ? CHASSIS_VELOCITY_SCALE : NORMALIZED_SCALE;
  command.vx = static_cast<float>(RAW[0]) / SCALE;
  command.vy = static_cast<float>(RAW[1]) / SCALE;
  command.wz = static_cast<float>(RAW[2]) / SCALE;
  return true;
}

inline constexpr const char* CHASSIS_MOTION_STATE_TOPIC_NAME =
    "chassis_motion_state";
inline constexpr bool CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER = true;

enum class ChassisMotionMode : uint8_t { NON_ROTOR, ROTOR };

struct ChassisMotionState {
  float yaw_rate_rad_s = 0.0f;
  bool yaw_rate_valid = false;
  bool online = false;
  ChassisMotionMode mode = ChassisMotionMode::NON_ROTOR;
};

/* 底盘轮速正解算的实测车体系速度（与导航 vx/vy/wz 同轴同号），chassis→gimbal。
 */
inline constexpr uint8_t CHASSIS_VELOCITY_FLAG_VALID = 0x01U;
inline constexpr uint8_t CHASSIS_VELOCITY_WHEEL_SHIFT =
    1U; /* bit1..4 wheel0..3 */
inline constexpr uint8_t CHASSIS_VELOCITY_MODE_SHIFT = 5U; /* bit5..7 mode */

struct __attribute__((packed)) ChassisVelocityFrame {
  int16_t vx_q;
  int16_t vy_q;
  int16_t wz_q;
  uint8_t sequence;
  uint8_t flags;
};
static_assert(sizeof(ChassisVelocityFrame) == 8U);

struct ChassisVelocity {
  PlanarVelocity velocity{};
  bool valid = false;
  bool wheel_online[4]{};
  uint8_t mode = 0U;
  uint8_t sequence = 0U;
};

inline bool EncodeChassisVelocity(const ChassisVelocity& input,
                                  ChassisVelocityFrame& frame) {
  frame = {};
  frame.sequence = input.sequence;
  uint8_t flags =
      static_cast<uint8_t>((input.mode & 0x07U) << CHASSIS_VELOCITY_MODE_SHIFT);
  for (uint8_t i = 0; i < 4U; i++) {
    if (input.wheel_online[i]) {
      flags |= static_cast<uint8_t>(1U << (CHASSIS_VELOCITY_WHEEL_SHIFT + i));
    }
  }
  const std::array<float, 3U> VALUES{
      input.velocity.vx_mps, input.velocity.vy_mps, input.velocity.wz_rad_s};
  constexpr float LIMIT =
      static_cast<float>(std::numeric_limits<int16_t>::max()) /
      CHASSIS_VELOCITY_SCALE;
  bool encodable = input.valid;
  for (float value : VALUES) {
    encodable = encodable && std::isfinite(value) && std::fabs(value) <= LIMIT;
  }
  if (encodable) {
    frame.vx_q =
        static_cast<int16_t>(std::lround(VALUES[0] * CHASSIS_VELOCITY_SCALE));
    frame.vy_q =
        static_cast<int16_t>(std::lround(VALUES[1] * CHASSIS_VELOCITY_SCALE));
    frame.wz_q =
        static_cast<int16_t>(std::lround(VALUES[2] * CHASSIS_VELOCITY_SCALE));
    flags |= CHASSIS_VELOCITY_FLAG_VALID;
  }
  frame.flags = flags;
  return encodable;
}

inline ChassisVelocity DecodeChassisVelocity(
    const ChassisVelocityFrame& frame) {
  ChassisVelocity output{};
  output.sequence = frame.sequence;
  output.mode =
      static_cast<uint8_t>(frame.flags >> CHASSIS_VELOCITY_MODE_SHIFT);
  for (uint8_t i = 0; i < 4U; i++) {
    output.wheel_online[i] =
        (frame.flags & (1U << (CHASSIS_VELOCITY_WHEEL_SHIFT + i))) != 0U;
  }
  output.valid = (frame.flags & CHASSIS_VELOCITY_FLAG_VALID) != 0U;
  if (output.valid) {
    output.velocity.vx_mps =
        static_cast<float>(frame.vx_q) / CHASSIS_VELOCITY_SCALE;
    output.velocity.vy_mps =
        static_cast<float>(frame.vy_q) / CHASSIS_VELOCITY_SCALE;
    output.velocity.wz_rad_s =
        static_cast<float>(frame.wz_q) / CHASSIS_VELOCITY_SCALE;
  }
  return output;
}

/* 8 位帧序号扩展为单调 32 位计数；首帧或回绕均按最小正差推进。 */
inline uint32_t ExtendSequence(uint32_t previous, uint8_t sequence,
                               bool has_previous) {
  if (!has_previous) {
    return sequence;
  }
  const uint8_t delta =
      static_cast<uint8_t>(sequence - static_cast<uint8_t>(previous));
  return previous + delta;
}

/* 导航力前馈，gimbal→chassis。与导航 vx/vy 同一坐标系，力 ×100（±327 N），力矩
 * ×100。 */
inline constexpr float CHASSIS_FF_SCALE = 100.0F;
inline constexpr uint8_t CHASSIS_FF_FLAG_VALID = 0x01U;

struct __attribute__((packed)) ChassisFeedforwardFrame {
  int16_t fx_q;
  int16_t fy_q;
  int16_t mz_q;
  uint8_t sequence;
  uint8_t flags;
};
static_assert(sizeof(ChassisFeedforwardFrame) == 8U);

/* 无效或超出量程时整帧置零并清有效位（不截断：超量程说明上游已出错）。 */
inline bool EncodeChassisFeedforward(const CMD::ChassisCMD& input,
                                     uint8_t sequence,
                                     ChassisFeedforwardFrame& frame) {
  frame = {};
  frame.sequence = sequence;
  constexpr float LIMIT =
      static_cast<float>(std::numeric_limits<int16_t>::max()) /
      CHASSIS_FF_SCALE;
  const std::array<float, 3U> VALUES{input.fx_n, input.fy_n, input.mz_nm};
  bool encodable = input.ff_valid;
  for (float value : VALUES) {
    encodable = encodable && std::isfinite(value) && std::fabs(value) <= LIMIT;
  }
  if (!encodable) {
    return false;
  }
  frame.fx_q = static_cast<int16_t>(std::lround(VALUES[0] * CHASSIS_FF_SCALE));
  frame.fy_q = static_cast<int16_t>(std::lround(VALUES[1] * CHASSIS_FF_SCALE));
  frame.mz_q = static_cast<int16_t>(std::lround(VALUES[2] * CHASSIS_FF_SCALE));
  frame.flags = CHASSIS_FF_FLAG_VALID;
  return true;
}

inline CMD::ChassisCMD DecodeChassisFeedforward(
    const ChassisFeedforwardFrame& frame) {
  CMD::ChassisCMD output{};
  output.ff_valid = (frame.flags & CHASSIS_FF_FLAG_VALID) != 0U;
  if (output.ff_valid) {
    output.fx_n = static_cast<float>(frame.fx_q) / CHASSIS_FF_SCALE;
    output.fy_n = static_cast<float>(frame.fy_q) / CHASSIS_FF_SCALE;
    output.mz_nm = static_cast<float>(frame.mz_q) / CHASSIS_FF_SCALE;
  }
  return output;
}

struct ChassisCommandAssembly {
  CMD::ChassisCMD command{};
  uint32_t motion_time_ms = 0U;
  uint32_t ff_time_ms = 0U;
  bool motion_seen = false;

  void Motion(const Command& input, uint32_t now) {
    command.vx = input.vx;
    command.vy = input.vy;
    command.wz = input.wz;
    command.self_define = static_cast<CMD::ChasStat>(input.self_define);
    motion_time_ms = now;
    motion_seen = true;
    Expire(now);
  }

  void Feedforward(const CMD::ChassisCMD& input, uint32_t now) {
    command.fx_n = input.fx_n;
    command.fy_n = input.fy_n;
    command.mz_nm = input.mz_nm;
    command.ff_valid = input.ff_valid;
    ff_time_ms = now;
  }

  bool Expire(uint32_t now) {
    if (!command.ff_valid || now - ff_time_ms <= 30U) return false;
    command.fx_n = command.fy_n = command.mz_nm = 0.0F;
    command.ff_valid = false;
    return true;
  }

  bool MotionFresh(uint32_t now) const {
    return motion_seen && now - motion_time_ms <= 100U;
  }
};

/* gimbal → 导航主机（navigation CDC，topic "chassis_feedback"）。
 * 布局须与 ros2_libxr wire.hpp 的 ChassisFeedback 一致。 */
inline constexpr const char* CHASSIS_FEEDBACK_TOPIC = "chassis_feedback";
inline constexpr uint8_t CHASSIS_FEEDBACK_FLAG_VALID = 0x01U;
inline constexpr uint8_t CHASSIS_FEEDBACK_WHEEL_SHIFT = 1U; /* bit1..4 */
inline constexpr uint8_t CHASSIS_FEEDBACK_FLAG_LINK = 0x20U;
inline constexpr uint8_t CHASSIS_FEEDBACK_FLAG_GYRO_VALID = 0x40U;

struct __attribute__((packed)) ChassisFeedback {
  float vx_mps; /* 轮速正解算，车体系 */
  float vy_mps;
  float wz_rad_s;     /* 轮速正解算的偏航角速度 */
  float gyro_z_rad_s; /* 底盘 IMU 陀螺 z（最近一次 MotionFrame） */
  uint32_t sequence; /* 由 CAN 8 位序号扩展；跳号表示 CAN/USB 丢包 */
  uint32_t stamp_us; /* 云台板收到 CAN 帧时的 µs 时钟低 32 位 */
  uint8_t mode;      /* 底盘实际模式（DualBoardControl::Mode 线值） */
  // bit0 valid; bit1..4 wheel0..3; bit5 link; bit6 gyro valid.
  uint8_t flags;
};
static_assert(sizeof(ChassisFeedback) == 26U);
static_assert(offsetof(ChassisFeedback, sequence) == 16U);
static_assert(offsetof(ChassisFeedback, stamp_us) == 20U);
static_assert(offsetof(ChassisFeedback, flags) == 25U);

}  // namespace Pldx::DualBoardControl

class RefereeCanCodec {
 public:
  static constexpr uint32_t GAME_STATUS_ID_OFFSET = 0x02U;
  static constexpr uint32_t FIELD_EVENT_ID_OFFSET = 0x23U;
  static constexpr uint32_t ROBOT_HP_ID_OFFSET = 0x25U;
  static constexpr uint32_t ROBOT_STATUS_ID_OFFSET = 0x07U;
  static constexpr uint32_t POWER_HEAT_ID_OFFSET = 0x0AU;
  static constexpr uint32_t ROBOT_BUFF_ID_OFFSET = 0x0CU;
  static constexpr uint32_t BULLET_REMAIN_ID_OFFSET = 0x0EU;
  static constexpr uint32_t RFID_ID_OFFSET = 0x03U;
  static constexpr uint32_t ROBOT_DAMAGE_ID_OFFSET = 0x13U;
  // Position payload uses six dedicated classic-CAN frames (40 bytes).
  static constexpr uint32_t ROBOT_POS_ID_OFFSET = 0x14U;
  static constexpr uint32_t SENTRY_INFO_ID_OFFSET = 0x21U;
  static constexpr uint32_t LINK_STATUS_ID_OFFSET = 0x1DU;
  static constexpr uint8_t LAYOUT_VERSION = 2U;
  static constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 20U;
  static constexpr size_t FRAGMENT_DATA_SIZE = 7U;
  // RobotPosForSentry is ten IEEE-754 floats (40 bytes); leave room for the
  // complete payload while retaining the seven-byte data area per CAN frame.
  static constexpr size_t MAX_DATA_SIZE = 42U;

  struct __attribute__((packed)) FragmentFrame {
    uint8_t sequence = 0U;
    uint8_t data[FRAGMENT_DATA_SIZE]{};
  };

  struct Assembly {
    uint8_t sequence = 0U;
    uint8_t received_mask = 0U;
    uint8_t published_sequence = 0U;
    uint32_t last_update_ms = 0U;
    uint8_t data[MAX_DATA_SIZE]{};
    bool active = false;
    bool has_published = false;
  };

  enum class PushResult : uint8_t {
    INVALID,
    INCOMPLETE,
    COMPLETE,
    DUPLICATE,
  };

  template <typename Data>
  static constexpr size_t FragmentCount() {
    return (sizeof(Data) + FRAGMENT_DATA_SIZE - 1U) / FRAGMENT_DATA_SIZE;
  }

  // Check actual arbitration IDs from both directions of the shared CAN3 bus.
  // Relative offsets alone miss collisions when the two bases differ by one.
  static constexpr bool HasUniqueCanIds(uint32_t chassis_base = 0x311U,
                                        uint32_t gimbal_base = 0x312U) {
    if (chassis_base > 0x7ffU || gimbal_base > 0x7ffU) return false;
    std::array<uint32_t, 64U> ids{};
    size_t count = 0U;
    const auto add = [&](uint32_t base, uint32_t offset, size_t length = 1U) {
      for (size_t i = 0U; i < length; ++i) ids[count++] = base + offset + i;
    };
    add(chassis_base, GAME_STATUS_ID_OFFSET);
    add(chassis_base, FIELD_EVENT_ID_OFFSET,
        FragmentCount<Referee::FieldEvents>());
    add(chassis_base, ROBOT_HP_ID_OFFSET, FragmentCount<Referee::RobotHP>());
    add(chassis_base, ROBOT_STATUS_ID_OFFSET,
        FragmentCount<Referee::RobotStatus>());
    add(chassis_base, POWER_HEAT_ID_OFFSET,
        FragmentCount<Referee::PowerHeat>());
    add(chassis_base, ROBOT_BUFF_ID_OFFSET,
        FragmentCount<Referee::RobotBuff>());
    add(chassis_base, BULLET_REMAIN_ID_OFFSET,
        FragmentCount<Referee::BulletRemain>());
    add(chassis_base, RFID_ID_OFFSET, FragmentCount<Referee::RFID>());
    add(chassis_base, ROBOT_DAMAGE_ID_OFFSET,
        FragmentCount<Referee::RobotDamage>());
    add(chassis_base, ROBOT_POS_ID_OFFSET,
        FragmentCount<Referee::RobotPosForSentry>());
    add(chassis_base, SENTRY_INFO_ID_OFFSET,
        FragmentCount<Referee::SentryInfo>());
    add(chassis_base, LINK_STATUS_ID_OFFSET);
    for (uint32_t offset : {0x00U, 0x10U, 0x1aU, 0x1eU, 0x1fU})
      add(chassis_base, offset);
    for (uint32_t offset :
         {0x00U, 0x10U, Pldx::DualBoardControl::USE_CAPACITOR_ID_OFFSET, 0x1aU,
          0x1fU})
      add(gimbal_base, offset);
    for (size_t i = 0U; i < count; ++i) {
      if (ids[i] > 0x7ffU) return false;
      for (size_t j = i + 1U; j < count; ++j)
        if (ids[i] == ids[j]) return false;
    }
    return true;
  }

  template <typename Data>
  static auto Encode(uint8_t sequence, const Data& data) {
    static_assert(std::is_trivially_copyable_v<Data>);
    static_assert(sizeof(Data) <= MAX_DATA_SIZE);
    constexpr size_t FRAGMENT_COUNT =
        (sizeof(Data) + FRAGMENT_DATA_SIZE - 1U) / FRAGMENT_DATA_SIZE;
    std::array<FragmentFrame, FRAGMENT_COUNT> frames{};
    const auto* bytes = reinterpret_cast<const uint8_t*>(&data);
    for (size_t index = 0U; index < frames.size(); ++index) {
      frames[index].sequence = sequence;
      const size_t byte_offset = index * FRAGMENT_DATA_SIZE;
      const size_t byte_count = std::min(
          FRAGMENT_DATA_SIZE, static_cast<size_t>(sizeof(Data) - byte_offset));
      LibXR::Memory::FastCopy(frames[index].data, bytes + byte_offset,
                              byte_count);
    }
    return frames;
  }

  template <typename Data>
  static PushResult Push(Assembly& assembly, size_t fragment_index,
                         const FragmentFrame& frame, uint32_t now_ms,
                         Data& output) {
    static_assert(std::is_trivially_copyable_v<Data>);
    static_assert(sizeof(Data) <= MAX_DATA_SIZE);
    constexpr size_t FRAGMENT_COUNT =
        (sizeof(Data) + FRAGMENT_DATA_SIZE - 1U) / FRAGMENT_DATA_SIZE;
    static_assert(FRAGMENT_COUNT <= 8U);
    if (fragment_index >= FRAGMENT_COUNT) {
      return PushResult::INVALID;
    }

    if (assembly.has_published && assembly.published_sequence == frame.sequence)
      return PushResult::DUPLICATE;
    Expire(assembly, now_ms);
    if (!assembly.active || assembly.sequence != frame.sequence) {
      const bool has_published = assembly.has_published;
      const uint8_t published_sequence = assembly.published_sequence;
      assembly = {};
      assembly.has_published = has_published;
      assembly.published_sequence = published_sequence;
      assembly.active = true;
      assembly.sequence = frame.sequence;
    }
    assembly.last_update_ms = now_ms;

    const size_t byte_offset = fragment_index * FRAGMENT_DATA_SIZE;
    const size_t byte_count = std::min(
        FRAGMENT_DATA_SIZE, static_cast<size_t>(sizeof(Data) - byte_offset));
    const uint8_t fragment_bit = static_cast<uint8_t>(1U << fragment_index);
    if ((assembly.received_mask & fragment_bit) != 0U &&
        std::memcmp(assembly.data + byte_offset, frame.data, byte_count) != 0) {
      // The same sequence must describe one immutable payload.
      assembly.active = false;
      assembly.received_mask = 0U;
      // Quarantine the conflicting sequence: later fragments must not assemble
      // a mixture of the two payloads advertised under that same sequence.
      assembly.has_published = true;
      assembly.published_sequence = frame.sequence;
      return PushResult::INVALID;
    }
    LibXR::Memory::FastCopy(assembly.data + byte_offset, frame.data,
                            byte_count);
    assembly.received_mask |= static_cast<uint8_t>(1U << fragment_index);
    constexpr uint8_t EXPECTED_MASK =
        static_cast<uint8_t>((1U << FRAGMENT_COUNT) - 1U);
    if (assembly.received_mask != EXPECTED_MASK) {
      return PushResult::INCOMPLETE;
    }
    if (assembly.has_published &&
        assembly.published_sequence == frame.sequence) {
      return PushResult::DUPLICATE;
    }

    LibXR::Memory::FastCopy(&output, assembly.data, sizeof(Data));
    assembly.has_published = true;
    assembly.published_sequence = frame.sequence;
    return PushResult::COMPLETE;
  }

  static bool Expire(Assembly& assembly, uint32_t now_ms) {
    if (!assembly.active ||
        now_ms - assembly.last_update_ms <= REASSEMBLY_TIMEOUT_MS) {
      return false;
    }
    assembly.active = false;
    assembly.received_mask = 0U;
    return true;
  }

  static constexpr uint16_t IntersectValidity(bool online, uint16_t local_mask,
                                              uint16_t upstream_mask,
                                              uint16_t supported_mask) {
    return online ? static_cast<uint16_t>(local_mask & upstream_mask &
                                          supported_mask)
                  : 0U;
  }
};

static_assert(sizeof(RefereeCanCodec::FragmentFrame) == 8U);
static_assert(RefereeCanCodec::HasUniqueCanIds(),
              "default CAN3 IDs must be unique");

struct __attribute__((packed)) SentryDecisionFrame {
  uint8_t version;
  uint8_t sequence;
  uint8_t valid_mask;
  uint8_t state;
  uint16_t buy_bullet_delta;
  uint8_t remote_request_counts;
  uint8_t flags;
};

static_assert(sizeof(SentryDecisionFrame) == 8U);

namespace SentryDecision {

constexpr uint8_t VERSION = 1U;
constexpr uint8_t STATE_VALID = 0x01U;
constexpr uint8_t BUY_BULLET_VALID = 0x02U;
constexpr uint8_t REMOTE_BULLET_VALID = 0x04U;
constexpr uint8_t REMOTE_HP_VALID = 0x08U;
constexpr uint8_t BUY_RESURRECTION_VALID = 0x10U;
constexpr uint8_t KNOWN_VALID_MASK = 0x1fU;
constexpr uint8_t BUY_RESURRECTION_FLAG = 0x01U;
constexpr uint32_t RX_TIMEOUT_MS = 100U;
constexpr uint32_t RETRY_PERIOD_MS = 10U;
constexpr uint8_t REQUIRED_SEND_SUCCESSES = 5U;

constexpr uint8_t PackRemoteCounts(uint8_t bullet, uint8_t hp) {
  return static_cast<uint8_t>((bullet & 0x0fU) | ((hp & 0x0fU) << 4U));
}

constexpr uint8_t RemoteBulletCount(uint8_t value) { return value & 0x0fU; }

constexpr uint8_t RemoteHpCount(uint8_t value) {
  return static_cast<uint8_t>((value >> 4U) & 0x0fU);
}

enum class UpdateKind : uint8_t {
  BUY_BULLET,
  REMOTE_BUY_BULLET,
  REMOTE_BUY_HP,
  BUY_RESURRECTION,
  STATE,
};

inline void accumulate(SentryDecisionFrame& pending, UpdateKind kind,
                       uint16_t value) {
  switch (kind) {
    case UpdateKind::BUY_BULLET: {
      const uint32_t TOTAL = pending.buy_bullet_delta + value;
      pending.buy_bullet_delta = static_cast<uint16_t>(std::min(TOTAL, 2047U));
      pending.valid_mask |= BUY_BULLET_VALID;
      break;
    }
    case UpdateKind::REMOTE_BUY_BULLET: {
      const uint8_t BULLET_COUNT = static_cast<uint8_t>(std::min<uint16_t>(
          RemoteBulletCount(pending.remote_request_counts) + value, 15U));
      const uint8_t HP_COUNT = RemoteHpCount(pending.remote_request_counts);
      pending.remote_request_counts = PackRemoteCounts(BULLET_COUNT, HP_COUNT);
      pending.valid_mask |= REMOTE_BULLET_VALID;
      break;
    }
    case UpdateKind::REMOTE_BUY_HP: {
      const uint8_t BULLET_COUNT =
          RemoteBulletCount(pending.remote_request_counts);
      const uint8_t HP_COUNT = static_cast<uint8_t>(std::min<uint16_t>(
          RemoteHpCount(pending.remote_request_counts) + value, 15U));
      pending.remote_request_counts = PackRemoteCounts(BULLET_COUNT, HP_COUNT);
      pending.valid_mask |= REMOTE_HP_VALID;
      break;
    }
    case UpdateKind::BUY_RESURRECTION:
      if (value != 0U) {
        pending.flags |= BUY_RESURRECTION_FLAG;
      } else {
        pending.flags &= static_cast<uint8_t>(~BUY_RESURRECTION_FLAG);
      }
      pending.valid_mask |= BUY_RESURRECTION_VALID;
      break;
    case UpdateKind::STATE:
      pending.state = static_cast<uint8_t>(value);
      pending.valid_mask |= STATE_VALID;
      break;
  }
}

inline bool Validate(const SentryDecisionFrame& frame) {
  if (frame.version != VERSION ||
      (frame.valid_mask & static_cast<uint8_t>(~KNOWN_VALID_MASK)) != 0U ||
      (frame.flags & static_cast<uint8_t>(~BUY_RESURRECTION_FLAG)) != 0U) {
    return false;
  }

  if ((frame.valid_mask & STATE_VALID) != 0U) {
    if (frame.state < 1U || frame.state > 3U) {
      return false;
    }
  } else if (frame.state != 0U) {
    return false;
  }

  if ((frame.valid_mask & BUY_BULLET_VALID) != 0U) {
    if (frame.buy_bullet_delta < 1U || frame.buy_bullet_delta > 2047U) {
      return false;
    }
  } else if (frame.buy_bullet_delta != 0U) {
    return false;
  }

  if ((frame.valid_mask & REMOTE_BULLET_VALID) == 0U &&
      RemoteBulletCount(frame.remote_request_counts) != 0U) {
    return false;
  }
  if ((frame.valid_mask & REMOTE_HP_VALID) == 0U &&
      RemoteHpCount(frame.remote_request_counts) != 0U) {
    return false;
  }
  if ((frame.valid_mask & BUY_RESURRECTION_VALID) == 0U && frame.flags != 0U) {
    return false;
  }

  return true;
}

class SequenceTracker {
 public:
  bool Accept(uint8_t sequence, uint32_t now_ms) {
    if (valid_ && sequence == last_sequence_ &&
        (LibXR::MillisecondTimestamp(now_ms) - last_rx_ms_).ToMillisecond() <=
            RX_TIMEOUT_MS) {
      ObserveDuplicate(now_ms);
      return false;
    }

    valid_ = true;
    last_sequence_ = sequence;
    last_rx_ms_ = now_ms;
    return true;
  }

  void ObserveDuplicate(uint32_t now_ms) { last_rx_ms_ = now_ms; }

 private:
  bool valid_ = false;
  uint8_t last_sequence_ = 0U;
  LibXR::MillisecondTimestamp last_rx_ms_{};
};

class RetryController {
 public:
  bool Begin(const SentryDecisionFrame& frame, uint8_t sequence,
             uint32_t now_ms) {
    if (active_) {
      return false;
    }

    frame_ = frame;
    frame_.sequence = sequence;
    active_ = true;
    successes_ = 0U;
    next_send_ms_ = now_ms;
    return true;
  }

  bool Due(uint32_t now_ms) const {
    /* 回绕后的差值落在前半周期内视为已到期 */
    return active_ && (LibXR::MillisecondTimestamp(now_ms) - next_send_ms_)
                              .ToMillisecond() < DUE_WINDOW_MS;
  }

  void OnSendResult(bool sent, uint32_t now_ms) {
    if (!active_) {
      return;
    }

    if (sent) {
      ++successes_;
      if (successes_ >= REQUIRED_SEND_SUCCESSES) {
        active_ = false;
        return;
      }
    }
    next_send_ms_ = now_ms + RETRY_PERIOD_MS;
  }

  bool Active() const { return active_; }
  uint8_t Successes() const { return successes_; }
  const SentryDecisionFrame& Frame() const { return frame_; }

 private:
  static constexpr uint32_t DUE_WINDOW_MS = 0x80000000U;

  SentryDecisionFrame frame_{};
  bool active_ = false;
  uint8_t successes_ = 0U;
  LibXR::MillisecondTimestamp next_send_ms_{};
};

}  // namespace SentryDecision

using Pldx::DualBoardControl::CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER;
using Pldx::DualBoardControl::CHASSIS_MOTION_STATE_TOPIC_NAME;
using Pldx::DualBoardControl::ChassisMotionMode;
using Pldx::DualBoardControl::ChassisMotionState;

/**
 * @brief 双板角色。
 * @note 使用全局枚举是为了让类模板保持 `DualBoard<ROLE, ChassisType>` 形式。
 */
enum class DualBoardRole : uint8_t {
  GIMBAL,
  CHASSIS,
};

template <DualBoardRole ROLE, typename ChassisType = Omni>
class DualBoard {
 public:
  using ChassisMode = typename ChassisType::ChassisMode;

  DualBoard(
      LibXR::CAN& can, CMD* cmd, PowerControl* power_control,
      uint32_t tx_id = 786, uint32_t rx_id = 785, uint32_t rx_buffer_size = 256,
      uint32_t offline_timeout_ms = 100,
      Chassis<ChassisType>* chassis = nullptr,
      const char* mode_topic_name = "dualboard_chassis_mode",
      const char* sentry_buy_bullet_num_topic_name = "sentry_buy_bullet_num",
      const char* sentry_remote_buy_bullet_times_topic_name =
          "sentry_remote_buy_bullet_times",
      const char* sentry_remote_buy_hp_times_topic_name =
          "sentry_remote_buy_hp_times",
      const char* sentry_buy_resurrection_topic_name =
          "sentry_buy_resurrection",
      const char* sentry_state_topic_name = "sentry_state",
      const char* chassis_euler_topic_name = "chassis_euler",
      float manual_hole_max_speed_mps = 0.0F,
      bool manual_configuration_interlock = false)
      : can_(&can),
        tx_id_(tx_id),
        rx_id_(rx_id),
        offline_timeout_ms_(offline_timeout_ms),
        chassis_(chassis),
        cmd_(cmd),
        mode_topic_name_(mode_topic_name),
        sentry_buy_bullet_num_topic_name_(sentry_buy_bullet_num_topic_name),
        sentry_remote_buy_bullet_times_topic_name_(
            sentry_remote_buy_bullet_times_topic_name),
        sentry_remote_buy_hp_times_topic_name_(
            sentry_remote_buy_hp_times_topic_name),
        sentry_buy_resurrection_topic_name_(sentry_buy_resurrection_topic_name),
        sentry_state_topic_name_(sentry_state_topic_name),
        chassis_euler_topic_name_(chassis_euler_topic_name),
        manual_hole_max_speed_mps_(manual_hole_max_speed_mps),
        manual_configuration_interlock_(manual_configuration_interlock),
        power_control_(power_control),
        rx_frames_(rx_buffer_size) {
    ASSERT(rx_buffer_size > 0U);
    REQUIRE(RefereeCanCodec::HasUniqueCanIds(
        ROLE == DualBoardRole::CHASSIS ? tx_id_ : rx_id_,
        ROLE == DualBoardRole::CHASSIS ? rx_id_ : tx_id_));
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      ASSERT(chassis_ != nullptr);
    } else {
      UNUSED(chassis_);
    }

    RegisterRoleTopics();
    RegisterModeEvent();
    RegisterCmdEvent();
    RegisterCanRx();

    thread_.Create(this, ThreadEntry, "DualBoard", 1536,
                   LibXR::Thread::Priority::MEDIUM);
  }

  void OnMonitor() {}

  LibXR::Event& GetEvent() { return dual_board_event_; }

  bool IsOnline() const { return online_; }

  bool IsRefereeLayoutCompatible() const {
    if constexpr (ROLE == DualBoardRole::GIMBAL)
      return state_.referee_layout_compatible_;
    return true;
  }

 private:
  using ControlFrame = Pldx::DualBoardControl::Frame;

  struct __attribute__((packed)) AngleFrame {
    int16_t yaw;
    int16_t pitch;
    uint8_t sequence;
    uint8_t reserved0;
    uint8_t reserved1;
    uint8_t reserved2;
  };

  struct __attribute__((packed)) MotionFrame {
    int16_t gyro_z_q;
    uint8_t gyro_valid;
    uint8_t reserved[5];
  };

  struct __attribute__((packed)) ChassisYawFrame {
    int16_t yaw_q;
    uint8_t valid;
    uint8_t sequence;
    uint32_t sample_time_ms;
  };

  struct __attribute__((packed)) CapacitorFrame {
    uint8_t capacity_percent;
    uint8_t valid;
    uint8_t sequence;
    uint8_t reserved;
    uint32_t sample_time_ms;
  };

  struct __attribute__((packed)) LauncherFeedbackFrame {
    uint16_t heat_limit;
    uint16_t cooling_rate;
    uint16_t heat;
    uint8_t bullet_speed_deci;
    uint8_t robot_level;
  };

  using RefereeFragmentFrame = RefereeCanCodec::FragmentFrame;

  struct __attribute__((packed)) RefereeLinkStatusFrame {
    uint8_t sequence;
    uint8_t referee_online;
    uint16_t supported_mask;
    uint16_t valid_mask;
    uint8_t layout_version;
    uint8_t reserved;
  };

  struct __attribute__((packed)) RefereeGameStatusData {
    uint8_t game_type;
    uint8_t game_progress;
    uint16_t stage_remain_time;
  };

  using RefereeAssembly = RefereeCanCodec::Assembly;

  static constexpr uint32_t CONTROL_PERIOD_MS = 10;
  static constexpr uint32_t LAUNCHER_FEEDBACK_PERIOD_MS = 20;
  static constexpr uint32_t PROTOCOL_THREAD_PERIOD_MS = 2;
  static constexpr uint32_t CONTROL_ID_OFFSET = 0x00U;
  static constexpr uint32_t ANGLE_ID_OFFSET = 0x10U;
  static constexpr uint32_t USE_CAPACITOR_ID_OFFSET =
      Pldx::DualBoardControl::USE_CAPACITOR_ID_OFFSET;
  static constexpr uint16_t DECISION_ID_OFFSET = 0x1fU;
  // Bottom-board yaw is sent on 0x330 (chassis tx_id 0x311 + 0x1f).
  static constexpr uint32_t CHASSIS_YAW_ID_OFFSET = 0x1FU;
  static constexpr uint32_t CAPACITOR_ID_OFFSET = 0x1EU;
  static constexpr uint32_t CHASSIS_YAW_TIMEOUT_MS = 100U;
  static constexpr uint32_t CAPACITOR_TIMEOUT_MS = 150U;
  // Measured chassis velocity on 0x32B (chassis tx_id 0x311 + 0x1a), 250 Hz.
  static constexpr uint32_t CHASSIS_VELOCITY_ID_OFFSET = 0x1AU;
  static constexpr uint32_t CHASSIS_VELOCITY_PERIOD_MS = 4U;
  static constexpr uint32_t CHASSIS_VELOCITY_SAMPLE_TIMEOUT_US = 10000U;
  static constexpr uint32_t CHASSIS_VELOCITY_TIMEOUT_MS = 100U;
  // Navigation force feedforward on 0x32C (gimbal tx_id 0x312 + 0x1a), 100 Hz.
  static constexpr uint32_t CHASSIS_FF_ID_OFFSET = 0x1AU;
  static constexpr uint32_t CHASSIS_FF_PERIOD_MS = 10U;
  static constexpr uint32_t CHASSIS_FF_SOURCE_TIMEOUT_MS = 30U;
  static constexpr uint32_t REFEREE_GAME_STATUS_ID_OFFSET =
      RefereeCanCodec::GAME_STATUS_ID_OFFSET;
  static constexpr uint32_t REFEREE_FIELD_EVENT_ID_OFFSET =
      RefereeCanCodec::FIELD_EVENT_ID_OFFSET;
  static constexpr uint32_t REFEREE_ROBOT_HP_ID_OFFSET =
      RefereeCanCodec::ROBOT_HP_ID_OFFSET;
  static constexpr uint32_t REFEREE_ROBOT_STATUS_ID_OFFSET =
      RefereeCanCodec::ROBOT_STATUS_ID_OFFSET;
  static constexpr uint32_t REFEREE_POWER_HEAT_ID_OFFSET =
      RefereeCanCodec::POWER_HEAT_ID_OFFSET;
  static constexpr uint32_t REFEREE_ROBOT_BUFF_ID_OFFSET =
      RefereeCanCodec::ROBOT_BUFF_ID_OFFSET;
  static constexpr uint32_t REFEREE_BULLET_REMAIN_ID_OFFSET =
      RefereeCanCodec::BULLET_REMAIN_ID_OFFSET;
  static constexpr uint32_t REFEREE_RFID_ID_OFFSET =
      RefereeCanCodec::RFID_ID_OFFSET;
  static constexpr uint32_t REFEREE_ROBOT_DAMAGE_ID_OFFSET =
      RefereeCanCodec::ROBOT_DAMAGE_ID_OFFSET;
  static constexpr uint32_t REFEREE_ROBOT_POS_ID_OFFSET =
      RefereeCanCodec::ROBOT_POS_ID_OFFSET;
  static constexpr uint32_t REFEREE_SENTRY_INFO_ID_OFFSET =
      RefereeCanCodec::SENTRY_INFO_ID_OFFSET;
  static constexpr uint32_t REFEREE_SENTRY_INFO_FRAGMENT_COUNT =
      (sizeof(Referee::SentryInfo) + RefereeCanCodec::FRAGMENT_DATA_SIZE - 1U) /
      RefereeCanCodec::FRAGMENT_DATA_SIZE;
  static constexpr uint32_t REFEREE_LINK_STATUS_ID_OFFSET =
      RefereeCanCodec::LINK_STATUS_ID_OFFSET;
  static constexpr uint32_t REFEREE_STATUS_PERIOD_MS = 1000U;
  static constexpr uint32_t DECISION_DROP_LOG_PERIOD_MS = 1000U;
  static constexpr size_t DECISION_UPDATE_QUEUE_CAPACITY = 32U;
  static constexpr uint32_t RX_ID_RANGE =
      ROLE == DualBoardRole::GIMBAL
          ? REFEREE_ROBOT_HP_ID_OFFSET +
                RefereeCanCodec::FragmentCount<Referee::RobotHP>() - 1U
          : DECISION_ID_OFFSET + 1U;
  static constexpr float ANGLE_SCALE = 10000.0f;
  static constexpr float ANGLE_LIMIT = 3.2f;
  static constexpr float GYRO_SCALE = 900.0f;
  static constexpr float BULLET_SPEED_SCALE = 10.0f;
  static constexpr float BULLET_SPEED_LIMIT = 25.5f;
  static_assert(sizeof(ControlFrame) == 8,
                "ControlFrame must be one classic CAN frame");
  static_assert(sizeof(Pldx::DualBoardControl::UseCapacitorCommand) == 8,
                "UseCapacitorCommand must be one classic CAN frame");
  static_assert(
      USE_CAPACITOR_ID_OFFSET != CONTROL_ID_OFFSET &&
          USE_CAPACITOR_ID_OFFSET != ANGLE_ID_OFFSET &&
          USE_CAPACITOR_ID_OFFSET != DECISION_ID_OFFSET &&
          USE_CAPACITOR_ID_OFFSET <= RX_ID_RANGE,
      "use_capacitor CAN ID must stay in the gimbal-to-chassis range");
  static_assert(
      static_cast<uint8_t>(ChassisMode::RELAX) ==
              static_cast<uint8_t>(Pldx::DualBoardControl::Mode::RELAX) &&
          static_cast<uint8_t>(ChassisMode::INDEPENDENT) ==
              static_cast<uint8_t>(Pldx::DualBoardControl::Mode::INDEPENDENT) &&
          static_cast<uint8_t>(ChassisMode::ROTOR) ==
              static_cast<uint8_t>(Pldx::DualBoardControl::Mode::ROTOR) &&
          static_cast<uint8_t>(ChassisMode::FOLLOW) ==
              static_cast<uint8_t>(Pldx::DualBoardControl::Mode::FOLLOW) &&
          static_cast<uint8_t>(ChassisMode::NAVIGATION) ==
              static_cast<uint8_t>(Pldx::DualBoardControl::Mode::NAVIGATION) &&
          static_cast<uint8_t>(ChassisMode::NAVIGATION_ROTOR) ==
              static_cast<uint8_t>(
                  Pldx::DualBoardControl::Mode::NAVIGATION_ROTOR) &&
          static_cast<uint8_t>(ChassisMode::HOLE) ==
              static_cast<uint8_t>(Pldx::DualBoardControl::Mode::HOLE),
      "DualBoard and chassis modes must use the same wire values");
  static_assert(sizeof(AngleFrame) == 8,
                "AngleFrame must be one classic CAN frame");
  static_assert(sizeof(MotionFrame) == 8,
                "MotionFrame must be one classic CAN frame");
  static_assert(sizeof(ChassisYawFrame) == 8,
                "ChassisYawFrame must be one classic CAN frame");
  static_assert(sizeof(CapacitorFrame) == 8,
                "CapacitorFrame must be one classic CAN frame");
  static_assert(sizeof(LauncherFeedbackFrame) == 8,
                "LauncherFeedbackFrame must be one classic CAN frame");
  static_assert(sizeof(RefereeFragmentFrame) == 8,
                "RefereeFragmentFrame must be one classic CAN frame");
  static_assert(sizeof(RefereeLinkStatusFrame) == 8,
                "RefereeLinkStatusFrame must be one classic CAN frame");
  static_assert(CAPACITOR_ID_OFFSET + 1U == CHASSIS_YAW_ID_OFFSET,
                "chassis yaw CAN ID must follow capacitor");
  static_assert(REFEREE_LINK_STATUS_ID_OFFSET < CAPACITOR_ID_OFFSET &&
                CHASSIS_YAW_ID_OFFSET < REFEREE_SENTRY_INFO_ID_OFFSET &&
                DECISION_ID_OFFSET + 1U < REFEREE_SENTRY_INFO_ID_OFFSET);
  static_assert(
      REFEREE_SENTRY_INFO_ID_OFFSET + REFEREE_SENTRY_INFO_FRAGMENT_COUNT <=
              REFEREE_FIELD_EVENT_ID_OFFSET &&
          REFEREE_FIELD_EVENT_ID_OFFSET +
                  RefereeCanCodec::FragmentCount<Referee::FieldEvents>() <=
              REFEREE_ROBOT_HP_ID_OFFSET &&
          (ROLE != DualBoardRole::GIMBAL ||
           REFEREE_ROBOT_HP_ID_OFFSET +
                   RefereeCanCodec::FragmentCount<Referee::RobotHP>() - 1U <=
               RX_ID_RANGE),
      "2027 referee expansion must use disjoint migrated CAN ranges");
  static_assert(CHASSIS_VELOCITY_ID_OFFSET >=
                        RefereeCanCodec::ROBOT_POS_ID_OFFSET + 6U &&
                    CHASSIS_VELOCITY_ID_OFFSET <
                        REFEREE_SENTRY_INFO_ID_OFFSET &&
                    CHASSIS_VELOCITY_ID_OFFSET <= RX_ID_RANGE,
                "chassis velocity CAN ID must not overlap other frames");
  // 0x312 + 0x1a = 0x32C: no chassis frame uses 0x311 + 0x1b, and it is inside
  // the chassis receive filter.
  static_assert(CHASSIS_FF_ID_OFFSET != CONTROL_ID_OFFSET &&
                    CHASSIS_FF_ID_OFFSET != ANGLE_ID_OFFSET &&
                    CHASSIS_FF_ID_OFFSET != USE_CAPACITOR_ID_OFFSET &&
                    CHASSIS_FF_ID_OFFSET != DECISION_ID_OFFSET &&
                    CHASSIS_FF_ID_OFFSET + 1U != CHASSIS_VELOCITY_ID_OFFSET &&
                    CHASSIS_FF_ID_OFFSET <= RX_ID_RANGE,
                "chassis feedforward CAN ID must not overlap other frames");

  static void ThreadEntry(DualBoard* self) { self->RunThread(); }

  static int16_t EncodeSigned(float value, float scale, float limit) {
    value = std::clamp(value, -limit, limit);
    return static_cast<int16_t>(value * scale);
  }

  static float DecodeSigned(int16_t value, float scale) {
    return static_cast<float>(value) / scale;
  }

  static uint8_t EncodeUnsigned8(float value, float scale, float limit) {
    value = std::clamp(value, 0.0f, limit);
    value = value * scale + 0.5f;
    value = std::clamp(value, 0.0f, 255.0f);
    return static_cast<uint8_t>(value);
  }

  static float DecodeUnsigned8(uint8_t value, float scale) {
    return static_cast<float>(value) / scale;
  }

  static bool IsDue(uint32_t now_ms, uint32_t& next_ms, uint32_t period_ms) {
    if (next_ms == 0U) next_ms = now_ms;
    if (static_cast<int32_t>(now_ms - next_ms) < 0) return false;
    next_ms += ((now_ms - next_ms) / period_ms + 1U) * period_ms;
    return true;
  }

  template <typename Data>
  LibXR::Topic CreateTopic(const char* name, bool multi_publisher = false) {
    return LibXR::Topic::CreateTopic<Data>(name, nullptr, multi_publisher);
  }

  void RegisterRoleTopics() {
    mode_topic_ = CreateTopic<uint32_t>(mode_topic_name_);
    RegisterDecisionTopics();

    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      RegisterTopicCallback<CMD::ChassisControl, &DualBoard::OnLocalControl>(
          "control_state");
      if (manual_configuration_interlock_) {
        state_.configuration_fold_topic_ = LibXR::Topic(
            LibXR::Topic::FindOrCreate<Pldx::ConfigurationFoldCommand>(
                Pldx::CONFIGURATION_FOLD_COMMAND_TOPIC));
        state_.configuration_folded_topic_ =
            CreateTopic<bool>("configuration_folded");
        LibXR::Topic::FindOrCreate<Pldx::ConfigurationFeedback>(
            Pldx::CONFIGURATION_FEEDBACK_TOPIC);
        RegisterTopicCallback<Pldx::ConfigurationFeedback,
                              &DualBoard::OnConfigurationFeedback>(
            Pldx::CONFIGURATION_FEEDBACK_TOPIC);
      }
      LibXR::Topic::FindOrCreate<float>("yawmotor_angle");
      LibXR::Topic::FindOrCreate<float>("pitchmotor_angle");
      RegisterTopicCallback<float, &DualBoard::OnLocalYawAngle>(
          "yawmotor_angle");
      RegisterTopicCallback<float, &DualBoard::OnLocalPitchAngle>(
          "pitchmotor_angle");

      state_.chassis_motion_state_topic_ =
          LibXR::Topic(LibXR::Topic::FindOrCreate<ChassisMotionState>(
              CHASSIS_MOTION_STATE_TOPIC_NAME, nullptr,
              CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER));

      launcher_ref_topic_ = CreateTopic<Referee::LauncherPack>("launcher_ref");
      sentry_ref_topic_ =
          CreateTopic<Referee::RobotGameRefereePack>("sentry_ref");
      state_.chassis_imu_yaw_topic_ = CreateTopic<float>("chassis_imu_yaw");
      state_.chassis_imu_yaw_valid_topic_ =
          CreateTopic<bool>("chassis_imu_yaw_valid");
      state_.chassis_capacitor_capacity_topic_ =
          CreateTopic<uint8_t>("chassis_capacitor_capacity");
      state_.chassis_capacitor_valid_topic_ =
          CreateTopic<bool>("chassis_capacitor_valid");
      // Published from the CAN RX thread and the protocol-thread watchdog.
      state_.chassis_feedback_topic_ =
          CreateTopic<Pldx::DualBoardControl::ChassisFeedback>(
              Pldx::DualBoardControl::CHASSIS_FEEDBACK_TOPIC, true);
      CreateTopic<uint32_t>(Pldx::DualBoardControl::NAV_CHASSIS_MODE_TOPIC);
      RegisterTopicCallback<uint32_t, &DualBoard::OnNavChassisMode>(
          Pldx::DualBoardControl::NAV_CHASSIS_MODE_TOPIC);
      CreateTopic<bool>(Pldx::DualBoardControl::USE_CAPACITOR_TOPIC);
      RegisterTopicCallback<bool, &DualBoard::OnLocalUseCapacitor>(
          Pldx::DualBoardControl::USE_CAPACITOR_TOPIC);
    } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
      state_.chassis_cmd_topic_ = CreateTopic<CMD::ChassisCMD>("chassis_cmd");
      state_.yaw_angle_topic_ = CreateTopic<float>("yawmotor_angle");
      state_.pitch_angle_topic_ = CreateTopic<float>("pitchmotor_angle");

      RegisterTopicCallback<Referee::LauncherPack,
                            &DualBoard::OnLocalLauncherFeedback>(
          "launcher_ref");
      RegisterTopicCallback<Referee::RobotGameRefereePack,
                            &DualBoard::OnLocalSentryRef>("sentry_ref");
      RegisterTopicCallback<Eigen::Matrix<float, 3, 1>,
                            &DualBoard::OnLocalChassisGyro>("chassis_gyro");
      RegisterTopicCallback<LibXR::EulerAngle<float>,
                            &DualBoard::OnLocalChassisEuler>(
          chassis_euler_topic_name_);
      state_.use_capacitor_topic_ =
          CreateTopic<bool>(Pldx::DualBoardControl::USE_CAPACITOR_TOPIC);
    }
  }

  void RegisterDecisionTopics() {
    sentry_buy_bullet_num_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<uint16_t>(
            sentry_buy_bullet_num_topic_name_, nullptr));
    sentry_remote_buy_bullet_times_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<uint8_t>(
            sentry_remote_buy_bullet_times_topic_name_, nullptr));
    sentry_remote_buy_hp_times_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<uint8_t>(
            sentry_remote_buy_hp_times_topic_name_, nullptr));
    sentry_buy_resurrection_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<bool>(
            sentry_buy_resurrection_topic_name_, nullptr));
    sentry_state_topic_ = LibXR::Topic(
        LibXR::Topic::FindOrCreate<uint8_t>(sentry_state_topic_name_, nullptr));

    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      RegisterDecisionQueue(sentry_buy_bullet_num_topic_,
                            state_.sentry_buy_bullet_updates_);
      RegisterDecisionQueue(sentry_remote_buy_bullet_times_topic_,
                            state_.sentry_remote_buy_bullet_updates_);
      RegisterDecisionQueue(sentry_remote_buy_hp_times_topic_,
                            state_.sentry_remote_buy_hp_updates_);
      RegisterDecisionQueue(sentry_buy_resurrection_topic_,
                            state_.sentry_buy_resurrection_updates_);
      RegisterDecisionQueue(sentry_state_topic_, state_.sentry_state_updates_);
    }
  }

  template <typename Data>
  void RegisterDecisionQueue(LibXR::Topic topic,
                             LibXR::SPSCQueue<Data>& queue) {
    auto* subscriber = new LibXR::Topic::QueuedSubscriber(topic, queue);
    UNUSED(subscriber);
    struct QueueBind {
      LibXR::SPSCQueue<Data>* queue_;
      std::atomic<uint32_t>* drops_;
    };
    auto* bind = new QueueBind{&queue, &state_.decision_update_drops_};
    auto callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, QueueBind* bind, const Data& data) {
          UNUSED(in_isr);
          UNUSED(data);
          if (bind->queue_->EmptySize() == 0U) {
            bind->drops_->fetch_add(1U, std::memory_order_relaxed);
          }
        },
        bind);
    topic.RegisterCallback(callback);
  }

  template <typename Data, void (DualBoard::*HANDLER)(const Data&)>
  void RegisterTopicCallback(const char* topic_name) {
    auto topic_handle = LibXR::Topic::Find(topic_name, nullptr);
    ASSERT(topic_handle != nullptr);

    auto callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, DualBoard* self, const Data& data) {
          UNUSED(in_isr);
          (self->*HANDLER)(data);
        },
        this);
    LibXR::Topic(topic_handle).RegisterCallback(callback);
  }

  void RegisterModeEvent() {
    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, DualBoard* self, uint32_t event_id) {
          UNUSED(in_isr);
          self->OnLocalModeEvent(event_id);
        },
        this);

    dual_board_event_.Register(static_cast<uint32_t>(ChassisMode::RELAX),
                               callback);
    dual_board_event_.Register(static_cast<uint32_t>(ChassisMode::INDEPENDENT),
                               callback);
    dual_board_event_.Register(static_cast<uint32_t>(ChassisMode::ROTOR),
                               callback);
    dual_board_event_.Register(static_cast<uint32_t>(ChassisMode::FOLLOW),
                               callback);
    dual_board_event_.Register(static_cast<uint32_t>(ChassisMode::NAVIGATION),
                               callback);
    dual_board_event_.Register(
        static_cast<uint32_t>(ChassisMode::NAVIGATION_ROTOR), callback);
    dual_board_event_.Register(static_cast<uint32_t>(ChassisMode::HOLE),
                               callback);
  }

  void RegisterCmdEvent() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (cmd_ == nullptr) {
        return;
      }

      auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
          [](bool in_isr, DualBoard* self, uint32_t event_id) {
            UNUSED(in_isr);
            UNUSED(event_id);
            self->SetLocalModeRelax();
          },
          this);

      auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
          [](bool in_isr, DualBoard* self, uint32_t event_id) {
            UNUSED(in_isr);
            UNUSED(event_id);
            self->ClearLocalModeRelax();
          },
          this);

      cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
      cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    }
  }

  void RegisterCanRx() {
    can_rx_callback_ = LibXR::CAN::Callback::Create(
        [](bool in_isr, DualBoard* self, const LibXR::CAN::ClassicPack& pack) {
          if (pack.id < self->rx_id_ || pack.id > self->rx_id_ + RX_ID_RANGE) {
            return;
          }
          if (self->rx_frames_.Push(pack) == LibXR::ErrorCode::OK) {
            self->rx_sem_.PostFromCallback(in_isr);
          }
        },
        this);

    can_->Register(can_rx_callback_, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, rx_id_,
                   rx_id_ + RX_ID_RANGE);

    can_error_callback_ = LibXR::CAN::Callback::Create(
        [](bool in_isr, DualBoard* self, const LibXR::CAN::ClassicPack& pack) {
          UNUSED(in_isr);
          self->OnCanError(pack);
        },
        this);
    can_->Register(can_error_callback_, LibXR::CAN::Type::ERROR);
  }

  static bool IsCanLinkFault(LibXR::CAN::ErrorID error_id) {
    return error_id == LibXR::CAN::ErrorID::CAN_ERROR_ID_BUS_OFF ||
           error_id == LibXR::CAN::ErrorID::CAN_ERROR_ID_ERROR_PASSIVE;
  }

  void OnCanError(const LibXR::CAN::ClassicPack& pack) {
    if (pack.type != LibXR::CAN::Type::ERROR ||
        !LibXR::CAN::IsErrorId(pack.id)) {
      return;
    }
    if (IsCanLinkFault(LibXR::CAN::ToErrorID(pack.id))) {
      can_bus_fault_.Set();
    }
  }

  void RefreshCanErrorState() {
    LibXR::CAN::ErrorState state{};
    if (can_->GetErrorState(state) != LibXR::ErrorCode::OK) {
      return;
    }
    if (state.bus_off || state.error_passive) {
      can_bus_fault_.Set();
    } else {
      can_bus_fault_.Clear();
    }
  }

  void ClearLatchedBusFaultIfUnobservable() {
    LibXR::CAN::ErrorState state{};
    if (can_->GetErrorState(state) != LibXR::ErrorCode::OK) {
      can_bus_fault_.Clear();
    }
  }

  void OnLocalControl(const CMD::ChassisControl& control) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      if (manual_configuration_interlock_ &&
          state_.local_control_.control_revision != control.control_revision) {
        state_.configuration_control_.Reset();
      }
      state_.local_chassis_command_ = control.command;
      state_.local_control_ = control;
      const auto requested_mode = Pldx::DualBoardControl::SelectRequestedMode(
          control,
          static_cast<Pldx::DualBoardControl::Mode>(state_.rc_chassis_mode_));
      state_.committed_mode_ = Pldx::DualBoardControl::SelectOutputMode(
          requested_mode, state_.nav_mode_valid_,
          static_cast<Pldx::DualBoardControl::Mode>(state_.nav_chassis_mode_),
          control.mode == CMD::Mode::CMD_AUTO_CTRL, control.ai_chassis_online,
          !control.rc_online, control.explicit_requests);
      state_.local_chassis_command_time_ms_ =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    } else {
      UNUSED(control);
    }
  }

  void OnConfigurationFeedback(const Pldx::ConfigurationFeedback& feedback) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.configuration_feedback_ = feedback;
      state_.configuration_feedback_time_ms_ =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    } else {
      UNUSED(feedback);
    }
  }

  void OnLocalYawAngle(const float& yaw_angle) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_yaw_angle_ = yaw_angle;
    } else {
      UNUSED(yaw_angle);
    }
  }

  void OnLocalPitchAngle(const float& pitch_angle) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_pitch_angle_ = pitch_angle;
    } else {
      UNUSED(pitch_angle);
    }
  }

  void OnLocalLauncherFeedback(const Referee::LauncherPack& launcher_pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_launcher_pack_ = launcher_pack;
      state_.launcher_feedback_valid_ = true;
    } else {
      UNUSED(launcher_pack);
    }
  }

  void OnLocalSentryRef(const Referee::RobotGameRefereePack& referee_pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_sentry_ref_ = referee_pack;
      state_.pending_referee_sources_ |= Referee::SourceMaskForCommand(
          static_cast<Referee::CommandID>(referee_pack.source_command_id));
      state_.referee_status_pending_ = true;
    } else {
      UNUSED(referee_pack);
    }
  }

  void OnLocalChassisGyro(const Eigen::Matrix<float, 3, 1>& gyro) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_chassis_gyro_ = gyro;
      state_.chassis_gyro_received_ = true;
    } else {
      UNUSED(gyro);
    }
  }

  void OnLocalChassisEuler(const LibXR::EulerAngle<float>& euler) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_chassis_yaw_ = euler.Yaw();
      state_.chassis_yaw_valid_ = std::isfinite(state_.local_chassis_yaw_);
      state_.local_chassis_yaw_time_ms_ =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    } else {
      UNUSED(euler);
    }
  }

  void OnNavChassisMode(const uint32_t& mode) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (!IsSupportedMode(mode)) {
        return;
      }

      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.nav_chassis_mode_ = static_cast<uint8_t>(mode);
      state_.nav_mode_valid_ = true;
    } else {
      UNUSED(mode);
    }
  }

  void OnLocalUseCapacitor(const bool& enabled) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_use_capacitor_ = enabled;
    } else {
      UNUSED(enabled);
    }
  }

  void OnLocalModeEvent(uint32_t mode) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (!IsSupportedMode(mode)) {
        return;
      }

      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.rc_chassis_mode_ = static_cast<uint8_t>(mode);
    } else {
      UNUSED(mode);
    }
  }

  void SetLocalModeRelax() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_chassis_command_ = {};
      state_.output_relax_ = true;
      state_.configuration_control_.Reset();
      PublishSelectedModeLocked(static_cast<uint8_t>(ChassisMode::RELAX));
    }
  }

  void ClearLocalModeRelax() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.output_relax_ = false;
    }
  }

  void RunThread() {
    auto last_time = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()) -
                     PROTOCOL_THREAD_PERIOD_MS;
    while (true) {
      const auto now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      const uint32_t elapsed = now_ms - last_time;
      if (elapsed < PROTOCOL_THREAD_PERIOD_MS) {
        if (rx_sem_.Wait(PROTOCOL_THREAD_PERIOD_MS - elapsed) ==
            LibXR::ErrorCode::OK) {
          LibXR::CAN::ClassicPack pack{};
          if (rx_frames_.Pop(pack) == LibXR::ErrorCode::OK) {
            HandleCanFrame(pack);
          }
        }
        continue;
      }
      last_time = now_ms;

      if constexpr (ROLE == DualBoardRole::GIMBAL) {
        DrainDecisionUpdates();
        SendDecisionFrameIfDue(now_ms);
        ReportDecisionUpdateDrops(now_ms);
        SendGimbalControlFrames(now_ms);
        SendChassisFeedforwardFrameIfDue(now_ms);
        CheckChassisYawWatchdog(now_ms);
        CheckCapacitorWatchdog(now_ms);
        CheckChassisVelocityWatchdog(now_ms);
        MaintainRefereeFreshness(now_ms);
      } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
        {
          LibXR::Mutex::LockGuard lock(data_mutex_);
          if (state_.command_assembly_.Expire(now_ms) &&
              state_.command_assembly_.MotionFresh(now_ms)) {
            PublishChassisCommandLocked(state_.remote_mode_);
          }
        }
        SendMotionFrameIfDue(now_ms);
        SendChassisVelocityFrameIfDue(now_ms);
        SendChassisYawFrameIfDue(now_ms);
        SendCapacitorFrameIfDue(now_ms);
        SendLauncherFeedbackFrameIfDue(now_ms);
        SendRefereeFramesIfDue(now_ms);
      }

      CheckOffline(now_ms);
    }
  }

  void DrainDecisionUpdates() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      for (size_t processed = 0U; processed < DECISION_UPDATE_QUEUE_CAPACITY;) {
        const size_t BEFORE = processed;
        uint16_t buy_bullet = 0U;
        if (processed < DECISION_UPDATE_QUEUE_CAPACITY &&
            state_.sentry_buy_bullet_updates_.Pop(buy_bullet) ==
                LibXR::ErrorCode::OK) {
          SentryDecision::accumulate(state_.pending_decision_,
                                     SentryDecision::UpdateKind::BUY_BULLET,
                                     buy_bullet);
          ++processed;
        }
        uint8_t remote_bullet = 0U;
        if (processed < DECISION_UPDATE_QUEUE_CAPACITY &&
            state_.sentry_remote_buy_bullet_updates_.Pop(remote_bullet) ==
                LibXR::ErrorCode::OK) {
          SentryDecision::accumulate(
              state_.pending_decision_,
              SentryDecision::UpdateKind::REMOTE_BUY_BULLET, remote_bullet);
          ++processed;
        }
        uint8_t remote_hp = 0U;
        if (processed < DECISION_UPDATE_QUEUE_CAPACITY &&
            state_.sentry_remote_buy_hp_updates_.Pop(remote_hp) ==
                LibXR::ErrorCode::OK) {
          SentryDecision::accumulate(state_.pending_decision_,
                                     SentryDecision::UpdateKind::REMOTE_BUY_HP,
                                     remote_hp);
          ++processed;
        }
        bool buy_resurrection = false;
        if (processed < DECISION_UPDATE_QUEUE_CAPACITY &&
            state_.sentry_buy_resurrection_updates_.Pop(buy_resurrection) ==
                LibXR::ErrorCode::OK) {
          SentryDecision::accumulate(
              state_.pending_decision_,
              SentryDecision::UpdateKind::BUY_RESURRECTION,
              buy_resurrection ? 1U : 0U);
          ++processed;
        }
        uint8_t state = 0U;
        if (processed < DECISION_UPDATE_QUEUE_CAPACITY &&
            state_.sentry_state_updates_.Pop(state) == LibXR::ErrorCode::OK) {
          SentryDecision::accumulate(state_.pending_decision_,
                                     SentryDecision::UpdateKind::STATE, state);
          ++processed;
        }
        if (processed == BEFORE) {
          break;
        }
      }
    }
  }

  void SendDecisionFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (!state_.decision_retry_.Active() &&
          state_.pending_decision_.valid_mask != 0U) {
        const bool STARTED = state_.decision_retry_.Begin(
            state_.pending_decision_, ++state_.decision_sequence_, now_ms);
        ASSERT(STARTED);
        state_.pending_decision_ = {};
        state_.pending_decision_.version = SentryDecision::VERSION;
      }

      if (!state_.decision_retry_.Due(now_ms)) {
        return;
      }

      const bool SENT = SendClassicFrame(tx_id_ + DECISION_ID_OFFSET,
                                         state_.decision_retry_.Frame());
      state_.decision_retry_.OnSendResult(SENT, now_ms);
    }
  }

  void ReportDecisionUpdateDrops(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      const uint32_t DROPS =
          state_.decision_update_drops_.load(std::memory_order_relaxed);
      if (DROPS == state_.reported_decision_update_drops_) {
        return;
      }
      if (state_.decision_drop_log_started_ &&
          now_ms - state_.last_decision_drop_log_ms_ <
              DECISION_DROP_LOG_PERIOD_MS) {
        return;
      }

      XR_LOG_WARN("DualBoard decision update queue dropped {} items", DROPS);
      state_.reported_decision_update_drops_ = DROPS;
      state_.last_decision_drop_log_ms_ = now_ms;
      state_.decision_drop_log_started_ = true;
    }
  }

  void SendMotionFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (!IsDue(now_ms, next_control_tx_ms_, CONTROL_PERIOD_MS)) {
        return;
      }

      Eigen::Matrix<float, 3, 1> gyro{};
      bool gyro_received = false;
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        gyro = state_.local_chassis_gyro_;
        gyro_received = state_.chassis_gyro_received_;
      }

      MotionFrame frame{};
      float gyro_z = 0.0f;
      if (gyro_received) {
        gyro_z = gyro.z();
      }
      if (gyro_received && std::isfinite(gyro_z)) {
        float gyro_z_q = gyro_z * GYRO_SCALE;
        if (gyro_z_q >=
                static_cast<float>(std::numeric_limits<int16_t>::min()) &&
            gyro_z_q <=
                static_cast<float>(std::numeric_limits<int16_t>::max())) {
          frame.gyro_z_q = static_cast<int16_t>(gyro_z_q);
          frame.gyro_valid = 1U;
        }
      }

      SendClassicFrame(tx_id_ + ANGLE_ID_OFFSET, frame);
    }
  }

  void SendChassisYawFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (!IsDue(now_ms, state_.next_chassis_yaw_tx_ms_, CONTROL_PERIOD_MS)) {
        return;
      }

      float yaw = 0.0F;
      bool valid = false;
      uint32_t chassis_yaw_time_ms = 0U;
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        yaw = state_.local_chassis_yaw_;
        valid = state_.chassis_yaw_valid_;
        chassis_yaw_time_ms = state_.local_chassis_yaw_time_ms_;
      }

      ChassisYawFrame frame{};
      frame.valid =
          valid && std::isfinite(yaw) &&
                  now_ms - chassis_yaw_time_ms <= CHASSIS_YAW_TIMEOUT_MS
              ? 1U
              : 0U;
      if (frame.valid != 0U) {
        yaw = std::clamp(yaw, -3.2F, 3.2F);
        frame.yaw_q = static_cast<int16_t>(std::lround(yaw * ANGLE_SCALE));
      }
      frame.sequence = state_.chassis_yaw_sequence_++;
      frame.sample_time_ms = now_ms;
      SendClassicFrame(tx_id_ + CHASSIS_YAW_ID_OFFSET, frame);
    }
  }

  void SendChassisVelocityFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::CHASSIS &&
                  std::is_same_v<ChassisType, Omni>) {
      if (!IsDue(now_ms, state_.next_chassis_velocity_tx_ms_,
                 CHASSIS_VELOCITY_PERIOD_MS)) {
        return;
      }

      const auto MEASURED = chassis_->GetMeasuredState();
      const uint64_t NOW_US = LibXR::Timebase::GetMicroseconds();
      const uint64_t SAMPLE_US = MEASURED.sample_time_us;
      Pldx::DualBoardControl::ChassisVelocity velocity{};
      velocity.velocity = {MEASURED.velocity.vx_mps, MEASURED.velocity.vy_mps,
                           MEASURED.velocity.wz_rad_s};
      velocity.valid = SAMPLE_US != 0U && NOW_US >= SAMPLE_US &&
                       NOW_US - SAMPLE_US <= CHASSIS_VELOCITY_SAMPLE_TIMEOUT_US;
      for (int i = 0; i < 4; i++) {
        velocity.wheel_online[i] = MEASURED.wheel_online[i];
      }
      velocity.mode = static_cast<uint8_t>(MEASURED.mode);
      velocity.sequence = state_.chassis_velocity_sequence_++;

      Pldx::DualBoardControl::ChassisVelocityFrame frame{};
      Pldx::DualBoardControl::EncodeChassisVelocity(velocity, frame);
      SendClassicFrame(tx_id_ + CHASSIS_VELOCITY_ID_OFFSET, frame);
    } else {
      UNUSED(now_ms);
    }
  }

  void SendChassisFeedforwardFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (!IsDue(now_ms, state_.next_chassis_ff_tx_ms_, CHASSIS_FF_PERIOD_MS)) {
        return;
      }
      CMD::ChassisCMD value{};
      uint32_t received_ms = 0U;
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        value = state_.local_chassis_command_;
        if (state_.output_relax_ || !state_.local_control_.rc_online)
          value = {};
        received_ms = state_.local_chassis_command_time_ms_;
      }
      // A stale host command must not keep pushing the chassis.
      if (now_ms - received_ms > CHASSIS_FF_SOURCE_TIMEOUT_MS) {
        value = {};
      }
      Pldx::DualBoardControl::ChassisFeedforwardFrame frame{};
      Pldx::DualBoardControl::EncodeChassisFeedforward(
          value, state_.chassis_ff_sequence_++, frame);
      SendClassicFrame(tx_id_ + CHASSIS_FF_ID_OFFSET, frame);
    } else {
      UNUSED(now_ms);
    }
  }

  void HandleChassisFeedforwardFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      Pldx::DualBoardControl::ChassisFeedforwardFrame frame{};
      LoadClassicFrame(pack, frame);
      const auto NOW =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.command_assembly_.Feedforward(
          Pldx::DualBoardControl::DecodeChassisFeedforward(frame), NOW);
      if (state_.command_assembly_.MotionFresh(NOW)) {
        PublishChassisCommandLocked(state_.remote_mode_);
      }
    } else {
      UNUSED(pack);
    }
  }

  void SendCapacitorFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (!IsDue(now_ms, state_.next_capacitor_tx_ms_, CONTROL_PERIOD_MS)) {
        return;
      }

      CapacitorFrame frame{};
      if (power_control_ != nullptr) {
        const PowerControlData DATA = power_control_->GetPowerControlData();
        const bool VALID =
            DATA.supercap_online && std::isfinite(DATA.cap_energy_normalized);
        frame.valid = VALID ? 1U : 0U;
        if (VALID) {
          frame.capacity_percent = static_cast<uint8_t>(std::lround(
              std::clamp(DATA.cap_energy_normalized, 0.0F, 1.0F) * 100.0F));
        }
      }
      frame.sequence = state_.capacitor_sequence_++;
      frame.sample_time_ms = now_ms;
      SendClassicFrame(tx_id_ + CAPACITOR_ID_OFFSET, frame);
    }
  }

  void SendGimbalControlFrames(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (!IsDue(now_ms, next_control_tx_ms_, CONTROL_PERIOD_MS)) {
        return;
      }

      CMD::ChassisCMD command{};
      float yaw_angle = 0.0f;
      float pitch_angle = 0.0f;
      bool use_capacitor = true;
      Pldx::DualBoardControl::Mode selected_mode =
          Pldx::DualBoardControl::Mode::RELAX;
      Pldx::ConfigurationFoldCommand fold_command{
          .action = Pldx::FoldAction::HOLD, .stamp_ms = now_ms};
      bool configuration_folded = false;

      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        command = state_.local_chassis_command_;
        yaw_angle = state_.local_yaw_angle_;
        pitch_angle = state_.local_pitch_angle_;
        use_capacitor = state_.local_use_capacitor_;
        const bool RC_LOST = state_.output_relax_ ||
                             !state_.local_control_.rc_online ||
                             now_ms - state_.local_chassis_command_time_ms_ >
                                 offline_timeout_ms_;
        selected_mode = RC_LOST ? Pldx::DualBoardControl::Mode::RELAX
                                : state_.committed_mode_;
        const bool manual_configuration =
            manual_configuration_interlock_ &&
            state_.local_control_.mode == CMD::Mode::CMD_OP_CTRL;
        if (manual_configuration_interlock_) {
          fold_command.control_revision =
              state_.local_control_.control_revision;
          if (!RC_LOST && manual_configuration) {
            const auto output = SelectManualConfiguration(now_ms);
            selected_mode = output.mode;
            fold_command.action = output.fold;
            configuration_folded = output.folded;
            if (std::isfinite(yaw_angle)) {
              yaw_angle =
                  std::remainder(yaw_angle, static_cast<float>(LibXR::TWO_PI));
            }
          } else {
            state_.configuration_control_.Reset();
          }
        }
        if (RC_LOST || ((manual_configuration ||
                         state_.local_control_.explicit_requests) &&
                        selected_mode == Pldx::DualBoardControl::Mode::RELAX)) {
          command = {};
        }
        PublishSelectedModeLocked(static_cast<uint8_t>(selected_mode));
      }
      if (manual_configuration_interlock_) {
        state_.configuration_fold_topic_.Publish(fold_command);
        state_.configuration_folded_topic_.Publish(configuration_folded);
      }

      const auto wire_command = Pldx::DualBoardControl::MakeControlCommand(
          command, selected_mode, manual_hole_max_speed_mps_);
      ControlFrame control_frame{};
      static_cast<void>(
          Pldx::DualBoardControl::Encode(wire_command, control_frame));

      AngleFrame angle_frame{};
      angle_frame.yaw = EncodeSigned(yaw_angle, ANGLE_SCALE, ANGLE_LIMIT);
      angle_frame.pitch = EncodeSigned(pitch_angle, ANGLE_SCALE, ANGLE_LIMIT);
      angle_frame.sequence = state_.tx_sequence_++;

      Pldx::DualBoardControl::UseCapacitorCommand use_capacitor_frame{};
      static_cast<void>(Pldx::DualBoardControl::EncodeUseCapacitor(
          use_capacitor, state_.use_capacitor_sequence_++,
          use_capacitor_frame));

      SendClassicFrame(tx_id_ + CONTROL_ID_OFFSET, control_frame);
      SendClassicFrame(tx_id_ + ANGLE_ID_OFFSET, angle_frame);
      SendClassicFrame(tx_id_ + USE_CAPACITOR_ID_OFFSET, use_capacitor_frame);
    }
  }

  Pldx::DualBoardControl::ConfigurationOutput SelectManualConfiguration(
      uint32_t now_ms) {
    using Pldx::DualBoardControl::Mode;
    const auto& feedback = state_.configuration_feedback_;
    const bool valid = feedback.fold_valid && feedback.yaw_valid &&
                       feedback.fold_sample_us != 0U &&
                       feedback.yaw_sample_us != 0U &&
                       now_ms - state_.configuration_feedback_time_ms_ <=
                           offline_timeout_ms_ &&
                       state_.has_chassis_velocity_sequence_ &&
                       !state_.chassis_velocity_stale_ &&
                       now_ms - state_.last_chassis_velocity_rx_ms_ <=
                           CHASSIS_VELOCITY_TIMEOUT_MS &&
                       !can_bus_fault_.IsSet();
    return state_.configuration_control_.Step(
        {.request = state_.local_control_.explicit_requests
                        ? state_.committed_mode_
                        : static_cast<Mode>(state_.rc_chassis_mode_),
         .actual_mode = state_.actual_chassis_mode_,
         .position = feedback.position,
         .yaw_rad = feedback.yaw_rad,
         .feedback_valid = valid,
         .yaw_sample_us = feedback.yaw_sample_us,
         .explicit_requests = state_.local_control_.explicit_requests,
         .folded_request = state_.local_control_.folded_request});
  }

  void SendLauncherFeedbackFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (!IsDue(now_ms, state_.next_launcher_feedback_tx_ms_,
                 LAUNCHER_FEEDBACK_PERIOD_MS)) {
        return;
      }

      Referee::LauncherPack launcher_pack{};
      bool valid = false;
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        launcher_pack = state_.local_launcher_pack_;
        valid = state_.launcher_feedback_valid_;
      }

      if (!valid) {
        return;
      }

      LauncherFeedbackFrame frame{};
      frame.heat_limit = launcher_pack.rs.shooter_heat_limit;
      frame.cooling_rate = launcher_pack.rs.shooter_cooling_value;
      frame.heat = launcher_pack.ph.launcher_id1_17_heat;
      frame.bullet_speed_deci =
          EncodeUnsigned8(launcher_pack.ld.bullet_speed, BULLET_SPEED_SCALE,
                          BULLET_SPEED_LIMIT);
      frame.robot_level = launcher_pack.rs.robot_level;

      SendClassicFrame(tx_id_ + CONTROL_ID_OFFSET, frame);
    }
  }

  template <typename Data>
  bool SendRefereeFragments(uint32_t base_offset, uint8_t sequence,
                            const Data& data) {
    const auto frames = RefereeCanCodec::Encode(sequence, data);
    for (size_t index = 0U; index < frames.size(); ++index) {
      if (!SendClassicFrame(tx_id_ + base_offset + index, frames[index]))
        return false;
    }
    return true;
  }

  void SendRefereeFramesIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      Referee::RobotGameRefereePack referee_pack{};
      bool status_pending = false;
      uint16_t source = 0U;
      size_t source_index = 0U;
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        referee_pack = state_.local_sentry_ref_;
        status_pending = state_.referee_status_pending_;
        state_.referee_status_pending_ = false;
        // One complete group per service gives control/motion frames priority.
        // Rotate even on FULL so one large group cannot starve the others.
        for (size_t count = 0U; count < 12U; ++count) {
          source_index = state_.next_referee_source_;
          state_.next_referee_source_ = (source_index + 1U) % 12U;
          source = static_cast<uint16_t>(1U << source_index);
          if ((state_.pending_referee_sources_ & source) != 0U) {
            state_.pending_referee_sources_ &= ~source;
            break;
          }
          source = 0U;
        }
      }

      const bool STATUS_DUE = IsDue(now_ms, state_.next_referee_status_tx_ms_,
                                    REFEREE_STATUS_PERIOD_MS);
      if (status_pending || STATUS_DUE) {
        RefereeLinkStatusFrame frame{};
        frame.sequence = state_.referee_status_sequence_++;
        frame.referee_online = referee_pack.referee_online ? 1U : 0U;
        frame.supported_mask = Referee::SUPPORTED_SOURCE_MASK;
        frame.valid_mask =
            referee_pack.source_valid_mask & referee_pack.source_fresh_mask;
        frame.layout_version = RefereeCanCodec::LAYOUT_VERSION;
        if (!SendClassicFrame(tx_id_ + REFEREE_LINK_STATUS_ID_OFFSET, frame)) {
          LibXR::Mutex::LockGuard lock(data_mutex_);
          state_.referee_status_pending_ = true;
          state_.pending_referee_sources_ |= source;
          return;
        }
      }

      // A failed partial group is retried as a new immutable sequence. This
      // prevents mixing data when a newer source value arrives during retry.
      const uint8_t sequence = state_.referee_sequences_[source_index]++;
      bool sent = true;
      switch (source) {
        case Referee::SOURCE_GAME_STATUS: {
          RefereeGameStatusData data{
              referee_pack.game_status.game_type,
              referee_pack.game_status.game_progress,
              referee_pack.game_status.stage_remain_time};
          sent = SendRefereeFragments(REFEREE_GAME_STATUS_ID_OFFSET, sequence,
                                      data);
          break;
        }
        case Referee::SOURCE_ROBOT_HP:
          sent = SendRefereeFragments(REFEREE_ROBOT_HP_ID_OFFSET, sequence,
                                      referee_pack.robot_hp);
          break;
        case Referee::SOURCE_FIELD_EVENT:
          sent = SendRefereeFragments(REFEREE_FIELD_EVENT_ID_OFFSET, sequence,
                                      referee_pack.field_event);
          break;
        case Referee::SOURCE_ROBOT_STATUS:
          sent = SendRefereeFragments(REFEREE_ROBOT_STATUS_ID_OFFSET, sequence,
                                      referee_pack.robot_status);
          break;
        case Referee::SOURCE_POWER_HEAT:
          sent = SendRefereeFragments(REFEREE_POWER_HEAT_ID_OFFSET, sequence,
                                      referee_pack.power_heat);
          break;
        case Referee::SOURCE_SENTRY_POS:
          sent = SendRefereeFragments(REFEREE_ROBOT_POS_ID_OFFSET, sequence,
                                      referee_pack.sentry_pos);
          break;
        case Referee::SOURCE_ROBOT_BUFF:
          sent = SendRefereeFragments(REFEREE_ROBOT_BUFF_ID_OFFSET, sequence,
                                      referee_pack.robot_buff);
          break;
        case Referee::SOURCE_ROBOT_DAMAGE:
          sent = SendRefereeFragments(REFEREE_ROBOT_DAMAGE_ID_OFFSET, sequence,
                                      referee_pack.robot_damage);
          break;
        case Referee::SOURCE_BULLET_REMAIN:
          sent = SendRefereeFragments(REFEREE_BULLET_REMAIN_ID_OFFSET, sequence,
                                      referee_pack.bullet_remain);
          break;
        case Referee::SOURCE_RFID:
          sent = SendRefereeFragments(REFEREE_RFID_ID_OFFSET, sequence,
                                      referee_pack.rfid);
          break;
        case Referee::SOURCE_SENTRY_INFO:
          sent = SendRefereeFragments(REFEREE_SENTRY_INFO_ID_OFFSET, sequence,
                                      referee_pack.sentry_info);
          break;
        default:
          break;
      }
      if (!sent) {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        state_.pending_referee_sources_ |= source;
      }
    }
  }

  template <typename Frame>
  bool SendClassicFrame(uint32_t id, const Frame& frame) {
    static_assert(sizeof(Frame) == 8U,
                  "DualBoard fixed frames must fill classic CAN payload");
    LibXR::CAN::ClassicPack pack{};
    pack.id = id;
    pack.type = LibXR::CAN::Type::STANDARD;
    pack.dlc = sizeof(Frame);
    LibXR::Memory::FastCopy(pack.data, &frame, sizeof(Frame));
    return can_->AddMessage(pack) == LibXR::ErrorCode::OK;
  }

  template <typename Frame>
  static void LoadClassicFrame(const LibXR::CAN::ClassicPack& pack,
                               Frame& frame) {
    static_assert(sizeof(Frame) == 8U,
                  "DualBoard fixed frames must fill classic CAN payload");
    LibXR::Memory::FastCopy(&frame, pack.data, sizeof(Frame));
  }

  template <typename Data>
  static void PublishValue(LibXR::Topic& topic, Data value) {
    topic.Publish(value);
  }

  void HandleCanFrame(const LibXR::CAN::ClassicPack& pack) {
    if (pack.type != LibXR::CAN::Type::STANDARD || pack.dlc != 8U) {
      return;
    }

    ClearLatchedBusFaultIfUnobservable();
    auto offset = pack.id - rx_id_;
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (offset == CONTROL_ID_OFFSET) {
        HandleControlFrame(pack);
      } else if (offset == ANGLE_ID_OFFSET) {
        HandleAngleFrame(pack);
      } else if (offset == USE_CAPACITOR_ID_OFFSET) {
        HandleUseCapacitorFrame(pack);
      } else if (offset == DECISION_ID_OFFSET) {
        HandleDecisionFrame(pack);
      } else if (offset == CHASSIS_FF_ID_OFFSET) {
        HandleChassisFeedforwardFrame(pack);
      }
    } else if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (HandleRefereeFrame(offset, pack)) {
        return;
      }
      if (offset == CHASSIS_YAW_ID_OFFSET) {
        HandleChassisYawFrame(pack);
      } else if (offset == CHASSIS_VELOCITY_ID_OFFSET) {
        HandleChassisVelocityFrame(pack);
      } else if (offset == CAPACITOR_ID_OFFSET) {
        HandleCapacitorFrame(pack);
      } else if (offset == CONTROL_ID_OFFSET) {
        HandleLauncherFeedbackFrame(pack);
      } else if (offset == ANGLE_ID_OFFSET) {
        HandleMotionFrame(pack);
      }
    }
  }

  void HandleUseCapacitorFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      Pldx::DualBoardControl::UseCapacitorCommand frame{};
      LoadClassicFrame(pack, frame);
      bool enabled = true;
      if (!Pldx::DualBoardControl::DecodeUseCapacitor(frame, enabled)) {
        return;
      }
      state_.use_capacitor_topic_.Publish(enabled);
    } else {
      UNUSED(pack);
    }
  }

  void HandleDecisionFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      SentryDecisionFrame frame{};
      LoadClassicFrame(pack, frame);
      if (!SentryDecision::Validate(frame)) {
        return;
      }

      const uint32_t now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      if (!state_.decision_sequence_tracker_.Accept(frame.sequence, now_ms)) {
        return;
      }

      if ((frame.valid_mask & SentryDecision::BUY_BULLET_VALID) != 0U) {
        uint16_t value = frame.buy_bullet_delta;
        sentry_buy_bullet_num_topic_.Publish(value);
      }
      if ((frame.valid_mask & SentryDecision::REMOTE_BULLET_VALID) != 0U) {
        uint8_t event = 1U;
        const uint8_t COUNT =
            SentryDecision::RemoteBulletCount(frame.remote_request_counts);
        for (uint8_t index = 0U; index < COUNT; ++index) {
          sentry_remote_buy_bullet_times_topic_.Publish(event);
        }
      }
      if ((frame.valid_mask & SentryDecision::REMOTE_HP_VALID) != 0U) {
        uint8_t event = 1U;
        const uint8_t COUNT =
            SentryDecision::RemoteHpCount(frame.remote_request_counts);
        for (uint8_t index = 0U; index < COUNT; ++index) {
          sentry_remote_buy_hp_times_topic_.Publish(event);
        }
      }
      if ((frame.valid_mask & SentryDecision::BUY_RESURRECTION_VALID) != 0U) {
        bool value =
            (frame.flags & SentryDecision::BUY_RESURRECTION_FLAG) != 0U;
        sentry_buy_resurrection_topic_.Publish(value);
      }
      if ((frame.valid_mask & SentryDecision::STATE_VALID) != 0U) {
        uint8_t value = frame.state;
        sentry_state_topic_.Publish(value);
      }
    } else {
      UNUSED(pack);
    }
  }

  template <typename Data>
  bool ReassembleReferee(uint32_t offset, uint32_t base_offset,
                         RefereeAssembly& assembly,
                         const LibXR::CAN::ClassicPack& pack, uint32_t now_ms,
                         Data& output, uint16_t source_mask,
                         Referee::CommandID command_id) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (offset < base_offset ||
          offset >= base_offset + RefereeCanCodec::FragmentCount<Data>()) {
        return false;
      }

      RefereeFragmentFrame frame{};
      LoadClassicFrame(pack, frame);
      const auto result = RefereeCanCodec::Push(
          assembly, static_cast<size_t>(offset - base_offset), frame, now_ms,
          output);
      if (result != RefereeCanCodec::PushResult::COMPLETE) return false;
      if (source_mask == Referee::SOURCE_SENTRY_POS) {
        state_.gimbal_sentry_ref_.sentry_pos_received_time_ms = now_ms;
      } else if (source_mask == Referee::SOURCE_ROBOT_HP) {
        state_.gimbal_sentry_ref_.robot_hp_received_time_ms = now_ms;
      }
      RecordRefereeSource(source_mask, now_ms);
      PublishGimbalReferee(static_cast<uint16_t>(command_id), now_ms);
      return true;
    }
    return false;
  }

  void RecordRefereeSource(uint16_t source_mask, uint32_t now_ms) {
    state_.local_referee_valid_mask_ |= source_mask;
    for (size_t index = 0; index < state_.referee_source_times_.size();
         ++index) {
      if (source_mask == (1U << index)) {
        state_.referee_source_times_[index] = now_ms;
        break;
      }
    }
  }

  void PublishGimbalReferee(uint16_t command_id, uint32_t now_ms) {
    auto& referee = state_.gimbal_sentry_ref_;
    referee.source_command_id = command_id;
    referee.referee_online =
        state_.referee_layout_compatible_ && state_.upstream_referee_online_;
    referee.source_valid_mask = RefereeCanCodec::IntersectValidity(
        referee.referee_online, state_.local_referee_valid_mask_,
        state_.upstream_referee_valid_mask_, Referee::SUPPORTED_SOURCE_MASK);
    referee.source_fresh_mask = Referee::FreshSourceMask(
        referee.source_valid_mask, state_.referee_source_times_, now_ms);
    sentry_ref_topic_.Publish(referee);
    Referee::LauncherPack launcher = state_.launcher_feedback_;
    if (!referee.referee_online) {
      launcher = {};
    } else {
      if ((referee.source_fresh_mask & Referee::SOURCE_ROBOT_BUFF) != 0U)
        launcher.rb = referee.robot_buff;
      else
        launcher.rb = {};
      if ((referee.source_fresh_mask & Referee::SOURCE_ROBOT_STATUS) != 0U)
        launcher.rs = referee.robot_status;
      else
        launcher.rs = {};
      if ((referee.source_fresh_mask & Referee::SOURCE_POWER_HEAT) != 0U)
        launcher.ph = referee.power_heat;
      else
        launcher.ph = {};
    }
    launcher_ref_topic_.Publish(launcher);
  }

  void ClearRefereeSources() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      state_.local_referee_valid_mask_ = 0U;
      state_.upstream_referee_valid_mask_ = 0U;
      state_.referee_source_times_ = {};
      for (auto& assembly : state_.referee_assemblies_) assembly = {};
    }
  }

  void MaintainRefereeFreshness(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      for (auto& assembly : state_.referee_assemblies_)
        RefereeCanCodec::Expire(assembly, now_ms);
      const auto& referee = state_.gimbal_sentry_ref_;
      const uint16_t fresh = Referee::FreshSourceMask(
          referee.source_valid_mask, state_.referee_source_times_, now_ms);
      if (fresh != referee.source_fresh_mask) PublishGimbalReferee(0U, now_ms);
    }
  }

  bool HandleRefereeFrame(uint32_t offset,
                          const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      const uint32_t now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      if (offset == REFEREE_LINK_STATUS_ID_OFFSET) {
        RefereeLinkStatusFrame frame{};
        LoadClassicFrame(pack, frame);
        const bool compatible =
            frame.layout_version == RefereeCanCodec::LAYOUT_VERSION;
        if (!compatible) {
          if (!state_.referee_layout_status_received_ ||
              state_.referee_layout_compatible_) {
            XR_LOG_WARN("DualBoard referee layout mismatch: {}",
                        frame.layout_version);
          }
          ClearRefereeSources();
        }
        state_.referee_layout_status_received_ = true;
        state_.referee_layout_compatible_ = compatible;
        state_.upstream_referee_online_ =
            compatible && frame.referee_online == 1U;
        state_.upstream_referee_valid_mask_ =
            compatible ? frame.valid_mask & frame.supported_mask : 0U;
        if (!state_.upstream_referee_online_) ClearRefereeSources();
        PublishGimbalReferee(0U, now_ms);
        last_rx_time_ms_ = now_ms;
        online_ = true;
        safe_state_published_ = false;
        return true;
      }
      // Only a matching status opens a new layout epoch. Discard earlier data
      // so old-layout bytes cannot become valid after a later v2 status.
      if (!state_.referee_layout_compatible_) return false;
      if (offset == REFEREE_GAME_STATUS_ID_OFFSET) {
        RefereeFragmentFrame frame{};
        LoadClassicFrame(pack, frame);
        auto& assembly = state_.referee_assemblies_[0];
        if (assembly.has_published &&
            assembly.published_sequence == frame.sequence) {
          return true;
        }
        assembly.has_published = true;
        assembly.published_sequence = frame.sequence;
        state_.gimbal_sentry_ref_.game_status.game_type = frame.data[0];
        state_.gimbal_sentry_ref_.game_status.game_progress = frame.data[1];
        LibXR::Memory::FastCopy(
            &state_.gimbal_sentry_ref_.game_status.stage_remain_time,
            &frame.data[2], sizeof(uint16_t));
        RecordRefereeSource(Referee::SOURCE_GAME_STATUS, now_ms);
        PublishGimbalReferee(
            static_cast<uint16_t>(Referee::CommandID::REF_CMD_ID_GAME_STATUS),
            now_ms);
        last_rx_time_ms_ = now_ms;
        online_ = true;
        safe_state_published_ = false;
        return true;
      }

#define HANDLE_REFEREE_GROUP(BASE, STATE, FIELD, MASK, COMMAND)            \
  if (ReassembleReferee(offset, BASE, STATE, pack, now_ms,                 \
                        state_.gimbal_sentry_ref_.FIELD, MASK, COMMAND)) { \
    last_rx_time_ms_ = now_ms;                                             \
    online_ = true;                                                        \
    safe_state_published_ = false;                                         \
    return true;                                                           \
  }
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_HP_ID_OFFSET,
                           state_.referee_assemblies_[1], robot_hp,
                           Referee::SOURCE_ROBOT_HP,
                           Referee::CommandID::REF_CMD_ID_GAME_ROBOT_HP)
      HANDLE_REFEREE_GROUP(REFEREE_FIELD_EVENT_ID_OFFSET,
                           state_.referee_assemblies_[2], field_event,
                           Referee::SOURCE_FIELD_EVENT,
                           Referee::CommandID::REF_CMD_ID_FIELD_EVENTS)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_STATUS_ID_OFFSET,
                           state_.referee_assemblies_[3], robot_status,
                           Referee::SOURCE_ROBOT_STATUS,
                           Referee::CommandID::REF_CMD_ID_ROBOT_STATUS)
      HANDLE_REFEREE_GROUP(REFEREE_POWER_HEAT_ID_OFFSET,
                           state_.referee_assemblies_[4], power_heat,
                           Referee::SOURCE_POWER_HEAT,
                           Referee::CommandID::REF_CMD_ID_POWER_HEAT_DATA)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_POS_ID_OFFSET,
                           state_.referee_assemblies_[5], sentry_pos,
                           Referee::SOURCE_SENTRY_POS,
                           Referee::CommandID::REF_CMD_ID_ROBOT_POS_TO_SENTRY)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_BUFF_ID_OFFSET,
                           state_.referee_assemblies_[6], robot_buff,
                           Referee::SOURCE_ROBOT_BUFF,
                           Referee::CommandID::REF_CMD_ID_ROBOT_BUFF)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_DAMAGE_ID_OFFSET,
                           state_.referee_assemblies_[7], robot_damage,
                           Referee::SOURCE_ROBOT_DAMAGE,
                           Referee::CommandID::REF_CMD_ID_ROBOT_DMG)
      HANDLE_REFEREE_GROUP(REFEREE_BULLET_REMAIN_ID_OFFSET,
                           state_.referee_assemblies_[8], bullet_remain,
                           Referee::SOURCE_BULLET_REMAIN,
                           Referee::CommandID::REF_CMD_ID_BULLET_REMAINING)
      HANDLE_REFEREE_GROUP(
          REFEREE_RFID_ID_OFFSET, state_.referee_assemblies_[9], rfid,
          Referee::SOURCE_RFID, Referee::CommandID::REF_CMD_ID_RFID)
      HANDLE_REFEREE_GROUP(REFEREE_SENTRY_INFO_ID_OFFSET,
                           state_.referee_assemblies_[10], sentry_info,
                           Referee::SOURCE_SENTRY_INFO,
                           Referee::CommandID::REF_CMD_ID_SENTRY_DECISION)
#undef HANDLE_REFEREE_GROUP
    }
    return false;
  }

  void HandleMotionFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      MotionFrame frame{};
      LoadClassicFrame(pack, frame);

      const auto now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        last_rx_time_ms_ = now_ms;
        online_ = true;
        safe_state_published_ = false;
        const float gyro_z = DecodeSigned(frame.gyro_z_q, GYRO_SCALE);
        state_.motion_state_.yaw_rate_rad_s =
            frame.gyro_valid == 1U && std::isfinite(gyro_z) ? gyro_z : 0.0f;
        state_.motion_state_.yaw_rate_valid =
            frame.gyro_valid == 1U && std::isfinite(gyro_z);
        state_.motion_state_.online = true;
        PublishMotionStateLocked();
      }
    } else {
      UNUSED(pack);
    }
  }

  void HandleChassisYawFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      ChassisYawFrame frame{};
      LoadClassicFrame(pack, frame);
      const bool valid = frame.valid == 1U &&
                         std::abs(static_cast<int16_t>(frame.yaw_q)) <= 32000;
      const float yaw = DecodeSigned(frame.yaw_q, ANGLE_SCALE);
      const uint32_t now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      const bool accepted = valid && std::isfinite(yaw);
      PublishValue(state_.chassis_imu_yaw_topic_, accepted ? yaw : 0.0F);
      PublishValue(state_.chassis_imu_yaw_valid_topic_, accepted);
      state_.last_chassis_yaw_rx_ms_ = now_ms;
      state_.chassis_yaw_stale_ = !accepted;
    } else {
      UNUSED(pack);
    }
  }

  void CheckChassisYawWatchdog(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (state_.last_chassis_yaw_rx_ms_ != 0U &&
          now_ms - state_.last_chassis_yaw_rx_ms_ <= CHASSIS_YAW_TIMEOUT_MS) {
        return;
      }
      if (state_.chassis_yaw_stale_) {
        return;
      }
      PublishValue(state_.chassis_imu_yaw_topic_, 0.0F);
      PublishValue(state_.chassis_imu_yaw_valid_topic_, false);
      state_.chassis_yaw_stale_ = true;
    }
  }

  void HandleChassisVelocityFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      Pldx::DualBoardControl::ChassisVelocityFrame frame{};
      LoadClassicFrame(pack, frame);
      const auto DECODED = Pldx::DualBoardControl::DecodeChassisVelocity(frame);
      const auto NOW_US =
          static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
      const auto NOW_MS =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());

      Pldx::DualBoardControl::ChassisFeedback feedback{};
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        const uint32_t SEQUENCE = Pldx::DualBoardControl::ExtendSequence(
            state_.chassis_velocity_sequence_, DECODED.sequence,
            state_.has_chassis_velocity_sequence_);
        state_.chassis_velocity_sequence_ = SEQUENCE;
        state_.has_chassis_velocity_sequence_ = true;
        state_.last_chassis_velocity_rx_ms_ = NOW_MS;
        state_.chassis_velocity_stale_ = !DECODED.valid;
        state_.actual_chassis_mode_ =
            static_cast<Pldx::DualBoardControl::Mode>(DECODED.mode);

        feedback.sequence = SEQUENCE;
        feedback.gyro_z_rad_s = state_.motion_state_.yaw_rate_rad_s;
        if (state_.motion_state_.yaw_rate_valid) {
          feedback.flags |=
              Pldx::DualBoardControl::CHASSIS_FEEDBACK_FLAG_GYRO_VALID;
        }
      }
      feedback.vx_mps = DECODED.velocity.vx_mps;
      feedback.vy_mps = DECODED.velocity.vy_mps;
      feedback.wz_rad_s = DECODED.velocity.wz_rad_s;
      feedback.stamp_us = static_cast<uint32_t>(NOW_US);
      feedback.mode = DECODED.mode;
      feedback.flags |= Pldx::DualBoardControl::CHASSIS_FEEDBACK_FLAG_LINK;
      if (DECODED.valid) {
        feedback.flags |= Pldx::DualBoardControl::CHASSIS_FEEDBACK_FLAG_VALID;
      }
      for (uint8_t i = 0; i < 4U; i++) {
        if (DECODED.wheel_online[i]) {
          feedback.flags |= static_cast<uint8_t>(
              1U << (Pldx::DualBoardControl::CHASSIS_FEEDBACK_WHEEL_SHIFT + i));
        }
      }
      // Publish outside data_mutex_: the navigation CDC client writes in this
      // call.
      state_.chassis_feedback_topic_.Publish(feedback);
    } else {
      UNUSED(pack);
    }
  }

  void CheckChassisVelocityWatchdog(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      Pldx::DualBoardControl::ChassisFeedback feedback{};
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        if (state_.chassis_velocity_stale_ ||
            (state_.last_chassis_velocity_rx_ms_ != 0U &&
             now_ms - state_.last_chassis_velocity_rx_ms_ <=
                 CHASSIS_VELOCITY_TIMEOUT_MS)) {
          return;
        }
        state_.chassis_velocity_stale_ = true;
        feedback.sequence = state_.chassis_velocity_sequence_;
      }
      // Link lost: tell the host explicitly instead of letting it time out.
      feedback.stamp_us =
          static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
      state_.chassis_feedback_topic_.Publish(feedback);
    } else {
      UNUSED(now_ms);
    }
  }

  void HandleCapacitorFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      CapacitorFrame frame{};
      LoadClassicFrame(pack, frame);
      const bool VALID = frame.valid == 1U && frame.capacity_percent <= 100U;
      const uint32_t NOW_MS =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      PublishValue<uint8_t>(state_.chassis_capacitor_capacity_topic_,
                            VALID ? frame.capacity_percent : 255U);
      PublishValue(state_.chassis_capacitor_valid_topic_, VALID);
      state_.last_capacitor_rx_ms_ = NOW_MS;
      state_.capacitor_stale_ = !VALID;
    } else {
      UNUSED(pack);
    }
  }

  void CheckCapacitorWatchdog(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (state_.last_capacitor_rx_ms_ != 0U &&
          now_ms - state_.last_capacitor_rx_ms_ <= CAPACITOR_TIMEOUT_MS) {
        return;
      }
      if (state_.capacitor_stale_) {
        return;
      }
      PublishValue<uint8_t>(state_.chassis_capacitor_capacity_topic_, 255U);
      PublishValue(state_.chassis_capacitor_valid_topic_, false);
      state_.capacitor_stale_ = true;
    }
  }

  void HandleControlFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      ControlFrame frame{};
      LoadClassicFrame(pack, frame);

      Pldx::DualBoardControl::Command decoded{};
      if (!Pldx::DualBoardControl::Decode(frame, decoded)) {
        return;
      }

      const bool restored = !online_;
      last_rx_time_ms_ =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      online_ = true;
      safe_state_published_ = false;

      const uint8_t MODE = static_cast<uint8_t>(decoded.mode);
      bool publish_mode = restored;
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        state_.command_assembly_.Motion(
            decoded, static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds()));
        publish_mode |= state_.remote_mode_ != MODE;
        if constexpr (std::is_same_v<ChassisType, Omni>) {
          publish_mode |= PublishChassisCommandLocked(MODE);
          state_.remote_mode_ = MODE;
        } else {
          PublishChassisCommandLocked(MODE);
          if (publish_mode) publish_mode = ForceRemoteMode(MODE);
        }
      }

      if (publish_mode) {
        uint32_t mode = MODE;
        mode_topic_.Publish(mode);
      }
    } else {
      UNUSED(pack);
    }
  }

  void HandleAngleFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (!online_) {
        return;
      }

      AngleFrame frame{};
      LoadClassicFrame(pack, frame);
      float yaw_angle = DecodeSigned(frame.yaw, ANGLE_SCALE);
      float pitch_angle = DecodeSigned(frame.pitch, ANGLE_SCALE);
      state_.yaw_angle_topic_.Publish(yaw_angle);
      state_.pitch_angle_topic_.Publish(pitch_angle);
    } else {
      UNUSED(pack);
    }
  }

  void HandleLauncherFeedbackFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LauncherFeedbackFrame frame{};
      LoadClassicFrame(pack, frame);

      Referee::LauncherPack launcher_pack = state_.launcher_feedback_;
      launcher_pack.rs.shooter_heat_limit = frame.heat_limit;
      launcher_pack.rs.shooter_cooling_value = frame.cooling_rate;
      launcher_pack.ph.launcher_id1_17_heat = frame.heat;
      launcher_pack.ld.bullet_speed =
          DecodeUnsigned8(frame.bullet_speed_deci, BULLET_SPEED_SCALE);
      launcher_pack.rs.robot_level = frame.robot_level;
      state_.launcher_feedback_ = launcher_pack;
      if (state_.referee_layout_compatible_ &&
          state_.upstream_referee_online_) {
        if ((state_.gimbal_sentry_ref_.source_fresh_mask &
             Referee::SOURCE_ROBOT_BUFF) != 0U)
          launcher_pack.rb = state_.gimbal_sentry_ref_.robot_buff;
        else
          launcher_pack.rb = {};
        launcher_ref_topic_.Publish(launcher_pack);
      }

      last_rx_time_ms_ =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      online_ = true;
      safe_state_published_ = false;
    } else {
      UNUSED(pack);
    }
  }

  bool LinkTimedOut(uint32_t now_ms) const {
    return offline_timeout_ms_ != 0U && last_rx_time_ms_ != 0U &&
           (LibXR::MillisecondTimestamp(now_ms) - last_rx_time_ms_)
                   .ToMillisecond() > offline_timeout_ms_;
  }

  void CheckOffline(uint32_t now_ms) {
    RefreshCanErrorState();
    const bool BUS_FAULT = can_bus_fault_.IsSet();

    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        if (!BUS_FAULT && !LinkTimedOut(now_ms)) {
          return;
        }

        online_ = false;
        if (safe_state_published_) {
          return;
        }
      }

      PublishOfflineState();
    } else {
      if (!BUS_FAULT && !LinkTimedOut(now_ms)) {
        return;
      }

      online_ = false;
      if (!safe_state_published_) {
        PublishOfflineState();
        safe_state_published_ = true;
      }
    }
  }

  void PublishOfflineState() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      PublishInvalidLauncherFeedback();
    } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
      PublishSafeChassisState();
    }
  }

  void PublishInvalidLauncherFeedback() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      const LibXR::MillisecondTimestamp NOW =
          LibXR::Timebase::GetMilliseconds();
      if ((NOW - last_rx_time_ms_).ToMillisecond() <= offline_timeout_ms_) {
        return;
      }

      Referee::LauncherPack launcher_pack{};
      launcher_ref_topic_.Publish(launcher_pack);
      PublishInvalidReferee();

      state_.motion_state_ = {};
      PublishMotionStateLocked();
      safe_state_published_ = true;
    }
  }

  void PublishInvalidReferee() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      ClearRefereeSources();
      state_.upstream_referee_online_ = false;
      state_.referee_layout_compatible_ = false;
      state_.referee_layout_status_received_ = false;
      state_.gimbal_sentry_ref_.source_command_id = 0U;
      state_.gimbal_sentry_ref_.source_valid_mask = 0U;
      state_.gimbal_sentry_ref_.source_fresh_mask = 0U;
      state_.gimbal_sentry_ref_.referee_online = false;
      sentry_ref_topic_.Publish(state_.gimbal_sentry_ref_);
    }
  }

  void PublishMotionStateLocked() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      state_.chassis_motion_state_topic_.Publish(state_.motion_state_);
    }
  }

  void PublishSelectedModeLocked(uint8_t mode) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (mode == state_.last_output_chassis_mode_ &&
          state_.output_mode_published_) {
        return;
      }
      uint32_t published_mode = mode;
      mode_topic_.Publish(published_mode);
      state_.motion_state_.mode =
          (mode == static_cast<uint8_t>(ChassisMode::ROTOR) ||
           mode == static_cast<uint8_t>(ChassisMode::NAVIGATION_ROTOR))
              ? ChassisMotionMode::ROTOR
              : ChassisMotionMode::NON_ROTOR;
      PublishMotionStateLocked();
      state_.last_output_chassis_mode_ = mode;
      state_.output_mode_published_ = true;
    } else {
      UNUSED(mode);
    }
  }

  void PublishSafeChassisState() {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      const auto relax = static_cast<uint8_t>(ChassisMode::RELAX);
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        state_.command_assembly_ = {};
        PublishChassisCommandLocked(relax);
        if constexpr (std::is_same_v<ChassisType, Omni>) {
          state_.remote_mode_ = relax;
        } else {
          ForceRemoteMode(relax);
        }
      }

      float zero_angle = 0.0f;
      state_.yaw_angle_topic_.Publish(zero_angle);
      state_.pitch_angle_topic_.Publish(zero_angle);

      uint32_t mode = relax;
      mode_topic_.Publish(mode);
    }
  }

  bool PublishChassisCommandLocked(uint8_t mode) {
    auto command = state_.command_assembly_.command;
    bool changed = false;
    if constexpr (std::is_same_v<ChassisType, Omni>) {
      changed = chassis_->ApplyControlCommand(static_cast<ChassisMode>(mode),
                                              command);
    }
    state_.chassis_cmd_topic_.Publish(command);
    return changed;
  }

  bool ForceRemoteMode(uint32_t mode) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      // 失败时保留未应用标记，让后续相同模式帧继续重试。
      state_.remote_mode_ = std::numeric_limits<uint8_t>::max();
      if (chassis_ != nullptr) {
        chassis_->GetEvent().Active(mode);
        state_.remote_mode_ = static_cast<uint8_t>(mode);
        return true;
      }
    } else {
      UNUSED(mode);
    }
    return false;
  }

  bool IsSupportedMode(uint32_t mode) const {
    return mode == static_cast<uint32_t>(ChassisMode::RELAX) ||
           mode == static_cast<uint32_t>(ChassisMode::INDEPENDENT) ||
           mode == static_cast<uint32_t>(ChassisMode::ROTOR) ||
           mode == static_cast<uint32_t>(ChassisMode::FOLLOW) ||
           mode == static_cast<uint32_t>(ChassisMode::NAVIGATION) ||
           mode == static_cast<uint32_t>(ChassisMode::NAVIGATION_ROTOR) ||
           mode == static_cast<uint32_t>(ChassisMode::HOLE);
  }

  bool online_ = false;
  bool safe_state_published_ = false;
  LibXR::MillisecondTimestamp last_rx_time_ms_{};
  uint32_t next_control_tx_ms_ = 0;

  LibXR::CAN* can_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t offline_timeout_ms_;
  Chassis<ChassisType>* chassis_;
  CMD* cmd_;
  const char* mode_topic_name_;
  const char* sentry_buy_bullet_num_topic_name_;
  const char* sentry_remote_buy_bullet_times_topic_name_;
  const char* sentry_remote_buy_hp_times_topic_name_;
  const char* sentry_buy_resurrection_topic_name_;
  const char* sentry_state_topic_name_;
  const char* chassis_euler_topic_name_;
  const float manual_hole_max_speed_mps_;
  const bool manual_configuration_interlock_;
  PowerControl* power_control_;
  LibXR::Topic mode_topic_;
  LibXR::Topic launcher_ref_topic_;
  LibXR::Topic sentry_ref_topic_;
  LibXR::Topic sentry_buy_bullet_num_topic_;
  LibXR::Topic sentry_remote_buy_bullet_times_topic_;
  LibXR::Topic sentry_remote_buy_hp_times_topic_;
  LibXR::Topic sentry_buy_resurrection_topic_;
  LibXR::Topic sentry_state_topic_;
  LibXR::Event dual_board_event_;
  LibXR::CAN::Callback can_rx_callback_;
  LibXR::CAN::Callback can_error_callback_;
  LibXR::Flag::Atomic can_bus_fault_;
  LibXR::SPSCQueue<LibXR::CAN::ClassicPack> rx_frames_;
  LibXR::Semaphore rx_sem_;
  LibXR::Thread thread_;
  LibXR::Mutex data_mutex_;

  struct GimbalState {
    LibXR::Topic chassis_motion_state_topic_;
    LibXR::Topic chassis_imu_yaw_topic_;
    LibXR::Topic chassis_imu_yaw_valid_topic_;
    LibXR::Topic chassis_capacitor_capacity_topic_;
    LibXR::Topic chassis_capacitor_valid_topic_;
    LibXR::Topic chassis_feedback_topic_;
    uint32_t local_chassis_command_time_ms_ = 0U;
    uint32_t next_chassis_ff_tx_ms_ = 0U;
    uint8_t chassis_ff_sequence_ = 0U;
    LibXR::SPSCQueue<uint16_t> sentry_buy_bullet_updates_{
        DECISION_UPDATE_QUEUE_CAPACITY};
    LibXR::SPSCQueue<uint8_t> sentry_remote_buy_bullet_updates_{
        DECISION_UPDATE_QUEUE_CAPACITY};
    LibXR::SPSCQueue<uint8_t> sentry_remote_buy_hp_updates_{
        DECISION_UPDATE_QUEUE_CAPACITY};
    LibXR::SPSCQueue<bool> sentry_buy_resurrection_updates_{
        DECISION_UPDATE_QUEUE_CAPACITY};
    LibXR::SPSCQueue<uint8_t> sentry_state_updates_{
        DECISION_UPDATE_QUEUE_CAPACITY};
    std::atomic<uint32_t> decision_update_drops_{0};
    CMD::ChassisCMD local_chassis_command_{};
    CMD::ChassisControl local_control_{};
    Pldx::DualBoardControl::Mode committed_mode_ =
        Pldx::DualBoardControl::Mode::RELAX;
    float local_yaw_angle_ = 0.0f;
    float local_pitch_angle_ = 0.0f;
    uint8_t rc_chassis_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
    uint8_t nav_chassis_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
    uint8_t last_output_chassis_mode_ =
        static_cast<uint8_t>(ChassisMode::RELAX);
    bool nav_mode_valid_ = false;
    bool output_relax_ = false;
    bool output_mode_published_ = false;
    bool local_use_capacitor_ = true;
    Pldx::DualBoardControl::Mode actual_chassis_mode_ =
        Pldx::DualBoardControl::Mode::RELAX;
    Pldx::ConfigurationFeedback configuration_feedback_{};
    uint32_t configuration_feedback_time_ms_ = 0U;
    Pldx::DualBoardControl::ConfigurationControl configuration_control_;
    LibXR::Topic configuration_fold_topic_;
    LibXR::Topic configuration_folded_topic_;
    Referee::RobotGameRefereePack gimbal_sentry_ref_{};
    Referee::LauncherPack launcher_feedback_{};
    RefereeAssembly referee_assemblies_[11]{};
    Referee::SourceTimes referee_source_times_{};
    uint16_t local_referee_valid_mask_ = 0U;
    uint16_t upstream_referee_valid_mask_ = 0U;
    bool upstream_referee_online_ = false;
    bool referee_layout_status_received_ = false;
    bool referee_layout_compatible_ = false;
    ChassisMotionState motion_state_{};
    SentryDecisionFrame pending_decision_{
        SentryDecision::VERSION, 0U, 0U, 0U, 0U, 0U, 0U};
    SentryDecision::RetryController decision_retry_{};
    uint32_t last_chassis_yaw_rx_ms_ = 0U;
    uint32_t last_capacitor_rx_ms_ = 0U;
    uint32_t last_chassis_velocity_rx_ms_ = 0U;
    uint32_t chassis_velocity_sequence_ = 0U;
    bool has_chassis_velocity_sequence_ = false;
    bool chassis_velocity_stale_ = true;
    uint32_t last_decision_drop_log_ms_ = 0U;
    uint32_t reported_decision_update_drops_ = 0U;
    uint8_t tx_sequence_ = 0;
    uint8_t use_capacitor_sequence_ = 0U;
    uint8_t decision_sequence_ = 0U;
    bool decision_drop_log_started_ = false;
    bool chassis_yaw_stale_ = true;
    bool capacitor_stale_ = true;
  };

  struct ChassisState {
    Pldx::DualBoardControl::ChassisCommandAssembly command_assembly_;
    LibXR::Topic chassis_cmd_topic_;
    LibXR::Topic yaw_angle_topic_;
    LibXR::Topic pitch_angle_topic_;
    LibXR::Topic use_capacitor_topic_;
    Referee::LauncherPack local_launcher_pack_{};
    Referee::RobotGameRefereePack local_sentry_ref_{};
    uint16_t pending_referee_sources_ = 0U;
    bool referee_status_pending_ = false;
    bool launcher_feedback_valid_ = false;
    Eigen::Matrix<float, 3, 1> local_chassis_gyro_{};
    bool chassis_gyro_received_ = false;
    float local_chassis_yaw_ = 0.0F;
    bool chassis_yaw_valid_ = false;
    uint32_t local_chassis_yaw_time_ms_ = 0U;
    SentryDecision::SequenceTracker decision_sequence_tracker_{};
    uint32_t next_chassis_yaw_tx_ms_ = 0U;
    uint32_t next_chassis_velocity_tx_ms_ = 0U;
    uint32_t next_capacitor_tx_ms_ = 0U;
    uint32_t next_launcher_feedback_tx_ms_ = 0;
    uint32_t next_referee_status_tx_ms_ = 0U;
    uint8_t remote_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
    uint8_t chassis_yaw_sequence_ = 0U;
    uint8_t chassis_velocity_sequence_ = 0U;
    uint8_t capacitor_sequence_ = 0U;
    uint8_t referee_sequences_[12]{};
    uint8_t referee_status_sequence_ = 0U;
    uint8_t next_referee_source_ = 0U;
  };

  std::conditional_t<ROLE == DualBoardRole::GIMBAL, GimbalState, ChassisState>
      state_;
};
