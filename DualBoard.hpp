#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 双板定频 CAN 业务帧模块，通过 LibXR Topic 保持上层接口并使用 Classic CAN 固定帧同步双板状态
constructor_args:
  - can_bus_name: "can2"
  - tx_id: 0x312
  - rx_id: 0x311
  - rx_buffer_size: 256
  - offline_timeout_ms: 100
  - chassis: '@nullptr'
  - mode_topic_name: "dualboard_chassis_mode"
  - cmd: '@nullptr'
  - sentry_buy_bullet_num_topic_name: "sentry_buy_bullet_num"
  - sentry_remote_buy_bullet_times_topic_name: "sentry_remote_buy_bullet_times"
  - sentry_remote_buy_hp_times_topic_name: "sentry_remote_buy_hp_times"
  - sentry_buy_resurrection_topic_name: "sentry_buy_resurrection"
  - sentry_state_topic_name: "sentry_state"
  - chassis_euler_topic_name: "chassis_euler"
  - power_control: '@nullptr'
template_args:
  - ROLE: DualBoardRole::GIMBAL
  - ChassisType: Omni
required_hardware:
  - can

depends:
  - pldx/CMD
  - pldx/Chassis
  - pldx/PowerControl
  - pldx/Referee
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "PowerControl.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "flag.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
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
inline constexpr float NAVIGATION_SCALE = 1000.0F;
inline constexpr float NAVIGATION_WIRE_LIMIT = 32767.0F / NAVIGATION_SCALE;
inline constexpr uint8_t NAVIGATION_SOURCE_BIT = 1U << 7U;
inline constexpr uint8_t RESERVED_MASK = 0x70U;
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
};

enum class Source : uint8_t {
  OPERATOR = 0U,
  NAVIGATION = 1U,
};

struct OperatorInput {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct NavigationVelocity {
  float vx_mps = 0.0F;
  float vy_mps = 0.0F;
  float wz_rad_s = 0.0F;
};

struct Command {
  Source source = Source::OPERATOR;
  OperatorInput operator_input{};
  NavigationVelocity navigation_velocity{};
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
  return static_cast<uint8_t>(mode) <= static_cast<uint8_t>(Mode::NAVIGATION);
}

inline Mode SelectOutputMode(Mode rc_mode, bool nav_valid, Mode nav_mode,
                             bool auto_ctrl, bool rc_lost) {
  if (rc_lost) {
    return Mode::RELAX;
  }
  if (auto_ctrl && nav_valid) {
    return nav_mode;
  }
  return rc_mode;
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

inline bool NavigationVelocityFinite(const NavigationVelocity& velocity) {
  return std::isfinite(velocity.vx_mps) && std::isfinite(velocity.vy_mps) &&
         std::isfinite(velocity.wz_rad_s);
}

inline bool NavigationVelocityEncodable(const NavigationVelocity& velocity) {
  return NavigationVelocityFinite(velocity) &&
         std::fabs(velocity.vx_mps) <= NAVIGATION_WIRE_LIMIT &&
         std::fabs(velocity.vy_mps) <= NAVIGATION_WIRE_LIMIT &&
         std::fabs(velocity.wz_rad_s) <= NAVIGATION_WIRE_LIMIT;
}

inline bool Encode(const Command& command, Frame& frame) {
  frame = SafeFrame();
  if (!IsSupportedMode(command.mode) ||
      !IsSupportedSelfDefine(command.self_define)) {
    return false;
  }

  std::array<int16_t, 3U> encoded{};
  if (command.source == Source::NAVIGATION) {
    if (!NavigationVelocityEncodable(command.navigation_velocity)) return false;
    const std::array<float, 3U> VALUES{command.navigation_velocity.vx_mps,
                                       command.navigation_velocity.vy_mps,
                                       command.navigation_velocity.wz_rad_s};
    for (size_t index = 0U; index < VALUES.size(); ++index) {
      encoded[index] =
          static_cast<int16_t>(std::round(VALUES[index] * NAVIGATION_SCALE));
    }
  } else {
    const std::array<float, 3U> VALUES{command.operator_input.x,
                                       command.operator_input.y,
                                       command.operator_input.z};
    for (size_t index = 0U; index < VALUES.size(); ++index) {
      if (!std::isfinite(VALUES[index])) return false;
      const float CLAMPED = std::clamp(VALUES[index], -1.0F, 1.0F);
      encoded[index] = static_cast<int16_t>(CLAMPED * NORMALIZED_SCALE);
    }
  }

  StoreSigned(frame, 0U, encoded[0]);
  StoreSigned(frame, 2U, encoded[1]);
  StoreSigned(frame, 4U, encoded[2]);
  frame[6] = static_cast<uint8_t>(command.self_define);
  frame[7] =
      static_cast<uint8_t>(command.mode) |
      (command.source == Source::NAVIGATION ? NAVIGATION_SOURCE_BIT : 0U);
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
  command.source = (MODE_RAW & NAVIGATION_SOURCE_BIT) != 0U ? Source::NAVIGATION
                                                            : Source::OPERATOR;
  if (command.source == Source::NAVIGATION) {
    command.navigation_velocity.vx_mps =
        static_cast<float>(RAW[0]) / NAVIGATION_SCALE;
    command.navigation_velocity.vy_mps =
        static_cast<float>(RAW[1]) / NAVIGATION_SCALE;
    command.navigation_velocity.wz_rad_s =
        static_cast<float>(RAW[2]) / NAVIGATION_SCALE;
    return NavigationVelocityFinite(command.navigation_velocity);
  }
  command.operator_input.x = static_cast<float>(RAW[0]) / NORMALIZED_SCALE;
  command.operator_input.y = static_cast<float>(RAW[1]) / NORMALIZED_SCALE;
  command.operator_input.z = static_cast<float>(RAW[2]) / NORMALIZED_SCALE;
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

}  // namespace Pldx::DualBoardControl

class RefereeCanCodec {
 public:
  static constexpr uint32_t GAME_STATUS_ID_OFFSET = 0x02U;
  static constexpr uint32_t FIELD_EVENT_ID_OFFSET = 0x03U;
  static constexpr uint32_t ROBOT_HP_ID_OFFSET = 0x04U;
  static constexpr uint32_t ROBOT_STATUS_ID_OFFSET = 0x07U;
  static constexpr uint32_t POWER_HEAT_ID_OFFSET = 0x0AU;
  static constexpr uint32_t ROBOT_BUFF_ID_OFFSET = 0x0CU;
  static constexpr uint32_t BULLET_REMAIN_ID_OFFSET = 0x0EU;
  static constexpr uint32_t RFID_ID_OFFSET = 0x12U;
  static constexpr uint32_t ROBOT_DAMAGE_ID_OFFSET = 0x13U;
  // Position payload uses six dedicated classic-CAN frames (40 bytes).
  static constexpr uint32_t ROBOT_POS_ID_OFFSET = 0x14U;
  static constexpr uint32_t LINK_STATUS_ID_OFFSET = 0x1EU;
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

    Expire(assembly, now_ms);
    if (!assembly.active || assembly.sequence != frame.sequence) {
      assembly = {};
      assembly.active = true;
      assembly.sequence = frame.sequence;
    }
    if (now_ms >= assembly.last_update_ms) assembly.last_update_ms = now_ms;

    const size_t byte_offset = fragment_index * FRAGMENT_DATA_SIZE;
    const size_t byte_count = std::min(
        FRAGMENT_DATA_SIZE, static_cast<size_t>(sizeof(Data) - byte_offset));
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
    if (!assembly.active || now_ms < assembly.last_update_ms ||
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
        now_ms - last_rx_ms_ <= RX_TIMEOUT_MS) {
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
  uint32_t last_rx_ms_ = 0U;
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
    return active_ && (now_ms - next_send_ms_) < 0x80000000U;
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
  SentryDecisionFrame frame_{};
  bool active_ = false;
  uint8_t successes_ = 0U;
  uint32_t next_send_ms_ = 0U;
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
class DualBoard : public LibXR::Application {
 public:
  using ChassisMode = typename ChassisType::ChassisMode;

  DualBoard(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      const char* can_bus_name, uint32_t tx_id, uint32_t rx_id,
      uint32_t rx_buffer_size, uint32_t offline_timeout_ms,
      Chassis<ChassisType>* chassis,
      const char* mode_topic_name = "dualboard_chassis_mode",
      CMD* cmd = nullptr,
      const char* sentry_buy_bullet_num_topic_name = "sentry_buy_bullet_num",
      const char* sentry_remote_buy_bullet_times_topic_name =
          "sentry_remote_buy_bullet_times",
      const char* sentry_remote_buy_hp_times_topic_name =
          "sentry_remote_buy_hp_times",
      const char* sentry_buy_resurrection_topic_name =
          "sentry_buy_resurrection",
      const char* sentry_state_topic_name = "sentry_state",
      const char* chassis_euler_topic_name = "chassis_euler",
      PowerControl* power_control = nullptr)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})),
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
        power_control_(power_control),
        rx_frames_(rx_buffer_size) {
    ASSERT(rx_buffer_size > 0U);
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      ASSERT(chassis_ != nullptr);
    } else {
      UNUSED(chassis_);
    }

    RegisterRoleTopics();
    RegisterModeEvent();
    RegisterCmdEvent();
    RegisterCanRx();

    rx_thread_.Create(this, RxThreadEntry, "DualBoardRx", 1536,
                      LibXR::Thread::Priority::MEDIUM);
    protocol_thread_.Create(this, ProtocolThreadEntry, "DualBoardProto", 1536,
                            LibXR::Thread::Priority::MEDIUM);

    app.Register(*this);
  }

  void OnMonitor() override {}

  LibXR::Event& GetEvent() { return dual_board_event_; }

  bool IsOnline() const { return online_; }

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

  struct __attribute__((packed)) AttitudeFrame {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t sequence;
    uint8_t reserved;
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
    uint8_t reserved[2];
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
  static constexpr uint32_t ATTITUDE_ID_OFFSET = 0x20U;
  // Bottom-board yaw is sent on 0x330 (chassis tx_id 0x311 + 0x1f).
  static constexpr uint32_t CHASSIS_YAW_ID_OFFSET = 0x1FU;
  static constexpr uint32_t CAPACITOR_ID_OFFSET = 0x1EU;
  static constexpr uint32_t CHASSIS_YAW_TIMEOUT_MS = 100U;
  static constexpr uint32_t CAPACITOR_TIMEOUT_MS = 150U;
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
  static constexpr uint32_t REFEREE_LINK_STATUS_ID_OFFSET =
      RefereeCanCodec::LINK_STATUS_ID_OFFSET;
  static constexpr uint32_t REFEREE_STATUS_PERIOD_MS = 1000U;
  static constexpr uint32_t DECISION_DROP_LOG_PERIOD_MS = 1000U;
  static constexpr size_t DECISION_UPDATE_QUEUE_CAPACITY = 32U;
  static constexpr uint32_t RX_ID_RANGE = ATTITUDE_ID_OFFSET;
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
          USE_CAPACITOR_ID_OFFSET != ATTITUDE_ID_OFFSET &&
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
              static_cast<uint8_t>(Pldx::DualBoardControl::Mode::NAVIGATION),
      "DualBoard and chassis modes must use the same wire values");
  static_assert(sizeof(AngleFrame) == 8,
                "AngleFrame must be one classic CAN frame");
  static_assert(sizeof(MotionFrame) == 8,
                "MotionFrame must be one classic CAN frame");
  static_assert(sizeof(ChassisYawFrame) == 8,
                "ChassisYawFrame must be one classic CAN frame");
  static_assert(sizeof(CapacitorFrame) == 8,
                "CapacitorFrame must be one classic CAN frame");
  static_assert(sizeof(AttitudeFrame) == 8,
                "AttitudeFrame must be one classic CAN frame");
  static_assert(sizeof(LauncherFeedbackFrame) == 8,
                "LauncherFeedbackFrame must be one classic CAN frame");
  static_assert(sizeof(RefereeFragmentFrame) == 8,
                "RefereeFragmentFrame must be one classic CAN frame");
  static_assert(sizeof(RefereeLinkStatusFrame) == 8,
                "RefereeLinkStatusFrame must be one classic CAN frame");
  static_assert(CAPACITOR_ID_OFFSET + 1U == CHASSIS_YAW_ID_OFFSET,
                "chassis yaw CAN ID must follow capacitor");
  static_assert(RefereeCanCodec::LINK_STATUS_ID_OFFSET < DECISION_ID_OFFSET,
                "referee link status CAN ID must stay below decision");

  static void RxThreadEntry(DualBoard* self) { self->RunRxThread(); }

  static void ProtocolThreadEntry(DualBoard* self) {
    self->RunProtocolThread();
  }

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
    if (next_ms == 0U) {
      next_ms = now_ms;
    }

    if (now_ms < next_ms) {
      return false;
    }

    do {
      next_ms += period_ms;
    } while (next_ms <= now_ms);
    return true;
  }

  static uint16_t SourceMaskForCommand(Referee::CommandID command_id) {
    switch (command_id) {
      case Referee::CommandID::REF_CMD_ID_GAME_STATUS:
        return Referee::SOURCE_GAME_STATUS;
      case Referee::CommandID::REF_CMD_ID_GAME_ROBOT_HP:
        return Referee::SOURCE_ROBOT_HP;
      case Referee::CommandID::REF_CMD_ID_FIELD_EVENTS:
        return Referee::SOURCE_FIELD_EVENT;
      case Referee::CommandID::REF_CMD_ID_ROBOT_STATUS:
        return Referee::SOURCE_ROBOT_STATUS;
      case Referee::CommandID::REF_CMD_ID_POWER_HEAT_DATA:
        return Referee::SOURCE_POWER_HEAT;
      case Referee::CommandID::REF_CMD_ID_ROBOT_POS:
        return Referee::SOURCE_ROBOT_POS;
      case Referee::CommandID::REF_CMD_ID_ROBOT_POS_TO_SENTRY:
        return Referee::SOURCE_SENTRY_POS;
      case Referee::CommandID::REF_CMD_ID_ROBOT_BUFF:
        return Referee::SOURCE_ROBOT_BUFF;
      case Referee::CommandID::REF_CMD_ID_ROBOT_DMG:
        return Referee::SOURCE_ROBOT_DAMAGE;
      case Referee::CommandID::REF_CMD_ID_BULLET_REMAINING:
        return Referee::SOURCE_BULLET_REMAIN;
      case Referee::CommandID::REF_CMD_ID_RFID:
        return Referee::SOURCE_RFID;
      default:
        return 0U;
    }
  }

  template <typename Data>
  LibXR::Topic CreateTopic(const char* name, bool multi_publisher = false) {
    return LibXR::Topic::CreateTopic<Data>(name, nullptr, multi_publisher);
  }

  void RegisterRoleTopics() {
    mode_topic_ = CreateTopic<uint32_t>(mode_topic_name_);
    RegisterDecisionTopics();

    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      RegisterTopicCallback<CMD::ChassisCMD, &DualBoard::OnLocalChassisCommand>(
          "chassis_cmd");
      RegisterTopicCallback<float, &DualBoard::OnLocalYawAngle>(
          "yawmotor_angle");
      RegisterTopicCallback<float, &DualBoard::OnLocalPitchAngle>(
          "pitchmotor_angle");
      RegisterTopicCallback<LibXR::EulerAngle<float>,
                            &DualBoard::OnLocalAttitude>("gimbal_euler");

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
      state_.attitude_topic_ =
          CreateTopic<LibXR::EulerAngle<float>>("gimbal_euler");

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

  void OnLocalChassisCommand(const CMD::ChassisCMD& command) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_chassis_command_ = command;
    } else {
      UNUSED(command);
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

  void OnLocalAttitude(const LibXR::EulerAngle<float>& attitude) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.local_attitude_ = attitude;
    } else {
      UNUSED(attitude);
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
      state_.pending_referee_sources_ |= SourceMaskForCommand(
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
      PublishSelectedModeLocked(static_cast<uint8_t>(ChassisMode::RELAX));
    }
  }

  void ClearLocalModeRelax() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      state_.output_relax_ = false;
    }
  }

  void RunRxThread() {
    while (true) {
      if (rx_sem_.Wait() != LibXR::ErrorCode::OK) {
        continue;
      }

      LibXR::CAN::ClassicPack pack{};
      while (rx_frames_.Pop(pack) == LibXR::ErrorCode::OK) {
        HandleCanFrame(pack);
      }
    }
  }

  void RunProtocolThread() {
    auto last_time = LibXR::Timebase::GetMilliseconds();
    while (true) {
      auto now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());

      if constexpr (ROLE == DualBoardRole::GIMBAL) {
        DrainDecisionUpdates();
        SendDecisionFrameIfDue(now_ms);
        ReportDecisionUpdateDrops(now_ms);
        SendGimbalControlFrames(now_ms);
        CheckChassisYawWatchdog(now_ms);
        CheckCapacitorWatchdog(now_ms);
      } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
        SendMotionFrameIfDue(now_ms);
        SendChassisYawFrameIfDue(now_ms);
        SendCapacitorFrameIfDue(now_ms);
        SendLauncherFeedbackFrameIfDue(now_ms);
        SendRefereeFramesIfDue(now_ms);
      }

      CheckOffline(now_ms);
      protocol_thread_.SleepUntil(last_time, PROTOCOL_THREAD_PERIOD_MS);
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
      LibXR::EulerAngle<float> attitude{};
      bool use_capacitor = true;
      Pldx::DualBoardControl::Mode selected_mode =
          Pldx::DualBoardControl::Mode::RELAX;

      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        command = state_.local_chassis_command_;
        yaw_angle = state_.local_yaw_angle_;
        pitch_angle = state_.local_pitch_angle_;
        attitude = state_.local_attitude_;
        use_capacitor = state_.local_use_capacitor_;
        const bool AUTO_CTRL =
            cmd_ != nullptr && cmd_->GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL;
        const bool RC_LOST =
            state_.output_relax_ || (cmd_ != nullptr && !cmd_->Online());
        selected_mode = Pldx::DualBoardControl::SelectOutputMode(
            static_cast<Pldx::DualBoardControl::Mode>(state_.rc_chassis_mode_),
            state_.nav_mode_valid_,
            static_cast<Pldx::DualBoardControl::Mode>(state_.nav_chassis_mode_),
            AUTO_CTRL, RC_LOST);
        if (RC_LOST) {
          command = {};
        }
        PublishSelectedModeLocked(static_cast<uint8_t>(selected_mode));
      }

      Pldx::DualBoardControl::Command wire_command{};
      wire_command.source =
          command.source == CMD::ChassisCommandSource::NAVIGATION
              ? Pldx::DualBoardControl::Source::NAVIGATION
              : Pldx::DualBoardControl::Source::OPERATOR;
      wire_command.operator_input = {command.operator_input.x,
                                     command.operator_input.y,
                                     command.operator_input.z};
      wire_command.navigation_velocity = {command.navigation_velocity.vx_mps,
                                          command.navigation_velocity.vy_mps,
                                          command.navigation_velocity.wz_rad_s};
      wire_command.self_define = static_cast<int8_t>(command.self_define);
      wire_command.mode = selected_mode;
      ControlFrame control_frame{};
      static_cast<void>(
          Pldx::DualBoardControl::Encode(wire_command, control_frame));

      AngleFrame angle_frame{};
      angle_frame.yaw = EncodeSigned(yaw_angle, ANGLE_SCALE, ANGLE_LIMIT);
      angle_frame.pitch = EncodeSigned(pitch_angle, ANGLE_SCALE, ANGLE_LIMIT);
      angle_frame.sequence = state_.tx_sequence_++;

      AttitudeFrame attitude_frame{};
      attitude_frame.roll =
          EncodeSigned(attitude.Roll(), ANGLE_SCALE, ANGLE_LIMIT);
      attitude_frame.pitch =
          EncodeSigned(attitude.Pitch(), ANGLE_SCALE, ANGLE_LIMIT);
      attitude_frame.yaw =
          EncodeSigned(attitude.Yaw(), ANGLE_SCALE, ANGLE_LIMIT);
      attitude_frame.sequence = angle_frame.sequence;

      Pldx::DualBoardControl::UseCapacitorCommand use_capacitor_frame{};
      static_cast<void>(Pldx::DualBoardControl::EncodeUseCapacitor(
          use_capacitor, state_.use_capacitor_sequence_++,
          use_capacitor_frame));

      SendClassicFrame(tx_id_ + CONTROL_ID_OFFSET, control_frame);
      SendClassicFrame(tx_id_ + ANGLE_ID_OFFSET, angle_frame);
      SendClassicFrame(tx_id_ + ATTITUDE_ID_OFFSET, attitude_frame);
      SendClassicFrame(tx_id_ + USE_CAPACITOR_ID_OFFSET, use_capacitor_frame);
    }
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
  void SendRefereeFragments(uint32_t base_offset, uint8_t sequence,
                            const Data& data) {
    const auto frames = RefereeCanCodec::Encode(sequence, data);
    for (size_t index = 0U; index < frames.size(); ++index) {
      SendClassicFrame(tx_id_ + base_offset + index, frames[index]);
    }
  }

  void SendRefereeFramesIfDue(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      Referee::RobotGameRefereePack referee_pack{};
      uint16_t pending_sources = 0U;
      bool status_pending = false;
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        referee_pack = state_.local_sentry_ref_;
        pending_sources = state_.pending_referee_sources_;
        state_.pending_referee_sources_ = 0U;
        status_pending = state_.referee_status_pending_;
        state_.referee_status_pending_ = false;
      }

      if ((pending_sources & Referee::SOURCE_GAME_STATUS) != 0U) {
        RefereeGameStatusData data{referee_pack.game_status.game_type,
                                   referee_pack.game_status.game_progress,
                                   referee_pack.game_status.stage_remain_time};
        SendRefereeFragments(REFEREE_GAME_STATUS_ID_OFFSET,
                             state_.referee_sequences_[0]++, data);
      }
      if ((pending_sources & Referee::SOURCE_ROBOT_HP) != 0U) {
        SendRefereeFragments(REFEREE_ROBOT_HP_ID_OFFSET,
                             state_.referee_sequences_[1]++,
                             referee_pack.robot_hp);
      }
      if ((pending_sources & Referee::SOURCE_FIELD_EVENT) != 0U) {
        SendRefereeFragments(REFEREE_FIELD_EVENT_ID_OFFSET,
                             state_.referee_sequences_[2]++,
                             referee_pack.field_event);
      }
      if ((pending_sources & Referee::SOURCE_ROBOT_STATUS) != 0U) {
        SendRefereeFragments(REFEREE_ROBOT_STATUS_ID_OFFSET,
                             state_.referee_sequences_[3]++,
                             referee_pack.robot_status);
      }
      if ((pending_sources & Referee::SOURCE_POWER_HEAT) != 0U) {
        SendRefereeFragments(REFEREE_POWER_HEAT_ID_OFFSET,
                             state_.referee_sequences_[4]++,
                             referee_pack.power_heat);
      }
      if ((pending_sources & Referee::SOURCE_SENTRY_POS) != 0U) {
        SendRefereeFragments(REFEREE_ROBOT_POS_ID_OFFSET,
                             state_.referee_sequences_[10]++,
                             referee_pack.sentry_pos);
      }
      if ((pending_sources & Referee::SOURCE_ROBOT_BUFF) != 0U) {
        SendRefereeFragments(REFEREE_ROBOT_BUFF_ID_OFFSET,
                             state_.referee_sequences_[6]++,
                             referee_pack.robot_buff);
      }
      if ((pending_sources & Referee::SOURCE_ROBOT_DAMAGE) != 0U) {
        SendRefereeFragments(REFEREE_ROBOT_DAMAGE_ID_OFFSET,
                             state_.referee_sequences_[7]++,
                             referee_pack.robot_damage);
      }
      if ((pending_sources & Referee::SOURCE_BULLET_REMAIN) != 0U) {
        SendRefereeFragments(REFEREE_BULLET_REMAIN_ID_OFFSET,
                             state_.referee_sequences_[8]++,
                             referee_pack.bullet_remain);
      }
      if ((pending_sources & Referee::SOURCE_RFID) != 0U) {
        SendRefereeFragments(REFEREE_RFID_ID_OFFSET,
                             state_.referee_sequences_[9]++, referee_pack.rfid);
      }

      const bool STATUS_DUE = IsDue(now_ms, state_.next_referee_status_tx_ms_,
                                    REFEREE_STATUS_PERIOD_MS);
      if (status_pending || STATUS_DUE) {
        RefereeLinkStatusFrame frame{};
        frame.sequence = state_.referee_status_sequence_++;
        frame.referee_online = referee_pack.referee_online ? 1U : 0U;
        frame.supported_mask = Referee::SUPPORTED_SOURCE_MASK;
        frame.valid_mask = referee_pack.source_valid_mask;
        SendClassicFrame(tx_id_ + REFEREE_LINK_STATUS_ID_OFFSET, frame);
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
      } else if (offset == ATTITUDE_ID_OFFSET) {
        HandleAttitudeFrame(pack);
      }
    } else if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (HandleRefereeFrame(offset, pack)) {
        return;
      }
      if (offset == CHASSIS_YAW_ID_OFFSET) {
        HandleChassisYawFrame(pack);
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
                         uint8_t fragment_count, RefereeAssembly& assembly,
                         const LibXR::CAN::ClassicPack& pack, uint32_t now_ms,
                         Data& output, uint16_t source_mask,
                         Referee::CommandID command_id) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (offset < base_offset || offset >= base_offset + fragment_count) {
        return false;
      }

      RefereeFragmentFrame frame{};
      LoadClassicFrame(pack, frame);
      const auto result = RefereeCanCodec::Push(
          assembly, static_cast<size_t>(offset - base_offset), frame, now_ms,
          output);
      if (result != RefereeCanCodec::PushResult::COMPLETE) return false;
      state_.local_referee_valid_mask_ |= source_mask;
      if (source_mask == Referee::SOURCE_SENTRY_POS) {
        state_.gimbal_sentry_ref_.sentry_pos_received_time_ms = now_ms;
      } else if (source_mask == Referee::SOURCE_ROBOT_HP) {
        state_.gimbal_sentry_ref_.robot_hp_received_time_ms = now_ms;
      }
      state_.gimbal_sentry_ref_.source_command_id =
          static_cast<uint16_t>(command_id);
      state_.gimbal_sentry_ref_.source_valid_mask =
          RefereeCanCodec::IntersectValidity(
              state_.upstream_referee_online_, state_.local_referee_valid_mask_,
              state_.upstream_referee_valid_mask_,
              Referee::SUPPORTED_SOURCE_MASK);
      state_.gimbal_sentry_ref_.referee_online =
          state_.upstream_referee_online_;
      sentry_ref_topic_.Publish(state_.gimbal_sentry_ref_);
      return true;
    }
    return false;
  }

  bool HandleRefereeFrame(uint32_t offset,
                          const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      const uint32_t now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      if (offset == REFEREE_LINK_STATUS_ID_OFFSET) {
        RefereeLinkStatusFrame frame{};
        LoadClassicFrame(pack, frame);
        state_.upstream_referee_online_ = frame.referee_online == 1U;
        state_.upstream_referee_valid_mask_ =
            frame.valid_mask & frame.supported_mask;
        if (!state_.upstream_referee_online_) {
          state_.local_referee_valid_mask_ = 0U;
        }
        state_.gimbal_sentry_ref_.source_command_id = 0U;
        state_.gimbal_sentry_ref_.source_valid_mask =
            RefereeCanCodec::IntersectValidity(
                state_.upstream_referee_online_,
                state_.local_referee_valid_mask_,
                state_.upstream_referee_valid_mask_,
                Referee::SUPPORTED_SOURCE_MASK);
        state_.gimbal_sentry_ref_.referee_online =
            state_.upstream_referee_online_;
        sentry_ref_topic_.Publish(state_.gimbal_sentry_ref_);
        last_rx_time_ms_ = now_ms;
        online_ = true;
        safe_state_published_ = false;
        return true;
      }
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
        state_.local_referee_valid_mask_ |= Referee::SOURCE_GAME_STATUS;
        state_.gimbal_sentry_ref_.source_command_id =
            static_cast<uint16_t>(Referee::CommandID::REF_CMD_ID_GAME_STATUS);
        state_.gimbal_sentry_ref_.source_valid_mask =
            RefereeCanCodec::IntersectValidity(
                state_.upstream_referee_online_,
                state_.local_referee_valid_mask_,
                state_.upstream_referee_valid_mask_,
                Referee::SUPPORTED_SOURCE_MASK);
        state_.gimbal_sentry_ref_.referee_online =
            state_.upstream_referee_online_;
        sentry_ref_topic_.Publish(state_.gimbal_sentry_ref_);
        last_rx_time_ms_ = now_ms;
        online_ = true;
        safe_state_published_ = false;
        return true;
      }

#define HANDLE_REFEREE_GROUP(BASE, COUNT, STATE, FIELD, MASK, COMMAND)     \
  if (ReassembleReferee(offset, BASE, COUNT, STATE, pack, now_ms,          \
                        state_.gimbal_sentry_ref_.FIELD, MASK, COMMAND)) { \
    last_rx_time_ms_ = now_ms;                                             \
    online_ = true;                                                        \
    safe_state_published_ = false;                                         \
    return true;                                                           \
  }
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_HP_ID_OFFSET, 3U,
                           state_.referee_assemblies_[1], robot_hp,
                           Referee::SOURCE_ROBOT_HP,
                           Referee::CommandID::REF_CMD_ID_GAME_ROBOT_HP)
      HANDLE_REFEREE_GROUP(REFEREE_FIELD_EVENT_ID_OFFSET, 1U,
                           state_.referee_assemblies_[2], field_event,
                           Referee::SOURCE_FIELD_EVENT,
                           Referee::CommandID::REF_CMD_ID_FIELD_EVENTS)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_STATUS_ID_OFFSET, 3U,
                           state_.referee_assemblies_[3], robot_status,
                           Referee::SOURCE_ROBOT_STATUS,
                           Referee::CommandID::REF_CMD_ID_ROBOT_STATUS)
      HANDLE_REFEREE_GROUP(REFEREE_POWER_HEAT_ID_OFFSET, 2U,
                           state_.referee_assemblies_[4], power_heat,
                           Referee::SOURCE_POWER_HEAT,
                           Referee::CommandID::REF_CMD_ID_POWER_HEAT_DATA)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_POS_ID_OFFSET, 6U,
                           state_.referee_assemblies_[5], sentry_pos,
                           Referee::SOURCE_SENTRY_POS,
                           Referee::CommandID::REF_CMD_ID_ROBOT_POS_TO_SENTRY)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_BUFF_ID_OFFSET, 2U,
                           state_.referee_assemblies_[6], robot_buff,
                           Referee::SOURCE_ROBOT_BUFF,
                           Referee::CommandID::REF_CMD_ID_ROBOT_BUFF)
      HANDLE_REFEREE_GROUP(REFEREE_ROBOT_DAMAGE_ID_OFFSET, 1U,
                           state_.referee_assemblies_[7], robot_damage,
                           Referee::SOURCE_ROBOT_DAMAGE,
                           Referee::CommandID::REF_CMD_ID_ROBOT_DMG)
      HANDLE_REFEREE_GROUP(REFEREE_BULLET_REMAIN_ID_OFFSET, 2U,
                           state_.referee_assemblies_[8], bullet_remain,
                           Referee::SOURCE_BULLET_REMAIN,
                           Referee::CommandID::REF_CMD_ID_BULLET_REMAINING)
      HANDLE_REFEREE_GROUP(
          REFEREE_RFID_ID_OFFSET, 1U, state_.referee_assemblies_[9], rfid,
          Referee::SOURCE_RFID, Referee::CommandID::REF_CMD_ID_RFID)
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

      CMD::ChassisCMD command{};
      command.source =
          decoded.source == Pldx::DualBoardControl::Source::NAVIGATION
              ? CMD::ChassisCommandSource::NAVIGATION
              : CMD::ChassisCommandSource::OPERATOR;
      command.operator_input = {decoded.operator_input.x,
                                decoded.operator_input.y,
                                decoded.operator_input.z};
      command.navigation_velocity = {decoded.navigation_velocity.vx_mps,
                                     decoded.navigation_velocity.vy_mps,
                                     decoded.navigation_velocity.wz_rad_s};
      command.self_define = static_cast<CMD::ChasStat>(decoded.self_define);
      state_.chassis_cmd_topic_.Publish(command);

      const uint8_t MODE = static_cast<uint8_t>(decoded.mode);
      if (restored || state_.remote_mode_ != MODE) {
        uint32_t mode = MODE;
        mode_topic_.Publish(mode);
        ForceRemoteMode(mode);
        state_.remote_mode_ = MODE;
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

  void HandleAttitudeFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (!online_) {
        return;
      }

      AttitudeFrame frame{};
      LoadClassicFrame(pack, frame);
      LibXR::EulerAngle<float> attitude(DecodeSigned(frame.roll, ANGLE_SCALE),
                                        DecodeSigned(frame.pitch, ANGLE_SCALE),
                                        DecodeSigned(frame.yaw, ANGLE_SCALE));
      state_.attitude_topic_.Publish(attitude);
    } else {
      UNUSED(pack);
    }
  }

  void HandleLauncherFeedbackFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LauncherFeedbackFrame frame{};
      LoadClassicFrame(pack, frame);

      Referee::LauncherPack launcher_pack{};
      launcher_pack.rs.shooter_heat_limit = frame.heat_limit;
      launcher_pack.rs.shooter_cooling_value = frame.cooling_rate;
      launcher_pack.ph.launcher_id1_17_heat = frame.heat;
      launcher_pack.ld.bullet_speed =
          DecodeUnsigned8(frame.bullet_speed_deci, BULLET_SPEED_SCALE);
      launcher_pack.rs.robot_level = frame.robot_level;
      launcher_ref_topic_.Publish(launcher_pack);

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
           (now_ms - last_rx_time_ms_) > offline_timeout_ms_;
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
      const auto now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      if ((now_ms - last_rx_time_ms_) <= offline_timeout_ms_) {
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
      state_.local_referee_valid_mask_ = 0U;
      state_.upstream_referee_valid_mask_ = 0U;
      state_.upstream_referee_online_ = false;
      state_.gimbal_sentry_ref_.source_command_id = 0U;
      state_.gimbal_sentry_ref_.source_valid_mask = 0U;
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
          mode == static_cast<uint8_t>(ChassisMode::ROTOR)
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
      CMD::ChassisCMD command{};
      command.self_define = CMD::ChasStat::NONE;
      state_.chassis_cmd_topic_.Publish(command);

      float zero_angle = 0.0f;
      state_.yaw_angle_topic_.Publish(zero_angle);
      state_.pitch_angle_topic_.Publish(zero_angle);
      LibXR::EulerAngle<float> attitude{};
      state_.attitude_topic_.Publish(attitude);

      uint32_t mode = static_cast<uint32_t>(ChassisMode::RELAX);
      mode_topic_.Publish(mode);
      ForceRemoteMode(static_cast<uint32_t>(ChassisMode::RELAX));
      state_.remote_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
    }
  }

  void ForceRemoteMode(uint32_t mode) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (chassis_ != nullptr) {
        chassis_->GetEvent().Active(mode);
      }
    } else {
      UNUSED(mode);
    }
  }

  bool IsSupportedMode(uint32_t mode) const {
    return mode == static_cast<uint32_t>(ChassisMode::RELAX) ||
           mode == static_cast<uint32_t>(ChassisMode::INDEPENDENT) ||
           mode == static_cast<uint32_t>(ChassisMode::ROTOR) ||
           mode == static_cast<uint32_t>(ChassisMode::FOLLOW) ||
           mode == static_cast<uint32_t>(ChassisMode::NAVIGATION);
  }

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
  LibXR::Thread rx_thread_;
  LibXR::Thread protocol_thread_;
  LibXR::Mutex data_mutex_;
  uint32_t next_control_tx_ms_ = 0;
  uint32_t last_rx_time_ms_ = 0;
  bool online_ = false;
  bool safe_state_published_ = false;

  struct GimbalState {
    LibXR::Topic chassis_motion_state_topic_;
    LibXR::Topic chassis_imu_yaw_topic_;
    LibXR::Topic chassis_imu_yaw_valid_topic_;
    LibXR::Topic chassis_capacitor_capacity_topic_;
    LibXR::Topic chassis_capacitor_valid_topic_;
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
    float local_yaw_angle_ = 0.0f;
    float local_pitch_angle_ = 0.0f;
    LibXR::EulerAngle<float> local_attitude_{};
    uint8_t rc_chassis_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
    uint8_t nav_chassis_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
    uint8_t last_output_chassis_mode_ =
        static_cast<uint8_t>(ChassisMode::RELAX);
    bool nav_mode_valid_ = false;
    bool output_relax_ = false;
    bool output_mode_published_ = false;
    bool local_use_capacitor_ = true;
    Referee::RobotGameRefereePack gimbal_sentry_ref_{};
    RefereeAssembly referee_assemblies_[10]{};
    uint16_t local_referee_valid_mask_ = 0U;
    uint16_t upstream_referee_valid_mask_ = 0U;
    bool upstream_referee_online_ = false;
    ChassisMotionState motion_state_{};
    SentryDecisionFrame pending_decision_{
        SentryDecision::VERSION, 0U, 0U, 0U, 0U, 0U, 0U};
    SentryDecision::RetryController decision_retry_{};
    uint32_t last_chassis_yaw_rx_ms_ = 0U;
    uint32_t last_capacitor_rx_ms_ = 0U;
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
    LibXR::Topic chassis_cmd_topic_;
    LibXR::Topic yaw_angle_topic_;
    LibXR::Topic pitch_angle_topic_;
    LibXR::Topic attitude_topic_;
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
    uint32_t next_capacitor_tx_ms_ = 0U;
    uint32_t next_launcher_feedback_tx_ms_ = 0;
    uint32_t next_referee_status_tx_ms_ = 0U;
    uint8_t remote_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
    uint8_t chassis_yaw_sequence_ = 0U;
    uint8_t capacitor_sequence_ = 0U;
    uint8_t referee_sequences_[11]{};
    uint8_t referee_status_sequence_ = 0U;
  };

  std::conditional_t<ROLE == DualBoardRole::GIMBAL, GimbalState, ChassisState>
      state_;
};
