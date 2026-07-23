#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 双板定频 CAN 业务帧模块，通过 LibXR Topic 保持上层接口并使用 Classic CAN 固定帧同步双板状态
constructor_args:
  - can_bus_name: "can2"
  - tx_id: 0x312
  - rx_id: 0x311
  - rx_buffer_size: 256
  - tx_slot_count: 8
  - offline_timeout_ms: 100
  - chassis: '@nullptr'
  - mode_topic_name: "dualboard_chassis_mode"
  - cmd: '@nullptr'
  - sentry_buy_bullet_num_topic_name: "sentry_buy_bullet_num"
  - sentry_remote_buy_bullet_times_topic_name: "sentry_remote_buy_bullet_times"
  - sentry_remote_buy_hp_times_topic_name: "sentry_remote_buy_hp_times"
  - sentry_buy_resurrection_topic_name: "sentry_buy_resurrection"
  - sentry_state_topic_name: "sentry_state"
template_args:
  - ROLE: DualBoardRole::GIMBAL
  - ChassisType: Omni
required_hardware:
  - can

depends:
  - pldx/CMD
  - pldx/Chassis
  - pldx/Referee
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "ChassisMotionState.hpp"
#include "Referee.hpp"
#include "RefereeCanCodec.hpp"
#include "SentryDecisionFrame.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "queue/mpmc_queue.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "transform.hpp"

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
      uint32_t rx_buffer_size, uint32_t tx_slot_count,
      uint32_t offline_timeout_ms, Chassis<ChassisType>* chassis,
      const char* mode_topic_name = "dualboard_chassis_mode",
      CMD* cmd = nullptr,
      const char* sentry_buy_bullet_num_topic_name = "sentry_buy_bullet_num",
      const char* sentry_remote_buy_bullet_times_topic_name =
          "sentry_remote_buy_bullet_times",
      const char* sentry_remote_buy_hp_times_topic_name =
          "sentry_remote_buy_hp_times",
      const char* sentry_buy_resurrection_topic_name =
          "sentry_buy_resurrection",
      const char* sentry_state_topic_name = "sentry_state")
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})),
        tx_id_(tx_id),
        rx_id_(rx_id),
        rx_buffer_size_(rx_buffer_size),
        tx_slot_count_(tx_slot_count),
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
        rx_frames_(rx_buffer_size) {
    ASSERT(rx_buffer_size_ > 0U);
    ASSERT(tx_slot_count_ > 0U);
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

  void OnMonitor() override {
    if (offline_timeout_ms_ == 0U || last_rx_time_ms_ == 0U) {
      return;
    }

    auto now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    online_ = (now_ms - last_rx_time_ms_) <= offline_timeout_ms_;
  }

  LibXR::Event& GetEvent() { return dual_board_event_; }

  bool IsOnline() const { return online_; }

 private:
  struct __attribute__((packed)) ControlFrame {
    int16_t x;
    int16_t y;
    int16_t z;
    int8_t self_define;
    uint8_t mode;
  };

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

  enum class DecisionUpdateKind : uint8_t {
    BUY_BULLET,
    REMOTE_BUY_BULLET,
    REMOTE_BUY_HP,
    BUY_RESURRECTION,
    STATE,
  };

  struct DecisionUpdate {
    DecisionUpdateKind kind;
    uint16_t value;
  };

  using RefereeAssembly = RefereeCanCodec::Assembly;

  static constexpr uint32_t CONTROL_PERIOD_MS = 10;
  static constexpr uint32_t LAUNCHER_FEEDBACK_PERIOD_MS = 20;
  static constexpr uint32_t PROTOCOL_THREAD_PERIOD_MS = 2;
  static constexpr uint32_t CONTROL_ID_OFFSET = 0x00U;
  static constexpr uint32_t ANGLE_ID_OFFSET = 0x10U;
  static constexpr uint16_t DECISION_ID_OFFSET = 0x1fU;
  static constexpr uint32_t ATTITUDE_ID_OFFSET = 0x20U;
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
  static constexpr uint32_t RX_ID_RANGE = ATTITUDE_ID_OFFSET;
  static constexpr float COMMAND_SCALE = 32767.0f;
  static constexpr float COMMAND_LIMIT = 1.0f;
  static constexpr float ANGLE_SCALE = 10000.0f;
  static constexpr float ANGLE_LIMIT = 3.2f;
  static constexpr float GYRO_SCALE = 900.0f;
  static constexpr float BULLET_SPEED_SCALE = 10.0f;
  static constexpr float BULLET_SPEED_LIMIT = 25.5f;
  static_assert(sizeof(ControlFrame) == 8,
                "ControlFrame must be one classic CAN frame");
  static_assert(sizeof(AngleFrame) == 8,
                "AngleFrame must be one classic CAN frame");
  static_assert(sizeof(MotionFrame) == 8,
                "MotionFrame must be one classic CAN frame");
  static_assert(sizeof(AttitudeFrame) == 8,
                "AttitudeFrame must be one classic CAN frame");
  static_assert(sizeof(LauncherFeedbackFrame) == 8,
                "LauncherFeedbackFrame must be one classic CAN frame");
  static_assert(sizeof(RefereeFragmentFrame) == 8,
                "RefereeFragmentFrame must be one classic CAN frame");
  static_assert(sizeof(RefereeLinkStatusFrame) == 8,
                "RefereeLinkStatusFrame must be one classic CAN frame");

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
  LibXR::Topic CreateTopic(const char* name) {
    return LibXR::Topic::CreateTopic<Data>(name, nullptr, true);
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

      chassis_motion_state_topic_ =
          LibXR::Topic(LibXR::Topic::FindOrCreate<ChassisMotionState>(
              CHASSIS_MOTION_STATE_TOPIC_NAME, nullptr,
              CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER));

      launcher_ref_topic_ = CreateTopic<Referee::LauncherPack>("launcher_ref");
      sentry_ref_topic_ =
          CreateTopic<Referee::RobotGameRefereePack>("sentry_ref");
    } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
      chassis_cmd_topic_ = CreateTopic<CMD::ChassisCMD>("chassis_cmd");
      yaw_angle_topic_ = CreateTopic<float>("yawmotor_angle");
      pitch_angle_topic_ = CreateTopic<float>("pitchmotor_angle");
      attitude_topic_ = CreateTopic<LibXR::EulerAngle<float>>("gimbal_euler");

      RegisterTopicCallback<Referee::LauncherPack,
                            &DualBoard::OnLocalLauncherFeedback>(
          "launcher_ref");
      RegisterTopicCallback<Referee::RobotGameRefereePack,
                            &DualBoard::OnLocalSentryRef>("sentry_ref");
      RegisterTopicCallback<Eigen::Matrix<float, 3, 1>,
                            &DualBoard::OnLocalChassisGyro>("chassis_gyro");
    }
  }

  void RegisterDecisionTopics() {
    sentry_buy_bullet_num_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<uint16_t>(
            sentry_buy_bullet_num_topic_name_, nullptr, true));
    sentry_remote_buy_bullet_times_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<uint8_t>(
            sentry_remote_buy_bullet_times_topic_name_, nullptr, true));
    sentry_remote_buy_hp_times_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<uint8_t>(
            sentry_remote_buy_hp_times_topic_name_, nullptr, true));
    sentry_buy_resurrection_topic_ =
        LibXR::Topic(LibXR::Topic::FindOrCreate<bool>(
            sentry_buy_resurrection_topic_name_, nullptr, true));
    sentry_state_topic_ = LibXR::Topic(LibXR::Topic::FindOrCreate<uint8_t>(
        sentry_state_topic_name_, nullptr, true));

    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      RegisterTopicCallback<uint16_t, &DualBoard::OnSentryBuyBullet>(
          sentry_buy_bullet_num_topic_name_);
      RegisterTopicCallback<uint8_t, &DualBoard::OnSentryRemoteBuyBullet>(
          sentry_remote_buy_bullet_times_topic_name_);
      RegisterTopicCallback<uint8_t, &DualBoard::OnSentryRemoteBuyHp>(
          sentry_remote_buy_hp_times_topic_name_);
      RegisterTopicCallback<bool, &DualBoard::OnSentryBuyResurrection>(
          sentry_buy_resurrection_topic_name_);
      RegisterTopicCallback<uint8_t, &DualBoard::OnSentryState>(
          sentry_state_topic_name_);
    }
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
            self->SetLocalModeRelax();
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
  }

  void OnLocalChassisCommand(const CMD::ChassisCMD& command) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_chassis_command_ = command;
    } else {
      UNUSED(command);
    }
  }

  void OnLocalYawAngle(const float& yaw_angle) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_yaw_angle_ = yaw_angle;
    } else {
      UNUSED(yaw_angle);
    }
  }

  void OnLocalPitchAngle(const float& pitch_angle) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_pitch_angle_ = pitch_angle;
    } else {
      UNUSED(pitch_angle);
    }
  }

  void OnLocalAttitude(const LibXR::EulerAngle<float>& attitude) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_attitude_ = attitude;
    } else {
      UNUSED(attitude);
    }
  }

  void OnLocalLauncherFeedback(const Referee::LauncherPack& launcher_pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_launcher_pack_ = launcher_pack;
      launcher_feedback_valid_ = true;
    } else {
      UNUSED(launcher_pack);
    }
  }

  void OnLocalSentryRef(const Referee::RobotGameRefereePack& referee_pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_sentry_ref_ = referee_pack;
      pending_referee_sources_ |= SourceMaskForCommand(
          static_cast<Referee::CommandID>(referee_pack.source_command_id));
      referee_status_pending_ = true;
    } else {
      UNUSED(referee_pack);
    }
  }

  void OnLocalChassisGyro(const Eigen::Matrix<float, 3, 1>& gyro) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_chassis_gyro_ = gyro;
      chassis_gyro_received_ = true;
    } else {
      UNUSED(gyro);
    }
  }

  void EnqueueDecisionUpdate(DecisionUpdateKind kind, uint16_t value) {
    const DecisionUpdate update{kind, value};
    if (decision_updates_.Push(update) != LibXR::ErrorCode::OK) {
      decision_update_drops_.fetch_add(1U, std::memory_order_relaxed);
    }
  }

  void OnSentryBuyBullet(const uint16_t& value) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      EnqueueDecisionUpdate(DecisionUpdateKind::BUY_BULLET, value);
    } else {
      UNUSED(value);
    }
  }

  void OnSentryRemoteBuyBullet(const uint8_t& value) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      EnqueueDecisionUpdate(DecisionUpdateKind::REMOTE_BUY_BULLET, value);
    } else {
      UNUSED(value);
    }
  }

  void OnSentryRemoteBuyHp(const uint8_t& value) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      EnqueueDecisionUpdate(DecisionUpdateKind::REMOTE_BUY_HP, value);
    } else {
      UNUSED(value);
    }
  }

  void OnSentryBuyResurrection(const bool& value) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      EnqueueDecisionUpdate(DecisionUpdateKind::BUY_RESURRECTION,
                            value ? 1U : 0U);
    } else {
      UNUSED(value);
    }
  }

  void OnSentryState(const uint8_t& value) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      EnqueueDecisionUpdate(DecisionUpdateKind::STATE, value);
    } else {
      UNUSED(value);
    }
  }

  void OnLocalModeEvent(uint32_t event_id) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (!IsSupportedMode(event_id)) {
        return;
      }

      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_chassis_mode_ = static_cast<uint8_t>(event_id);
      local_mode_valid_ = true;
      uint32_t mode = event_id;
      mode_topic_.Publish(mode);
      motion_state_.mode = event_id == static_cast<uint32_t>(ChassisMode::ROTOR)
                               ? ChassisMotionMode::ROTOR
                               : ChassisMotionMode::NON_ROTOR;
      PublishMotionStateLocked();
    } else {
      UNUSED(event_id);
    }
  }

  void SetLocalModeRelax() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      local_chassis_command_ = {};
      local_chassis_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
      local_mode_valid_ = true;
      uint32_t mode = static_cast<uint32_t>(ChassisMode::RELAX);
      mode_topic_.Publish(mode);
      motion_state_.mode = ChassisMotionMode::NON_ROTOR;
      PublishMotionStateLocked();
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
        CheckOffline(now_ms);
      } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
        SendMotionFrameIfDue(now_ms);
        SendLauncherFeedbackFrameIfDue(now_ms);
        SendRefereeFramesIfDue(now_ms);
        CheckOffline(now_ms);
      }

      protocol_thread_.SleepUntil(last_time, PROTOCOL_THREAD_PERIOD_MS);
    }
  }

  void DrainDecisionUpdates() {
    if constexpr (ROLE != DualBoardRole::GIMBAL) {
      return;
    }

    DecisionUpdate update{};
    while (decision_updates_.Pop(update) == LibXR::ErrorCode::OK) {
      switch (update.kind) {
        case DecisionUpdateKind::BUY_BULLET: {
          const uint32_t TOTAL =
              pending_decision_.buy_bullet_delta + update.value;
          pending_decision_.buy_bullet_delta =
              static_cast<uint16_t>(std::min(TOTAL, 2047U));
          pending_decision_.valid_mask |= SentryDecision::BUY_BULLET_VALID;
          break;
        }
        case DecisionUpdateKind::REMOTE_BUY_BULLET: {
          const uint8_t BULLET_COUNT = static_cast<uint8_t>(
              std::min<uint16_t>(SentryDecision::RemoteBulletCount(
                                     pending_decision_.remote_request_counts) +
                                     update.value,
                                 15U));
          const uint8_t HP_COUNT = SentryDecision::RemoteHpCount(
              pending_decision_.remote_request_counts);
          pending_decision_.remote_request_counts =
              SentryDecision::PackRemoteCounts(BULLET_COUNT, HP_COUNT);
          pending_decision_.valid_mask |= SentryDecision::REMOTE_BULLET_VALID;
          break;
        }
        case DecisionUpdateKind::REMOTE_BUY_HP: {
          const uint8_t BULLET_COUNT = SentryDecision::RemoteBulletCount(
              pending_decision_.remote_request_counts);
          const uint8_t HP_COUNT = static_cast<uint8_t>(
              std::min<uint16_t>(SentryDecision::RemoteHpCount(
                                     pending_decision_.remote_request_counts) +
                                     update.value,
                                 15U));
          pending_decision_.remote_request_counts =
              SentryDecision::PackRemoteCounts(BULLET_COUNT, HP_COUNT);
          pending_decision_.valid_mask |= SentryDecision::REMOTE_HP_VALID;
          break;
        }
        case DecisionUpdateKind::BUY_RESURRECTION:
          if (update.value != 0U) {
            pending_decision_.flags |= SentryDecision::BUY_RESURRECTION_FLAG;
          } else {
            pending_decision_.flags &=
                static_cast<uint8_t>(~SentryDecision::BUY_RESURRECTION_FLAG);
          }
          pending_decision_.valid_mask |=
              SentryDecision::BUY_RESURRECTION_VALID;
          break;
        case DecisionUpdateKind::STATE:
          pending_decision_.state = static_cast<uint8_t>(update.value);
          pending_decision_.valid_mask |= SentryDecision::STATE_VALID;
          break;
      }
    }
  }

  void SendDecisionFrameIfDue(uint32_t now_ms) {
    if constexpr (ROLE != DualBoardRole::GIMBAL) {
      UNUSED(now_ms);
      return;
    }

    if (!decision_retry_.Active() && pending_decision_.valid_mask != 0U) {
      active_decision_ = pending_decision_;
      pending_decision_ = {};
      pending_decision_.version = SentryDecision::VERSION;
      active_decision_.sequence = ++decision_sequence_;
      const bool STARTED = decision_retry_.Begin(
          active_decision_, active_decision_.sequence, now_ms);
      ASSERT(STARTED);
    }

    if (!decision_retry_.Due(now_ms)) {
      return;
    }

    const bool SENT =
        SendClassicFrame(tx_id_ + DECISION_ID_OFFSET, active_decision_);
    decision_retry_.OnSendResult(SENT, now_ms);
  }

  void ReportDecisionUpdateDrops(uint32_t now_ms) {
    if constexpr (ROLE != DualBoardRole::GIMBAL) {
      UNUSED(now_ms);
      return;
    }

    const uint32_t DROPS =
        decision_update_drops_.load(std::memory_order_relaxed);
    if (DROPS == reported_decision_update_drops_) {
      return;
    }
    if (decision_drop_log_started_ &&
        now_ms - last_decision_drop_log_ms_ < DECISION_DROP_LOG_PERIOD_MS) {
      return;
    }

    XR_LOG_WARN("DualBoard decision update queue dropped {} items", DROPS);
    reported_decision_update_drops_ = DROPS;
    last_decision_drop_log_ms_ = now_ms;
    decision_drop_log_started_ = true;
  }

  void SendMotionFrameIfDue(uint32_t now_ms) {
    if (!IsDue(now_ms, next_control_tx_ms_, CONTROL_PERIOD_MS)) {
      return;
    }

    Eigen::Matrix<float, 3, 1> gyro{};
    bool gyro_received = false;
    {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      gyro = local_chassis_gyro_;
      gyro_received = chassis_gyro_received_;
    }

    MotionFrame frame{};
    float gyro_z = 0.0f;
    if (gyro_received) {
      gyro_z = gyro.z();
    }
    if (gyro_received && std::isfinite(gyro_z)) {
      float gyro_z_q = gyro_z * GYRO_SCALE;
      if (gyro_z_q >= static_cast<float>(std::numeric_limits<int16_t>::min()) &&
          gyro_z_q <= static_cast<float>(std::numeric_limits<int16_t>::max())) {
        frame.gyro_z_q = static_cast<int16_t>(gyro_z_q);
        frame.gyro_valid = 1U;
      }
    }

    SendClassicFrame(tx_id_ + ANGLE_ID_OFFSET, frame);
  }

  void SendGimbalControlFrames(uint32_t now_ms) {
    if (!IsDue(now_ms, next_control_tx_ms_, CONTROL_PERIOD_MS)) {
      return;
    }

    CMD::ChassisCMD command{};
    float yaw_angle = 0.0f;
    float pitch_angle = 0.0f;
    LibXR::EulerAngle<float> attitude{};
    uint8_t mode = static_cast<uint8_t>(ChassisMode::RELAX);

    {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      command = local_chassis_command_;
      yaw_angle = local_yaw_angle_;
      pitch_angle = local_pitch_angle_;
      attitude = local_attitude_;
      if (local_mode_valid_) {
        mode = local_chassis_mode_;
      }
    }

    ControlFrame control_frame{};
    control_frame.x = EncodeSigned(command.x, COMMAND_SCALE, COMMAND_LIMIT);
    control_frame.y = EncodeSigned(command.y, COMMAND_SCALE, COMMAND_LIMIT);
    control_frame.z = EncodeSigned(command.z, COMMAND_SCALE, COMMAND_LIMIT);
    control_frame.self_define = static_cast<int8_t>(command.self_define);
    control_frame.mode = mode;

    AngleFrame angle_frame{};
    angle_frame.yaw = EncodeSigned(yaw_angle, ANGLE_SCALE, ANGLE_LIMIT);
    angle_frame.pitch = EncodeSigned(pitch_angle, ANGLE_SCALE, ANGLE_LIMIT);
    angle_frame.sequence = tx_sequence_++;

    AttitudeFrame attitude_frame{};
    attitude_frame.roll =
        EncodeSigned(attitude.Roll(), ANGLE_SCALE, ANGLE_LIMIT);
    attitude_frame.pitch =
        EncodeSigned(attitude.Pitch(), ANGLE_SCALE, ANGLE_LIMIT);
    attitude_frame.yaw = EncodeSigned(attitude.Yaw(), ANGLE_SCALE, ANGLE_LIMIT);
    attitude_frame.sequence = angle_frame.sequence;

    SendClassicFrame(tx_id_ + CONTROL_ID_OFFSET, control_frame);
    SendClassicFrame(tx_id_ + ANGLE_ID_OFFSET, angle_frame);
    SendClassicFrame(tx_id_ + ATTITUDE_ID_OFFSET, attitude_frame);
  }

  void SendLauncherFeedbackFrameIfDue(uint32_t now_ms) {
    if (!IsDue(now_ms, next_launcher_feedback_tx_ms_,
               LAUNCHER_FEEDBACK_PERIOD_MS)) {
      return;
    }

    Referee::LauncherPack launcher_pack{};
    bool valid = false;
    {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      launcher_pack = local_launcher_pack_;
      valid = launcher_feedback_valid_;
    }

    if (!valid) {
      return;
    }

    LauncherFeedbackFrame frame{};
    frame.heat_limit = launcher_pack.rs.shooter_heat_limit;
    frame.cooling_rate = launcher_pack.rs.shooter_cooling_value;
    frame.heat = launcher_pack.ph.launcher_id1_17_heat;
    frame.bullet_speed_deci = EncodeUnsigned8(
        launcher_pack.ld.bullet_speed, BULLET_SPEED_SCALE, BULLET_SPEED_LIMIT);
    frame.robot_level = launcher_pack.rs.robot_level;

    SendClassicFrame(tx_id_ + CONTROL_ID_OFFSET, frame);
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
    if constexpr (ROLE != DualBoardRole::CHASSIS) {
      UNUSED(now_ms);
      return;
    }

    Referee::RobotGameRefereePack referee_pack{};
    uint16_t pending_sources = 0U;
    bool status_pending = false;
    {
      LibXR::Mutex::LockGuard lock(data_mutex_);
      referee_pack = local_sentry_ref_;
      pending_sources = pending_referee_sources_;
      pending_referee_sources_ = 0U;
      status_pending = referee_status_pending_;
      referee_status_pending_ = false;
    }

    if ((pending_sources & Referee::SOURCE_GAME_STATUS) != 0U) {
      RefereeGameStatusData data{referee_pack.game_status.game_type,
                                 referee_pack.game_status.game_progress,
                                 referee_pack.game_status.stage_remain_time};
      SendRefereeFragments(REFEREE_GAME_STATUS_ID_OFFSET,
                           referee_sequences_[0]++, data);
    }
    if ((pending_sources & Referee::SOURCE_ROBOT_HP) != 0U) {
      SendRefereeFragments(REFEREE_ROBOT_HP_ID_OFFSET, referee_sequences_[1]++,
                           referee_pack.robot_hp);
    }
    if ((pending_sources & Referee::SOURCE_FIELD_EVENT) != 0U) {
      SendRefereeFragments(REFEREE_FIELD_EVENT_ID_OFFSET,
                           referee_sequences_[2]++, referee_pack.field_event);
    }
    if ((pending_sources & Referee::SOURCE_ROBOT_STATUS) != 0U) {
      SendRefereeFragments(REFEREE_ROBOT_STATUS_ID_OFFSET,
                           referee_sequences_[3]++, referee_pack.robot_status);
    }
    if ((pending_sources & Referee::SOURCE_POWER_HEAT) != 0U) {
      SendRefereeFragments(REFEREE_POWER_HEAT_ID_OFFSET,
                           referee_sequences_[4]++, referee_pack.power_heat);
    }
    if ((pending_sources & Referee::SOURCE_ROBOT_POS) != 0U) {
      SendRefereeFragments(REFEREE_ROBOT_POS_ID_OFFSET, referee_sequences_[5]++,
                           referee_pack.robot_pos);
    }
    if ((pending_sources & Referee::SOURCE_ROBOT_BUFF) != 0U) {
      SendRefereeFragments(REFEREE_ROBOT_BUFF_ID_OFFSET,
                           referee_sequences_[6]++, referee_pack.robot_buff);
    }
    if ((pending_sources & Referee::SOURCE_ROBOT_DAMAGE) != 0U) {
      SendRefereeFragments(REFEREE_ROBOT_DAMAGE_ID_OFFSET,
                           referee_sequences_[7]++, referee_pack.robot_damage);
    }
    if ((pending_sources & Referee::SOURCE_BULLET_REMAIN) != 0U) {
      SendRefereeFragments(REFEREE_BULLET_REMAIN_ID_OFFSET,
                           referee_sequences_[8]++, referee_pack.bullet_remain);
    }
    if ((pending_sources & Referee::SOURCE_RFID) != 0U) {
      SendRefereeFragments(REFEREE_RFID_ID_OFFSET, referee_sequences_[9]++,
                           referee_pack.rfid);
    }

    const bool STATUS_DUE =
        IsDue(now_ms, next_referee_status_tx_ms_, REFEREE_STATUS_PERIOD_MS);
    if (status_pending || STATUS_DUE) {
      RefereeLinkStatusFrame frame{};
      frame.sequence = referee_status_sequence_++;
      frame.referee_online = referee_pack.referee_online ? 1U : 0U;
      frame.supported_mask = Referee::SUPPORTED_SOURCE_MASK;
      frame.valid_mask = referee_pack.source_valid_mask;
      SendClassicFrame(tx_id_ + REFEREE_LINK_STATUS_ID_OFFSET, frame);
    }
  }

  template <typename Frame>
  bool SendClassicFrame(uint32_t id, const Frame& frame) {
    LibXR::CAN::ClassicPack pack{};
    pack.id = id;
    pack.type = LibXR::CAN::Type::STANDARD;
    pack.dlc = sizeof(Frame);
    std::memcpy(pack.data, &frame, sizeof(Frame));
    return can_->AddMessage(pack) == LibXR::ErrorCode::OK;
  }

  void HandleCanFrame(const LibXR::CAN::ClassicPack& pack) {
    if (pack.type != LibXR::CAN::Type::STANDARD || pack.dlc != 8U) {
      return;
    }

    auto offset = pack.id - rx_id_;
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      if (offset == CONTROL_ID_OFFSET) {
        HandleControlFrame(pack);
      } else if (offset == ANGLE_ID_OFFSET) {
        HandleAngleFrame(pack);
      } else if (offset == DECISION_ID_OFFSET) {
        HandleDecisionFrame(pack);
      } else if (offset == ATTITUDE_ID_OFFSET) {
        HandleAttitudeFrame(pack);
      }
    } else if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (HandleRefereeFrame(offset, pack)) {
        return;
      }
      if (offset == CONTROL_ID_OFFSET) {
        HandleLauncherFeedbackFrame(pack);
      } else if (offset == ANGLE_ID_OFFSET) {
        HandleMotionFrame(pack);
      }
    }
  }

  void HandleDecisionFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      SentryDecisionFrame frame{};
      std::memcpy(&frame, pack.data, sizeof(frame));
      if (!SentryDecision::Validate(frame)) {
        return;
      }

      const uint32_t now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      last_decision_rx_time_ms_ = now_ms;
      if (!decision_sequence_tracker_.Accept(frame.sequence, now_ms)) {
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
    if (offset < base_offset || offset >= base_offset + fragment_count) {
      return false;
    }

    RefereeFragmentFrame frame{};
    std::memcpy(&frame, pack.data, sizeof(frame));
    const auto result = RefereeCanCodec::Push(
        assembly, static_cast<size_t>(offset - base_offset), frame, now_ms,
        output);
    if (result != RefereeCanCodec::PushResult::COMPLETE) {
      return true;
    }
    local_referee_valid_mask_ |= source_mask;
    gimbal_sentry_ref_.source_command_id = static_cast<uint16_t>(command_id);
    gimbal_sentry_ref_.source_valid_mask = RefereeCanCodec::IntersectValidity(
        upstream_referee_online_, local_referee_valid_mask_,
        upstream_referee_valid_mask_, Referee::SUPPORTED_SOURCE_MASK);
    gimbal_sentry_ref_.referee_online = upstream_referee_online_;
    sentry_ref_topic_.Publish(gimbal_sentry_ref_);
    return true;
  }

  bool HandleRefereeFrame(uint32_t offset,
                          const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE != DualBoardRole::GIMBAL) {
      UNUSED(offset);
      UNUSED(pack);
      return false;
    }
    const uint32_t now_ms =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    if (offset == REFEREE_LINK_STATUS_ID_OFFSET) {
      RefereeLinkStatusFrame frame{};
      std::memcpy(&frame, pack.data, sizeof(frame));
      upstream_referee_online_ = frame.referee_online == 1U;
      upstream_referee_valid_mask_ = frame.valid_mask & frame.supported_mask;
      if (!upstream_referee_online_) {
        local_referee_valid_mask_ = 0U;
      }
      gimbal_sentry_ref_.source_command_id = 0U;
      gimbal_sentry_ref_.source_valid_mask = RefereeCanCodec::IntersectValidity(
          upstream_referee_online_, local_referee_valid_mask_,
          upstream_referee_valid_mask_, Referee::SUPPORTED_SOURCE_MASK);
      gimbal_sentry_ref_.referee_online = upstream_referee_online_;
      sentry_ref_topic_.Publish(gimbal_sentry_ref_);
      last_rx_time_ms_ = now_ms;
      online_ = true;
      safe_state_published_ = false;
      return true;
    }
    if (offset == REFEREE_GAME_STATUS_ID_OFFSET) {
      RefereeFragmentFrame frame{};
      std::memcpy(&frame, pack.data, sizeof(frame));
      auto& assembly = referee_assemblies_[0];
      if (assembly.has_published &&
          assembly.published_sequence == frame.sequence) {
        return true;
      }
      assembly.has_published = true;
      assembly.published_sequence = frame.sequence;
      gimbal_sentry_ref_.game_status.game_type = frame.data[0];
      gimbal_sentry_ref_.game_status.game_progress = frame.data[1];
      std::memcpy(&gimbal_sentry_ref_.game_status.stage_remain_time,
                  &frame.data[2], sizeof(uint16_t));
      local_referee_valid_mask_ |= Referee::SOURCE_GAME_STATUS;
      gimbal_sentry_ref_.source_command_id =
          static_cast<uint16_t>(Referee::CommandID::REF_CMD_ID_GAME_STATUS);
      gimbal_sentry_ref_.source_valid_mask = RefereeCanCodec::IntersectValidity(
          upstream_referee_online_, local_referee_valid_mask_,
          upstream_referee_valid_mask_, Referee::SUPPORTED_SOURCE_MASK);
      gimbal_sentry_ref_.referee_online = upstream_referee_online_;
      sentry_ref_topic_.Publish(gimbal_sentry_ref_);
      last_rx_time_ms_ = now_ms;
      online_ = true;
      safe_state_published_ = false;
      return true;
    }

#define HANDLE_REFEREE_GROUP(BASE, COUNT, STATE, FIELD, MASK, COMMAND) \
  if (ReassembleReferee(offset, BASE, COUNT, STATE, pack, now_ms,      \
                        gimbal_sentry_ref_.FIELD, MASK, COMMAND)) {    \
    last_rx_time_ms_ = now_ms;                                         \
    online_ = true;                                                    \
    safe_state_published_ = false;                                     \
    return true;                                                       \
  }
    HANDLE_REFEREE_GROUP(REFEREE_ROBOT_HP_ID_OFFSET, 3U, referee_assemblies_[1],
                         robot_hp, Referee::SOURCE_ROBOT_HP,
                         Referee::CommandID::REF_CMD_ID_GAME_ROBOT_HP)
    HANDLE_REFEREE_GROUP(REFEREE_FIELD_EVENT_ID_OFFSET, 1U,
                         referee_assemblies_[2], field_event,
                         Referee::SOURCE_FIELD_EVENT,
                         Referee::CommandID::REF_CMD_ID_FIELD_EVENTS)
    HANDLE_REFEREE_GROUP(REFEREE_ROBOT_STATUS_ID_OFFSET, 3U,
                         referee_assemblies_[3], robot_status,
                         Referee::SOURCE_ROBOT_STATUS,
                         Referee::CommandID::REF_CMD_ID_ROBOT_STATUS)
    HANDLE_REFEREE_GROUP(REFEREE_POWER_HEAT_ID_OFFSET, 2U,
                         referee_assemblies_[4], power_heat,
                         Referee::SOURCE_POWER_HEAT,
                         Referee::CommandID::REF_CMD_ID_POWER_HEAT_DATA)
    HANDLE_REFEREE_GROUP(
        REFEREE_ROBOT_POS_ID_OFFSET, 2U, referee_assemblies_[5], robot_pos,
        Referee::SOURCE_ROBOT_POS, Referee::CommandID::REF_CMD_ID_ROBOT_POS)
    HANDLE_REFEREE_GROUP(
        REFEREE_ROBOT_BUFF_ID_OFFSET, 2U, referee_assemblies_[6], robot_buff,
        Referee::SOURCE_ROBOT_BUFF, Referee::CommandID::REF_CMD_ID_ROBOT_BUFF)
    HANDLE_REFEREE_GROUP(REFEREE_ROBOT_DAMAGE_ID_OFFSET, 1U,
                         referee_assemblies_[7], robot_damage,
                         Referee::SOURCE_ROBOT_DAMAGE,
                         Referee::CommandID::REF_CMD_ID_ROBOT_DMG)
    HANDLE_REFEREE_GROUP(REFEREE_BULLET_REMAIN_ID_OFFSET, 2U,
                         referee_assemblies_[8], bullet_remain,
                         Referee::SOURCE_BULLET_REMAIN,
                         Referee::CommandID::REF_CMD_ID_BULLET_REMAINING)
    HANDLE_REFEREE_GROUP(REFEREE_RFID_ID_OFFSET, 1U, referee_assemblies_[9],
                         rfid, Referee::SOURCE_RFID,
                         Referee::CommandID::REF_CMD_ID_RFID)
#undef HANDLE_REFEREE_GROUP
    return false;
  }

  void HandleMotionFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      MotionFrame frame{};
      std::memcpy(&frame, pack.data, sizeof(frame));

      const auto now_ms =
          static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        last_rx_time_ms_ = now_ms;
        online_ = true;
        safe_state_published_ = false;
        const float gyro_z = DecodeSigned(frame.gyro_z_q, GYRO_SCALE);
        motion_state_.yaw_rate_rad_s =
            frame.gyro_valid == 1U && std::isfinite(gyro_z) ? gyro_z : 0.0f;
        motion_state_.yaw_rate_valid =
            frame.gyro_valid == 1U && std::isfinite(gyro_z);
        motion_state_.online = true;
        PublishMotionStateLocked();
      }
    } else {
      UNUSED(pack);
    }
  }

  void HandleControlFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      ControlFrame frame{};
      std::memcpy(&frame, pack.data, sizeof(frame));
      if (!IsSupportedMode(frame.mode)) {
        return;
      }

      CMD::ChassisCMD command{};
      command.x = DecodeSigned(frame.x, COMMAND_SCALE);
      command.y = DecodeSigned(frame.y, COMMAND_SCALE);
      command.z = DecodeSigned(frame.z, COMMAND_SCALE);
      command.self_define = static_cast<CMD::ChasStat>(frame.self_define);
      chassis_cmd_topic_.Publish(command);

      auto now_ms = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      bool restored = !online_;
      last_rx_time_ms_ = now_ms;
      online_ = true;
      safe_state_published_ = false;

      if (restored || remote_mode_ != frame.mode) {
        uint32_t mode = frame.mode;
        mode_topic_.Publish(mode);
        ForceRemoteMode(mode);
        remote_mode_ = frame.mode;
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
      std::memcpy(&frame, pack.data, sizeof(frame));
      float yaw_angle = DecodeSigned(frame.yaw, ANGLE_SCALE);
      float pitch_angle = DecodeSigned(frame.pitch, ANGLE_SCALE);
      yaw_angle_topic_.Publish(yaw_angle);
      pitch_angle_topic_.Publish(pitch_angle);
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
      std::memcpy(&frame, pack.data, sizeof(frame));
      LibXR::EulerAngle<float> attitude(DecodeSigned(frame.roll, ANGLE_SCALE),
                                        DecodeSigned(frame.pitch, ANGLE_SCALE),
                                        DecodeSigned(frame.yaw, ANGLE_SCALE));
      attitude_topic_.Publish(attitude);
    } else {
      UNUSED(pack);
    }
  }

  void HandleLauncherFeedbackFrame(const LibXR::CAN::ClassicPack& pack) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      LauncherFeedbackFrame frame{};
      std::memcpy(&frame, pack.data, sizeof(frame));

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

  void CheckOffline(uint32_t now_ms) {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      {
        LibXR::Mutex::LockGuard lock(data_mutex_);
        if (offline_timeout_ms_ == 0U || last_rx_time_ms_ == 0U ||
            (now_ms - last_rx_time_ms_) <= offline_timeout_ms_) {
          return;
        }

        online_ = false;
        if (safe_state_published_) {
          return;
        }
      }

      PublishOfflineState();
      return;
    }

    if (offline_timeout_ms_ == 0U || last_rx_time_ms_ == 0U) {
      return;
    }

    if ((now_ms - last_rx_time_ms_) <= offline_timeout_ms_) {
      return;
    }

    online_ = false;
    if (!safe_state_published_) {
      PublishOfflineState();
      safe_state_published_ = true;
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

      motion_state_ = {};
      PublishMotionStateLocked();
      launcher_feedback_valid_ = false;
      safe_state_published_ = true;
    }
  }

  void PublishInvalidReferee() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      local_referee_valid_mask_ = 0U;
      upstream_referee_valid_mask_ = 0U;
      upstream_referee_online_ = false;
      gimbal_sentry_ref_.source_command_id = 0U;
      gimbal_sentry_ref_.source_valid_mask = 0U;
      gimbal_sentry_ref_.referee_online = false;
      sentry_ref_topic_.Publish(gimbal_sentry_ref_);
    }
  }

  void PublishMotionStateLocked() {
    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      chassis_motion_state_topic_.Publish(motion_state_);
    }
  }

  void PublishSafeChassisState() {
    if constexpr (ROLE == DualBoardRole::CHASSIS) {
      CMD::ChassisCMD command{};
      command.self_define = CMD::ChasStat::NONE;
      chassis_cmd_topic_.Publish(command);

      float zero_angle = 0.0f;
      yaw_angle_topic_.Publish(zero_angle);
      pitch_angle_topic_.Publish(zero_angle);
      LibXR::EulerAngle<float> attitude{};
      attitude_topic_.Publish(attitude);

      uint32_t mode = static_cast<uint32_t>(ChassisMode::RELAX);
      mode_topic_.Publish(mode);
      ForceRemoteMode(static_cast<uint32_t>(ChassisMode::RELAX));
      remote_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
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
           mode == static_cast<uint32_t>(ChassisMode::FOLLOW);
  }

  LibXR::CAN* can_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t rx_buffer_size_;
  uint32_t tx_slot_count_;
  uint32_t offline_timeout_ms_;
  Chassis<ChassisType>* chassis_;
  CMD* cmd_;
  const char* mode_topic_name_;
  const char* sentry_buy_bullet_num_topic_name_;
  const char* sentry_remote_buy_bullet_times_topic_name_;
  const char* sentry_remote_buy_hp_times_topic_name_;
  const char* sentry_buy_resurrection_topic_name_;
  const char* sentry_state_topic_name_;

  LibXR::Topic mode_topic_;
  LibXR::Topic chassis_cmd_topic_;
  LibXR::Topic yaw_angle_topic_;
  LibXR::Topic pitch_angle_topic_;
  LibXR::Topic attitude_topic_;
  LibXR::Topic launcher_ref_topic_;
  LibXR::Topic sentry_ref_topic_;
  LibXR::Topic sentry_buy_bullet_num_topic_;
  LibXR::Topic sentry_remote_buy_bullet_times_topic_;
  LibXR::Topic sentry_remote_buy_hp_times_topic_;
  LibXR::Topic sentry_buy_resurrection_topic_;
  LibXR::Topic sentry_state_topic_;
  LibXR::Topic chassis_motion_state_topic_;
  LibXR::Event dual_board_event_;
  LibXR::CAN::Callback can_rx_callback_;

  LibXR::MPMCQueue<LibXR::CAN::ClassicPack> rx_frames_;
  LibXR::MPMCQueue<DecisionUpdate> decision_updates_{32};
  std::atomic<uint32_t> decision_update_drops_{0};
  LibXR::Semaphore rx_sem_;
  LibXR::Thread rx_thread_;
  LibXR::Thread protocol_thread_;
  LibXR::Mutex data_mutex_;

  CMD::ChassisCMD local_chassis_command_{};
  float local_yaw_angle_ = 0.0f;
  float local_pitch_angle_ = 0.0f;
  LibXR::EulerAngle<float> local_attitude_{};
  uint8_t local_chassis_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
  bool local_mode_valid_ = false;
  Referee::LauncherPack local_launcher_pack_{};
  Referee::RobotGameRefereePack local_sentry_ref_{};
  uint16_t pending_referee_sources_ = 0U;
  bool referee_status_pending_ = false;
  Referee::RobotGameRefereePack gimbal_sentry_ref_{};
  RefereeAssembly referee_assemblies_[10]{};
  uint16_t local_referee_valid_mask_ = 0U;
  uint16_t upstream_referee_valid_mask_ = 0U;
  bool upstream_referee_online_ = false;
  bool launcher_feedback_valid_ = false;
  Eigen::Matrix<float, 3, 1> local_chassis_gyro_{};
  bool chassis_gyro_received_ = false;
  ChassisMotionState motion_state_{};
  SentryDecisionFrame pending_decision_{
      SentryDecision::VERSION, 0U, 0U, 0U, 0U, 0U, 0U};
  SentryDecisionFrame active_decision_{};
  SentryDecision::RetryController decision_retry_{};
  SentryDecision::SequenceTracker decision_sequence_tracker_{};

  uint32_t next_control_tx_ms_ = 0;
  uint32_t next_launcher_feedback_tx_ms_ = 0;
  uint32_t next_referee_status_tx_ms_ = 0U;
  uint32_t last_rx_time_ms_ = 0;
  uint32_t last_decision_rx_time_ms_ = 0U;
  uint32_t last_decision_drop_log_ms_ = 0U;
  uint32_t reported_decision_update_drops_ = 0U;
  uint8_t remote_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
  uint8_t tx_sequence_ = 0;
  uint8_t referee_sequences_[10]{};
  uint8_t referee_status_sequence_ = 0U;
  uint8_t decision_sequence_ = 0U;
  bool online_ = false;
  bool safe_state_published_ = false;
  bool decision_drop_log_started_ = false;
};
