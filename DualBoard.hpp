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
template_args:
  - ROLE: DualBoardRole::GIMBAL
  - ChassisType: Omni
required_hardware:
  - can

depends:
  - qdu-future/CMD
  - qdu-future/Chassis
  - qdu-future/Referee
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
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

  DualBoard(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
            const char* can_bus_name, uint32_t tx_id, uint32_t rx_id,
            uint32_t rx_buffer_size, uint32_t tx_slot_count,
            uint32_t offline_timeout_ms, Chassis<ChassisType>* chassis,
            const char* mode_topic_name = "dualboard_chassis_mode",
            CMD* cmd = nullptr)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})),
        tx_id_(tx_id),
        rx_id_(rx_id),
        rx_buffer_size_(rx_buffer_size),
        tx_slot_count_(tx_slot_count),
        offline_timeout_ms_(offline_timeout_ms),
        chassis_(chassis),
        cmd_(cmd),
        mode_topic_name_(mode_topic_name),
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

  static constexpr uint32_t CONTROL_PERIOD_MS = 10;
  static constexpr uint32_t LAUNCHER_FEEDBACK_PERIOD_MS = 20;
  static constexpr uint32_t PROTOCOL_THREAD_PERIOD_MS = 2;
  static constexpr uint32_t CONTROL_ID_OFFSET = 0x00U;
  static constexpr uint32_t ANGLE_ID_OFFSET = 0x10U;
  static constexpr uint32_t ATTITUDE_ID_OFFSET = 0x20U;
  static constexpr uint32_t RX_ID_RANGE = ATTITUDE_ID_OFFSET;
  static constexpr float COMMAND_SCALE = 32767.0f;
  static constexpr float COMMAND_LIMIT = 1.0f;
  static constexpr float ANGLE_SCALE = 10000.0f;
  static constexpr float ANGLE_LIMIT = 3.2f;
  static constexpr float BULLET_SPEED_SCALE = 10.0f;
  static constexpr float BULLET_SPEED_LIMIT = 25.5f;

  static_assert(sizeof(ControlFrame) == 8,
                "ControlFrame must be one classic CAN frame");
  static_assert(sizeof(AngleFrame) == 8,
                "AngleFrame must be one classic CAN frame");
  static_assert(sizeof(AttitudeFrame) == 8,
                "AttitudeFrame must be one classic CAN frame");
  static_assert(sizeof(LauncherFeedbackFrame) == 8,
                "LauncherFeedbackFrame must be one classic CAN frame");

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

  template <typename Data>
  LibXR::Topic CreateTopic(const char* name) {
    return LibXR::Topic::CreateTopic<Data>(name, nullptr, true);
  }

  void RegisterRoleTopics() {
    mode_topic_ = CreateTopic<uint32_t>(mode_topic_name_);

    if constexpr (ROLE == DualBoardRole::GIMBAL) {
      RegisterTopicCallback<CMD::ChassisCMD, &DualBoard::OnLocalChassisCommand>(
          "chassis_cmd");
      RegisterTopicCallback<float, &DualBoard::OnLocalYawAngle>(
          "yawmotor_angle");
      RegisterTopicCallback<float, &DualBoard::OnLocalPitchAngle>(
          "pitchmotor_angle");
      RegisterTopicCallback<LibXR::EulerAngle<float>,
                            &DualBoard::OnLocalAttitude>("gimbal_euler");

      launcher_ref_topic_ = CreateTopic<Referee::LauncherPack>("launcher_ref");
      sentry_ref_topic_ =
          CreateTopic<Referee::RobotGameRefereePack>("sentry_ref");
      sentry_state_topic_ = CreateTopic<uint8_t>("sentry_state");
    } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
      chassis_cmd_topic_ = CreateTopic<CMD::ChassisCMD>("chassis_cmd");
      yaw_angle_topic_ = CreateTopic<float>("yawmotor_angle");
      pitch_angle_topic_ = CreateTopic<float>("pitchmotor_angle");
      attitude_topic_ = CreateTopic<LibXR::EulerAngle<float>>("gimbal_euler");

      RegisterTopicCallback<Referee::LauncherPack,
                            &DualBoard::OnLocalLauncherFeedback>(
          "launcher_ref");
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
      cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL,
                                start_ctrl_callback);
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
        SendGimbalControlFrames(now_ms);
        CheckOffline(now_ms);
      } else if constexpr (ROLE == DualBoardRole::CHASSIS) {
        SendLauncherFeedbackFrameIfDue(now_ms);
        CheckOffline(now_ms);
      }

      protocol_thread_.SleepUntil(last_time, PROTOCOL_THREAD_PERIOD_MS);
    }
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
      } else if (offset == ATTITUDE_ID_OFFSET) {
        HandleAttitudeFrame(pack);
      }
    } else if constexpr (ROLE == DualBoardRole::GIMBAL) {
      if (offset == CONTROL_ID_OFFSET) {
        HandleLauncherFeedbackFrame(pack);
      }
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
      Referee::LauncherPack launcher_pack{};
      launcher_ref_topic_.Publish(launcher_pack);

      LibXR::Mutex::LockGuard lock(data_mutex_);
      launcher_feedback_valid_ = false;
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

  LibXR::Topic mode_topic_;
  LibXR::Topic chassis_cmd_topic_;
  LibXR::Topic yaw_angle_topic_;
  LibXR::Topic pitch_angle_topic_;
  LibXR::Topic attitude_topic_;
  LibXR::Topic launcher_ref_topic_;
  LibXR::Topic sentry_ref_topic_;
  LibXR::Topic sentry_state_topic_;
  LibXR::Event dual_board_event_;
  LibXR::CAN::Callback can_rx_callback_;

  LibXR::MPMCQueue<LibXR::CAN::ClassicPack> rx_frames_;
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
  bool launcher_feedback_valid_ = false;

  uint32_t next_control_tx_ms_ = 0;
  uint32_t next_launcher_feedback_tx_ms_ = 0;
  uint32_t last_rx_time_ms_ = 0;
  uint8_t remote_mode_ = static_cast<uint8_t>(ChassisMode::RELAX);
  uint8_t tx_sequence_ = 0;
  bool online_ = false;
  bool safe_state_published_ = false;
};
