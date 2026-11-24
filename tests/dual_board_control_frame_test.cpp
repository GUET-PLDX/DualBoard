#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <limits>

#include "DualBoard.hpp"

static_assert(
    sizeof(DualBoard<DualBoardRole::CHASSIS>) <
        sizeof(DualBoard<DualBoardRole::GIMBAL>),
    "Chassis must not carry gimbal receive buffers and decision queues");

namespace {

using Pldx::DualBoardControl::Command;
using Pldx::DualBoardControl::Decode;
using Pldx::DualBoardControl::Encode;
using Pldx::DualBoardControl::Frame;
using Pldx::DualBoardControl::Mode;

void ExpectFrame(const Command& command, const Frame& expected) {
  Frame actual{};
  assert(Encode(command, actual));
  assert(actual == expected);

  Command decoded{};
  assert(Decode(actual, decoded));
  assert(decoded.mode == command.mode);
  assert(decoded.self_define == command.self_define);
}

void MatchesGoldenVectors() {
  Command operator_command{};
  operator_command.vx = 1.0F;
  operator_command.vy = -1.0F;
  operator_command.wz = 0.5F;
  operator_command.mode = Mode::INDEPENDENT;
  ExpectFrame(operator_command,
              {0xFFU, 0x7FU, 0x01U, 0x80U, 0xFFU, 0x3FU, 0x00U, 0x01U});

  Command navigation_command{};
  navigation_command.mode = Mode::NAVIGATION;
  ExpectFrame(navigation_command,
              {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x04U});
}

void RejectsInvalidWireValues() {
  Command decoded{};
  Frame frame{};
  frame[7] = 0x91U;
  assert(!Decode(frame, decoded));

  frame = {};
  frame[7] = 0x08U;
  assert(!Decode(frame, decoded));

  frame = {};
  frame[6] = 3U;
  assert(!Decode(frame, decoded));

  frame = {};
  frame[0] = 0x00U;
  frame[1] = 0x80U;
  assert(!Decode(frame, decoded));

  frame = {};
  frame[0] = 0xC5U;
  frame[1] = 0x09U;
  frame[7] = 0x87U;  // reserved high bit
  assert(!Decode(frame, decoded));
}

void MotionTargetsRoundTrip() {
  for (Mode mode :
       {Mode::RELAX, Mode::INDEPENDENT, Mode::ROTOR, Mode::FOLLOW,
        Mode::NAVIGATION, Mode::ROTOR_VARIABLE, Mode::NAVIGATION_ROTOR,
        Mode::HOLE}) {
    Command command{};
    command.mode = mode;
    command.vx = (mode == Mode::NAVIGATION || mode == Mode::NAVIGATION_ROTOR ||
                  mode == Mode::HOLE)
                     ? 2.5F
                     : 0.25F;
    command.vy = -command.vx;
    command.wz = 0.5F;
    Frame frame{};
    assert(Encode(command, frame));
    Command decoded{};
    assert(Decode(frame, decoded));
    assert(decoded.mode == mode);
    assert(std::abs(decoded.vx - command.vx) < 0.001F);
    assert(std::abs(decoded.vy - command.vy) < 0.001F);
    assert(std::abs(decoded.wz - command.wz) < 0.001F);
  }
}

void EncoderFailsClosed() {
  Frame frame{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
  Command command{};
  command.vx = std::numeric_limits<float>::quiet_NaN();
  command.mode = Mode::INDEPENDENT;
  assert(!Encode(command, frame));
  assert(frame == Frame{});

  command = {};
  command.vx = 2.0F;
  command.vy = -2.0F;
  command.mode = Mode::INDEPENDENT;
  assert(Encode(command, frame));
  assert(frame[0] == 0xFFU && frame[1] == 0x7FU);
  assert(frame[2] == 0x01U && frame[3] == 0x80U);
}

void NavigationVelocityGoldenVector() {
  Command command{};
  command.vx = 1.5F;
  command.vy = -2.5F;
  command.wz = 0.25F;
  command.mode = Mode::NAVIGATION;
  ExpectFrame(command, {0xDC, 0x05, 0x3C, 0xF6, 0xFA, 0x00, 0x00, 0x04});
  command.mode = Mode::NAVIGATION_ROTOR;
  ExpectFrame(command, {0xDC, 0x05, 0x3C, 0xF6, 0xFA, 0x00, 0x00, 0x06});
  command.mode = Mode::HOLE;
  ExpectFrame(command, {0xDC, 0x05, 0x3C, 0xF6, 0xFA, 0x00, 0x00, 0x07});
  Frame frame{};
  assert(Encode(command, frame));
  frame[7] |= 0x80U;  // Reserved high bits fail closed.
  Command decoded{};
  assert(!Decode(frame, decoded));
}

void UseCapacitorFrameRoundTrip() {
  Pldx::DualBoardControl::UseCapacitorCommand frame{};
  assert(Pldx::DualBoardControl::EncodeUseCapacitor(true, 9U, frame));
  assert(frame.enabled == 1U && frame.sequence == 9U);
  bool enabled = false;
  assert(Pldx::DualBoardControl::DecodeUseCapacitor(frame, enabled));
  assert(enabled);

  assert(Pldx::DualBoardControl::EncodeUseCapacitor(false, 0U, frame));
  assert(Pldx::DualBoardControl::DecodeUseCapacitor(frame, enabled));
  assert(!enabled);

  frame.enabled = 2U;
  assert(!Pldx::DualBoardControl::DecodeUseCapacitor(frame, enabled));
  frame.enabled = 1U;
  frame.reserved[0] = 1U;
  assert(!Pldx::DualBoardControl::DecodeUseCapacitor(frame, enabled));
}

void SelectsChassisOutputMode() {
  using Pldx::DualBoardControl::SelectOutputMode;
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, false, true, true) ==
         Mode::RELAX);
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, false, true, false) ==
         Mode::FOLLOW);
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, true, true, false) ==
         Mode::ROTOR);
  assert(SelectOutputMode(Mode::FOLLOW, false, Mode::ROTOR, true, true, false) ==
         Mode::FOLLOW);
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, true, false, false) ==
         Mode::FOLLOW);

  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::HOLE, true, true, false) ==
         Mode::RELAX);
  assert(SelectOutputMode(Mode::HOLE, true, Mode::NAVIGATION, false, true, false) ==
         Mode::HOLE);
  assert(SelectOutputMode(Mode::HOLE, true, Mode::NAVIGATION, false, true, true) ==
         Mode::RELAX);
  assert(SelectOutputMode(Mode::HOLE, true, Mode::NAVIGATION, true, true, false) ==
         Mode::NAVIGATION);
  assert(SelectOutputMode(Mode::HOLE, true, Mode::NAVIGATION, true, false, false) ==
         Mode::RELAX);
}

void MapsManualHoleVelocityInFoldedFrontFrame() {
  using Pldx::DualBoardControl::MapManualHoleVelocity;
  constexpr float FULL_SPEED_MPS = 3.9F;
  const float COMPONENT_MPS = FULL_SPEED_MPS / std::sqrt(2.0F);
  for (float direction : {-1.0F, 1.0F}) {
    CMD::ChassisCMD forward{};
    forward.vy = direction;
    forward.wz = 1.0F;
    MapManualHoleVelocity(forward, FULL_SPEED_MPS);
    assert(std::abs(forward.vx - direction * COMPONENT_MPS) < 0.001F);
    assert(std::abs(forward.vy - direction * COMPONENT_MPS) < 0.001F);
    assert(forward.wz == 0.0F);

    CMD::ChassisCMD strafe{};
    strafe.vx = direction;
    MapManualHoleVelocity(strafe, FULL_SPEED_MPS);
    assert(std::abs(strafe.vx - direction * COMPONENT_MPS) < 0.001F);
    assert(std::abs(strafe.vy + direction * COMPONENT_MPS) < 0.001F);
  }
}

void DrivesManualConfigurationInMeasuredOrder() {
  using Pldx::FoldAction;
  using Pldx::LifterPosition;
  using Pldx::DualBoardControl::ConfigurationControl;
  using Pldx::DualBoardControl::ConfigurationInput;
  ConfigurationControl controller;
  ConfigurationInput input{.request = Mode::HOLE,
                           .actual_mode = Mode::ROTOR,
                           .position = LifterPosition::TOP,
                           .feedback_valid = true};
  const auto expect = [&](uint32_t time_ms, Mode mode, FoldAction fold) {
    input.yaw_sample_us = 1U + time_ms * 1000U;
    const auto output = controller.Step(input);
    assert(output.mode == mode);
    assert(output.fold == fold);
    CMD::ChassisCMD stick{};
    stick.vx = 0.2F;
    stick.vy = 0.6F;
    const auto wire =
        Pldx::DualBoardControl::MakeControlCommand(stick, output.mode, 3.9F);
    Frame frame{};
    Command decoded{};
    assert(Encode(wire, frame) && Decode(frame, decoded));
    assert(decoded.mode == mode);
    const float expected_vx = mode == Mode::HOLE ? 2.206F : 0.2F;
    const float expected_vy = mode == Mode::HOLE ? 1.103F : 0.6F;
    assert(std::abs(decoded.vx - expected_vx) < 0.001F);
    assert(std::abs(decoded.vy - expected_vy) < 0.001F);
    return output.folded;
  };

  expect(0, Mode::HOLE, FoldAction::HOLD);
  input.yaw_rad = 0.78539816F;
  expect(10, Mode::HOLE, FoldAction::HOLD);
  input.actual_mode = Mode::HOLE;
  expect(20, Mode::HOLE, FoldAction::HOLD);
  expect(69, Mode::HOLE, FoldAction::HOLD);
  assert(!expect(70, Mode::HOLE, FoldAction::BOTTOM));
  input.position = LifterPosition::BOTTOM;
  assert(expect(80, Mode::HOLE, FoldAction::BOTTOM));

  input.request = Mode::ROTOR;
  expect(90, Mode::HOLE, FoldAction::TOP);
  input.position = LifterPosition::TOP;
  expect(100, Mode::FOLLOW, FoldAction::TOP);
  input.yaw_rad = 0.0F;
  expect(110, Mode::FOLLOW, FoldAction::TOP);
  input.actual_mode = Mode::FOLLOW;
  expect(120, Mode::FOLLOW, FoldAction::TOP);
  expect(170, Mode::ROTOR, FoldAction::TOP);
  input.actual_mode = Mode::ROTOR;
  input.yaw_rad = 1.2F;
  expect(180, Mode::ROTOR, FoldAction::TOP);
}

void FollowsLatestRequestDuringConfigurationChange() {
  using Pldx::FoldAction;
  using Pldx::LifterPosition;
  using Pldx::DualBoardControl::ConfigurationControl;
  using Pldx::DualBoardControl::ConfigurationInput;
  ConfigurationControl controller;
  ConfigurationInput input{.request = Mode::HOLE,
                           .actual_mode = Mode::FOLLOW,
                           .position = LifterPosition::TOP,
                           .feedback_valid = true};
  const auto expect = [&](uint32_t time_ms, Mode mode, FoldAction fold) {
    input.yaw_sample_us = 1U + time_ms * 1000U;
    const auto output = controller.Step(input);
    assert(output.mode == mode && output.fold == fold);
  };

  expect(0, Mode::HOLE, FoldAction::HOLD);
  input.request = Mode::ROTOR;
  expect(10, Mode::FOLLOW, FoldAction::TOP);
  input.request = Mode::HOLE;
  input.actual_mode = Mode::HOLE;
  input.yaw_rad = 0.78539816F;
  expect(20, Mode::HOLE, FoldAction::HOLD);
  expect(70, Mode::HOLE, FoldAction::BOTTOM);
  input.position = LifterPosition::MIDDLE;
  input.request = Mode::ROTOR;
  expect(80, Mode::HOLE, FoldAction::TOP);
  input.yaw_rad = 0.70F;
  expect(90, Mode::HOLE, FoldAction::HOLD);
  input.request = Mode::HOLE;
  input.yaw_rad = 0.78539816F;
  expect(100, Mode::HOLE, FoldAction::HOLD);
  expect(150, Mode::HOLE, FoldAction::BOTTOM);

  input.request = Mode::ROTOR;
  expect(160, Mode::HOLE, FoldAction::TOP);
  input.position = LifterPosition::TOP;
  expect(170, Mode::FOLLOW, FoldAction::TOP);
  input.actual_mode = Mode::FOLLOW;
  input.yaw_rad = 0.3F;
  input.request = Mode::HOLE;
  expect(180, Mode::HOLE, FoldAction::HOLD);
  input.yaw_rad = 0.78539816F;
  expect(240, Mode::HOLE, FoldAction::HOLD);
  input.actual_mode = Mode::HOLE;
  expect(250, Mode::HOLE, FoldAction::HOLD);
  expect(300, Mode::HOLE, FoldAction::BOTTOM);
}

void StopsAndRestartsFromMeasuredConfiguration() {
  using Pldx::FoldAction;
  using Pldx::LifterPosition;
  using Pldx::DualBoardControl::ConfigurationControl;
  using Pldx::DualBoardControl::ConfigurationInput;
  ConfigurationControl controller;
  ConfigurationInput input{.request = Mode::ROTOR};
  const auto expect = [&](uint32_t time_ms, Mode mode, FoldAction fold) {
    input.yaw_sample_us = 1U + time_ms * 1000U;
    const auto output = controller.Step(input);
    assert(output.mode == mode && output.fold == fold);
  };

  expect(0, Mode::RELAX, FoldAction::HOLD);
  input.feedback_valid = true;
  input.position = LifterPosition::BOTTOM;
  expect(10, Mode::HOLE, FoldAction::HOLD);
  input.actual_mode = Mode::HOLE;
  input.yaw_rad = 0.78539816F;
  expect(20, Mode::HOLE, FoldAction::HOLD);
  expect(70, Mode::HOLE, FoldAction::TOP);
  input.request = Mode::RELAX;
  expect(71, Mode::RELAX, FoldAction::HOLD);
  input.request = Mode::HOLE;
  input.position = LifterPosition::MIDDLE;
  expect(80, Mode::HOLE, FoldAction::HOLD);
  input.feedback_valid = false;
  expect(100, Mode::RELAX, FoldAction::HOLD);
  input.feedback_valid = true;
  expect(140, Mode::HOLE, FoldAction::HOLD);
  expect(190, Mode::HOLE, FoldAction::BOTTOM);

  input.request = Mode::ROTOR;
  input.position = LifterPosition::TOP;
  input.actual_mode = Mode::FOLLOW;
  input.yaw_rad = 0.0F;
  expect(200, Mode::FOLLOW, FoldAction::TOP);
  expect(250, Mode::ROTOR, FoldAction::TOP);
  input.yaw_rad = std::numeric_limits<float>::quiet_NaN();
  expect(260, Mode::RELAX, FoldAction::HOLD);
  input.yaw_rad = 0.0F;
  expect(270, Mode::FOLLOW, FoldAction::TOP);
  expect(320, Mode::ROTOR, FoldAction::TOP);
}

void RepeatedYawSampleDoesNotCountAsSettled() {
  using Pldx::FoldAction;
  using Pldx::LifterPosition;
  using Pldx::DualBoardControl::ConfigurationControl;
  using Pldx::DualBoardControl::ConfigurationInput;
  ConfigurationControl controller;
  ConfigurationInput input{.request = Mode::HOLE,
                           .actual_mode = Mode::HOLE,
                           .position = LifterPosition::TOP,
                           .yaw_rad = 0.78539816F,
                           .feedback_valid = true,
                           .yaw_sample_us = 1000U};
  assert(controller.Step(input).fold == FoldAction::HOLD);
  assert(controller.Step(input).fold == FoldAction::HOLD);
  input.yaw_sample_us = 101000U;
  assert(controller.Step(input).fold == FoldAction::HOLD);
  input.yaw_sample_us = 151000U;
  assert(controller.Step(input).fold == FoldAction::BOTTOM);
}

}  // namespace

int main() {
  MatchesGoldenVectors();
  RejectsInvalidWireValues();
  MotionTargetsRoundTrip();
  EncoderFailsClosed();
  NavigationVelocityGoldenVector();
  UseCapacitorFrameRoundTrip();
  SelectsChassisOutputMode();
  MapsManualHoleVelocityInFoldedFrontFrame();
  DrivesManualConfigurationInMeasuredOrder();
  FollowsLatestRequestDuringConfigurationChange();
  StopsAndRestartsFromMeasuredConfiguration();
  RepeatedYawSampleDoesNotCountAsSettled();
  static_assert(Pldx::DualBoardControl::RESERVED_MASK == 0xF0U);
  static_assert(Pldx::DualBoardControl::USE_CAPACITOR_ID_OFFSET == 0x11U);
}
