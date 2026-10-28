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
  frame[7] = 0x06U;  // first unsupported mode
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
  frame[7] = 0x86U;  // reserved high bit and unsupported mode
  assert(!Decode(frame, decoded));
}

void MotionTargetsRoundTrip() {
  for (Mode mode : {Mode::RELAX, Mode::INDEPENDENT, Mode::ROTOR, Mode::FOLLOW,
                    Mode::NAVIGATION, Mode::ROTOR_VARIABLE}) {
    Command command{};
    command.mode = mode;
    command.vx = mode == Mode::NAVIGATION ? 2.5F : 0.25F;
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
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, false, true) ==
         Mode::RELAX);
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, false, false) ==
         Mode::FOLLOW);
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, true, false) ==
         Mode::ROTOR);
  assert(SelectOutputMode(Mode::FOLLOW, false, Mode::ROTOR, true, false) ==
         Mode::FOLLOW);
  assert(SelectOutputMode(Mode::FOLLOW, true, Mode::ROTOR, false, false) ==
         Mode::FOLLOW);
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
  static_assert(Pldx::DualBoardControl::RESERVED_MASK == 0xF0U);
  static_assert(Pldx::DualBoardControl::USE_CAPACITOR_ID_OFFSET == 0x11U);
}
