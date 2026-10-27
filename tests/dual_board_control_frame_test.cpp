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
using Pldx::DualBoardControl::Source;

void ExpectFrame(const Command& command, const Frame& expected) {
  Frame actual{};
  assert(Encode(command, actual));
  assert(actual == expected);

  Command decoded{};
  assert(Decode(actual, decoded));
  assert(decoded.mode == command.mode);
  assert(decoded.self_define == command.self_define);
  assert(decoded.source == command.source);
}

void MatchesGoldenVectors() {
  Command operator_command{};
  operator_command.operator_input = {1.0F, -1.0F, 0.5F};
  operator_command.mode = Mode::INDEPENDENT;
  ExpectFrame(operator_command,
              {0xFFU, 0x7FU, 0x01U, 0x80U, 0xFFU, 0x3FU, 0x00U, 0x01U});

  Command navigation_command{};
  navigation_command.source = Source::NAVIGATION;
  navigation_command.mode = Mode::NAVIGATION;
  ExpectFrame(navigation_command,
              {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x84U});
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
  frame[7] = 0x86U;  // navigation source with an unsupported mode
  assert(!Decode(frame, decoded));
}

void SourceAndModeAreIndependent() {
  for (Mode mode : {Mode::RELAX, Mode::INDEPENDENT, Mode::ROTOR, Mode::FOLLOW,
                    Mode::NAVIGATION, Mode::ROTOR_VARIABLE}) {
    for (Source source : {Source::OPERATOR, Source::NAVIGATION}) {
      Command command{};
      command.mode = mode;
      command.source = source;
      command.operator_input.x = 0.25F;
      Frame frame{};
      assert(Encode(command, frame));
      Command decoded{};
      assert(Decode(frame, decoded));
      assert(decoded.source == source && decoded.mode == mode);
      if (source == Source::NAVIGATION) {
        assert(decoded.operator_input.x == 0.0F);
      }
    }
  }
}

void EncoderFailsClosed() {
  Frame frame{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
  Command command{};
  command.operator_input.x = std::numeric_limits<float>::quiet_NaN();
  command.mode = Mode::INDEPENDENT;
  assert(!Encode(command, frame));
  assert(frame == Frame{});

  command = {};
  command.operator_input = {2.0F, -2.0F, 0.0F};
  command.mode = Mode::INDEPENDENT;
  assert(Encode(command, frame));
  assert(frame[0] == 0xFFU && frame[1] == 0x7FU);
  assert(frame[2] == 0x01U && frame[3] == 0x80U);
}

void NavigationCarriesOnlyModeAndSource() {
  Command command{};
  command.source = Source::NAVIGATION;
  command.operator_input = {1.0F, -1.0F, 1.0F};
  command.mode = Mode::NAVIGATION;
  Frame frame{};
  assert(Encode(command, frame));
  Command decoded{};
  assert(Decode(frame, decoded));
  assert(decoded.operator_input.x == 0.0F);
  frame[0] = 1U;
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
  SourceAndModeAreIndependent();
  EncoderFailsClosed();
  NavigationCarriesOnlyModeAndSource();
  UseCapacitorFrameRoundTrip();
  SelectsChassisOutputMode();
  static_assert(Pldx::DualBoardControl::RESERVED_MASK == 0x70U);
  static_assert(Pldx::DualBoardControl::USE_CAPACITOR_ID_OFFSET == 0x11U);
}
