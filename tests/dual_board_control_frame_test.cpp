#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "DualBoardControlFrame.hpp"

namespace {

using Pldx::DualBoardControl::Admission;
using Pldx::DualBoardControl::AdmissionState;
using Pldx::DualBoardControl::Command;
using Pldx::DualBoardControl::Decode;
using Pldx::DualBoardControl::Encode;
using Pldx::DualBoardControl::ForceFrame;
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
  assert(decoded.si_units == command.si_units);
}

void MatchesGoldenVectors() {
  ExpectFrame({1.0F, -1.0F, 0.5F, 0, Mode::INDEPENDENT, false},
              {0xFFU, 0x7FU, 0x01U, 0x80U, 0xFFU, 0x3FU, 0x00U, 0x01U});
  ExpectFrame({2.5F, -2.5F, 1.8F, 0, Mode::INDEPENDENT, true},
              {0xC4U, 0x09U, 0x3CU, 0xF6U, 0x08U, 0x07U, 0x00U, 0x81U});
  ExpectFrame({0.0005F, -0.0005F, 0.0015F, 1, Mode::FOLLOW, true},
              {0x01U, 0x00U, 0xFFU, 0xFFU, 0x02U, 0x00U, 0x01U, 0x83U});
}

void RejectsInvalidWireValues() {
  Command decoded{};
  Frame frame{};
  frame[7] = 0x91U;
  assert(!Decode(frame, decoded));

  frame = {};
  frame[7] = 0x04U;
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
  frame[7] = 0x81U;
  assert(!Decode(frame, decoded));
}

void EncoderFailsClosed() {
  Frame frame{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
  Command command{std::numeric_limits<float>::quiet_NaN(),
                  0.0F,
                  0.0F,
                  0,
                  Mode::INDEPENDENT,
                  true};
  assert(!Encode(command, frame));
  assert(frame == Frame{});

  command = {2.5001F, 0.0F, 0.0F, 0, Mode::INDEPENDENT, true};
  assert(!Encode(command, frame));
  assert(frame == Frame{});

  command = {2.0F, -2.0F, 0.0F, 0, Mode::INDEPENDENT, false};
  assert(Encode(command, frame));
  assert(frame[0] == 0xFFU && frame[1] == 0x7FU);
  assert(frame[2] == 0x01U && frame[3] == 0x80U);
}

Frame EncodeZero(Mode mode = Mode::INDEPENDENT, bool si_units = true) {
  Frame frame{};
  assert(Encode({0.0F, 0.0F, 0.0F, 0, mode, si_units}, frame));
  return frame;
}

void RequiresContinuousZeroRearm() {
  AdmissionState state;
  const Frame ZERO = EncodeZero();
  assert(!state.Process(ZERO, 0U).accepted);
  assert(state.State() == Admission::PROBATION);
  assert(!state.Process(ZERO, 50U).accepted);
  const auto ACCEPTED = state.Process(ZERO, 100U);
  assert(ACCEPTED.accepted);
  assert(ACCEPTED.admission == Admission::ACCEPTED);

  Frame moving{};
  assert(Encode({1.0F, 0.0F, 0.0F, 0, Mode::INDEPENDENT, true}, moving));
  assert(state.Process(moving, 101U).accepted);

  Frame invalid = ZERO;
  invalid[7] = 0x91U;
  assert(!state.Process(invalid, 102U).accepted);
  assert(state.State() == Admission::INVALID);
  assert(!state.Process(moving, 103U).accepted);
  assert(state.State() == Admission::INVALID);

  assert(!state.Process(ZERO, 110U).accepted);
  assert(!state.Process(ZERO, 161U).accepted);
  assert(!state.Process(ZERO, 211U).accepted);
  assert(state.Process(ZERO, 261U).accepted);
}

void HandlesClockWrap() {
  AdmissionState state;
  const Frame ZERO = EncodeZero(Mode::RELAX);
  assert(!state.Process(ZERO, UINT32_MAX - 74U).accepted);
  assert(!state.Process(ZERO, UINT32_MAX - 24U).accepted);
  assert(state.Process(ZERO, 25U).accepted);
}

void ForceFrameRoundTrip() {
  Command command{};
  command.force_x_global_n = 12.34F;
  command.force_y_global_n = -5.67F;
  command.torque_z_global_nm = 3.21F;
  command.force_control = true;
  ForceFrame frame{};
  assert(Pldx::DualBoardControl::EncodeForce(command, frame, 7U));
  assert(frame[7] == 7U && frame[6] == 1U);
  Command decoded{};
  assert(Pldx::DualBoardControl::DecodeForce(frame, decoded));
  assert(std::fabs(decoded.force_x_global_n - 12.34F) < 0.011F);
  assert(std::fabs(decoded.force_y_global_n + 5.67F) < 0.011F);
  assert(std::fabs(decoded.torque_z_global_nm - 3.21F) < 0.011F);
  assert(decoded.force_control);
}

}  // namespace

int main() {
  MatchesGoldenVectors();
  RejectsInvalidWireValues();
  EncoderFailsClosed();
  RequiresContinuousZeroRearm();
  HandlesClockWrap();
  ForceFrameRoundTrip();
}
