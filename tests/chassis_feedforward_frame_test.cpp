#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "DualBoard.hpp"

namespace {

using Pldx::DualBoardControl::ChassisFeedforwardFrame;
using Pldx::DualBoardControl::DecodeChassisFeedforward;
using Pldx::DualBoardControl::EncodeChassisFeedforward;

void RoundTripsAtCentinewtonResolution() {
  CMD::ChassisCMD in{};
  in.fx_n = 55.555F;
  in.fy_n = -12.344F;
  in.mz_nm = 3.21F;
  in.ff_valid = true;
  ChassisFeedforwardFrame frame{};
  assert(EncodeChassisFeedforward(in, 7U, frame));
  assert(frame.fx_q == 5556 && frame.fy_q == -1234 && frame.mz_q == 321);
  assert(frame.sequence == 7U && frame.flags == 0x01U);
  const CMD::ChassisCMD out = DecodeChassisFeedforward(frame);
  assert(out.ff_valid);
  assert(std::fabs(out.fx_n - 55.56F) < 1e-4F);
  assert(std::fabs(out.fy_n + 12.34F) < 1e-4F);
  assert(std::fabs(out.mz_nm - 3.21F) < 1e-4F);
}

void InvalidInputSendsZeroAndNoValidBit() {
  CMD::ChassisCMD in{};
  in.fx_n = 10.0F;
  in.ff_valid = false;  // stale host command
  ChassisFeedforwardFrame frame{};
  assert(!EncodeChassisFeedforward(in, 1U, frame));
  assert(frame.fx_q == 0 && frame.flags == 0U && frame.sequence == 1U);
  assert(!DecodeChassisFeedforward(frame).ff_valid);
  assert(DecodeChassisFeedforward(frame).fx_n == 0.0F);

  in.ff_valid = true;
  in.fy_n = std::numeric_limits<float>::infinity();
  assert(!EncodeChassisFeedforward(in, 2U, frame));
  in.fy_n = 0.0F;
  in.fx_n = 400.0F;  // beyond ±327.67 N: an upstream error, not clipped
  assert(!EncodeChassisFeedforward(in, 3U, frame));
  assert(frame.fx_q == 0 && frame.flags == 0U);
}

void ZeroValidCommandStaysValid() {
  CMD::ChassisCMD in{};
  in.ff_valid = true;
  ChassisFeedforwardFrame frame{};
  assert(EncodeChassisFeedforward(in, 0U, frame));
  const auto out = DecodeChassisFeedforward(frame);
  assert(out.ff_valid && out.fx_n == 0.0F && out.fy_n == 0.0F &&
         out.mz_nm == 0.0F);
}

void AssemblyKeepsIndependentLifetimes() {
  Pldx::DualBoardControl::ChassisCommandAssembly state;
  Pldx::DualBoardControl::Command motion{};
  motion.vx = 1.5F;
  CMD::ChassisCMD force{};
  force.fx_n = 10;
  force.ff_valid = true;
  state.Feedforward(force, 0);
  assert(!state.MotionFresh(0));
  state.Motion(motion, 0);
  assert(state.command.vx == 1.5F && state.command.ff_valid);
  state.Motion(motion, 30);
  assert(state.command.ff_valid);
  state.Motion(motion, 31);
  assert(!state.command.ff_valid && state.command.fx_n == 0);
  state.Feedforward(force, 32);
  assert(!state.Expire(62));
  assert(state.Expire(63));
  assert(!state.command.ff_valid && state.command.vx == 1.5F);
  assert(!state.Expire(64));
  state.Feedforward(force, 1000);
  assert(
      !state.MotionFresh(1000));  // Force traffic cannot revive stale motion.
  state = {};
  state.Motion(motion, UINT32_MAX - 10);
  state.Feedforward(force, UINT32_MAX - 10);
  assert(!state.Expire(19) && state.MotionFresh(19));
  assert(state.Expire(20));
  state.Feedforward(CMD::ChassisCMD{}, 21);
  assert(!state.command.ff_valid && state.command.fx_n == 0);
}

}  // namespace

int main() {
  static_assert(sizeof(ChassisFeedforwardFrame) == 8U);
  AssemblyKeepsIndependentLifetimes();
  RoundTripsAtCentinewtonResolution();
  InvalidInputSendsZeroAndNoValidBit();
  ZeroValidCommandStaysValid();
  return 0;
}
