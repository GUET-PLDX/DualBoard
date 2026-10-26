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
  CMD::NavigationFeedforward in{};
  in.fx_n = 55.555F;
  in.fy_n = -12.344F;
  in.mz_nm = 3.21F;
  in.valid = true;
  ChassisFeedforwardFrame frame{};
  assert(EncodeChassisFeedforward(in, 7U, frame));
  assert(frame.fx_q == 5556 && frame.fy_q == -1234 && frame.mz_q == 321);
  assert(frame.sequence == 7U && frame.flags == 0x01U);
  const CMD::NavigationFeedforward out = DecodeChassisFeedforward(frame);
  assert(out.valid);
  assert(std::fabs(out.fx_n - 55.56F) < 1e-4F);
  assert(std::fabs(out.fy_n + 12.34F) < 1e-4F);
  assert(std::fabs(out.mz_nm - 3.21F) < 1e-4F);
}

void InvalidInputSendsZeroAndNoValidBit() {
  CMD::NavigationFeedforward in{};
  in.fx_n = 10.0F;
  in.valid = false;  // stale host command
  ChassisFeedforwardFrame frame{};
  assert(!EncodeChassisFeedforward(in, 1U, frame));
  assert(frame.fx_q == 0 && frame.flags == 0U && frame.sequence == 1U);
  assert(!DecodeChassisFeedforward(frame).valid);
  assert(DecodeChassisFeedforward(frame).fx_n == 0.0F);

  in.valid = true;
  in.fy_n = std::numeric_limits<float>::infinity();
  assert(!EncodeChassisFeedforward(in, 2U, frame));
  in.fy_n = 0.0F;
  in.fx_n = 400.0F;  // beyond ±327.67 N: an upstream error, not clipped
  assert(!EncodeChassisFeedforward(in, 3U, frame));
  assert(frame.fx_q == 0 && frame.flags == 0U);
}

void ZeroValidCommandStaysValid() {
  CMD::NavigationFeedforward in{};
  in.valid = true;
  ChassisFeedforwardFrame frame{};
  assert(EncodeChassisFeedforward(in, 0U, frame));
  const auto out = DecodeChassisFeedforward(frame);
  assert(out.valid && out.fx_n == 0.0F && out.fy_n == 0.0F &&
         out.mz_nm == 0.0F);
}

}  // namespace

int main() {
  static_assert(sizeof(ChassisFeedforwardFrame) == 8U);
  RoundTripsAtCentinewtonResolution();
  InvalidInputSendsZeroAndNoValidBit();
  ZeroValidCommandStaysValid();
  return 0;
}
