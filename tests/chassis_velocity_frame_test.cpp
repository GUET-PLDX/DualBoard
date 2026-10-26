#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "DualBoard.hpp"

namespace {

using Pldx::DualBoardControl::ChassisFeedback;
using Pldx::DualBoardControl::ChassisVelocity;
using Pldx::DualBoardControl::ChassisVelocityFrame;
using Pldx::DualBoardControl::DecodeChassisVelocity;
using Pldx::DualBoardControl::EncodeChassisVelocity;
using Pldx::DualBoardControl::ExtendSequence;

bool Near(float a, float b) { return std::fabs(a - b) <= 0.5e-3F; }

void RoundTripsWithMillimetreResolution() {
  ChassisVelocity in{};
  in.velocity = {1.2345F, -0.6789F, 15.9F};
  in.valid = true;
  in.wheel_online[0] = true;
  in.wheel_online[2] = true;
  in.mode = 4U;  // NAVIGATION
  in.sequence = 200U;
  ChassisVelocityFrame frame{};
  assert(EncodeChassisVelocity(in, frame));
  assert(frame.vx_q == 1235 && frame.vy_q == -679 && frame.wz_q == 15900);
  // valid | wheel0 | wheel2 | mode 4 << 5
  assert(frame.flags == (0x01U | 0x02U | 0x08U | (4U << 5U)));
  const ChassisVelocity out = DecodeChassisVelocity(frame);
  assert(out.valid && out.sequence == 200U && out.mode == 4U);
  assert(out.wheel_online[0] && !out.wheel_online[1] && out.wheel_online[2] &&
         !out.wheel_online[3]);
  assert(Near(out.velocity.vx_mps, 1.2345F));
  assert(Near(out.velocity.vy_mps, -0.6789F));
  assert(Near(out.velocity.wz_rad_s, 15.9F));
}

void InvalidOrUnencodableInputClearsValueAndValidBit() {
  ChassisVelocity in{};
  in.velocity = {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
  in.valid = true;
  in.wheel_online[3] = true;
  in.mode = 2U;
  ChassisVelocityFrame frame{};
  assert(!EncodeChassisVelocity(in, frame));
  assert(frame.vx_q == 0 && frame.vy_q == 0 && frame.wz_q == 0);
  assert(frame.flags == (0x10U | (2U << 5U)));
  ChassisVelocity out = DecodeChassisVelocity(frame);
  assert(!out.valid && out.wheel_online[3] && out.mode == 2U);
  assert(out.velocity.vx_mps == 0.0F);

  in.velocity = {40.0F, 0.0F, 0.0F};  // beyond ±32.767 m/s
  assert(!EncodeChassisVelocity(in, frame));
  in.velocity = {1.0F, 0.0F, 0.0F};
  in.valid = false;  // stale sample
  assert(!EncodeChassisVelocity(in, frame));
  assert(!DecodeChassisVelocity(frame).valid);
}

void ExtendsSequenceAcrossWrapAndLoss() {
  assert(ExtendSequence(0U, 250U, false) == 250U);
  assert(ExtendSequence(250U, 251U, true) == 251U);
  assert(ExtendSequence(255U, 0U, true) == 256U);
  assert(ExtendSequence(256U, 3U, true) == 259U);  // two frames lost
  assert(ExtendSequence(0x1FFU, 0x01U, true) == 0x201U);
  assert(ExtendSequence(259U, 3U, true) == 259U);  // duplicate
}

void FeedbackLayoutMatchesHostMirror() {
  static_assert(sizeof(ChassisVelocityFrame) == 8U);
  static_assert(sizeof(ChassisFeedback) == 26U);
  static_assert(offsetof(ChassisFeedback, vx_mps) == 0U);
  static_assert(offsetof(ChassisFeedback, gyro_z_rad_s) == 12U);
  static_assert(offsetof(ChassisFeedback, sequence) == 16U);
  static_assert(offsetof(ChassisFeedback, stamp_us) == 20U);
  static_assert(offsetof(ChassisFeedback, mode) == 24U);
  static_assert(offsetof(ChassisFeedback, flags) == 25U);
}

}  // namespace

int main() {
  RoundTripsWithMillimetreResolution();
  InvalidOrUnencodableInputClearsValueAndValidBit();
  ExtendsSequenceAcrossWrapAndLoss();
  FeedbackLayoutMatchesHostMirror();
  return 0;
}
