#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "DualBoard.hpp"

namespace {

using Codec = RefereeCanCodec;
using Bytes20 = std::array<uint8_t, 20U>;
using Bytes40 = std::array<uint8_t, 40U>;

Bytes20 MakePayload() {
  Bytes20 payload{};
  for (size_t index = 0U; index < payload.size(); ++index) {
    payload[index] = static_cast<uint8_t>(index + 1U);
  }
  return payload;
}

void ReassemblesOnlyAfterAllFragments() {
  const auto payload = MakePayload();
  const auto frames = Codec::Encode(7U, payload);
  Codec::Assembly assembly{};
  Bytes20 output{};

  assert(Codec::Push(assembly, 0U, frames[0], 100U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(Codec::Push(assembly, 2U, frames[2], 101U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(Codec::Push(assembly, 1U, frames[1], 102U, output) ==
         Codec::PushResult::COMPLETE);
  assert(output == payload);
}

void RejectsMixedSequences() {
  const auto payload = MakePayload();
  const auto first = Codec::Encode(1U, payload);
  const auto second = Codec::Encode(2U, payload);
  Codec::Assembly assembly{};
  Bytes20 output{};

  assert(Codec::Push(assembly, 0U, first[0], 10U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(Codec::Push(assembly, 1U, second[1], 11U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(Codec::Push(assembly, 2U, second[2], 12U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(Codec::Push(assembly, 0U, second[0], 13U, output) ==
         Codec::PushResult::COMPLETE);
}

void IgnoresDuplicateCompletion() {
  const auto frames = Codec::Encode(9U, MakePayload());
  Codec::Assembly assembly{};
  Bytes20 output{};
  for (size_t index = 0U; index < frames.size(); ++index) {
    Codec::Push(assembly, index, frames[index], 20U + index, output);
  }
  assert(Codec::Push(assembly, 2U, frames[2], 30U, output) ==
         Codec::PushResult::DUPLICATE);
}

void ExpiresIncompleteGroupAfterTwentyMilliseconds() {
  const auto frames = Codec::Encode(3U, MakePayload());
  Codec::Assembly assembly{};
  Bytes20 output{};
  assert(Codec::Push(assembly, 0U, frames[0], 100U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(!Codec::Expire(assembly, 120U));
  assert(Codec::Expire(assembly, 121U));
  assert(Codec::Push(assembly, 1U, frames[1], 121U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(Codec::Push(assembly, 2U, frames[2], 122U, output) ==
         Codec::PushResult::INCOMPLETE);
}

void LinkStatusCannotValidateMissingLocalGroups() {
  constexpr uint16_t local = 0x0009U;
  constexpr uint16_t upstream = 0x0019U;
  constexpr uint16_t supported = 0x03FFU;
  static_assert(Codec::IntersectValidity(true, local, upstream, supported) ==
                0x0009U);
  static_assert(Codec::IntersectValidity(false, local, upstream, supported) ==
                0U);
}

void ReassemblesFortyBytePositionPayload() {
  Bytes40 payload{};
  for (size_t index = 0U; index < payload.size(); ++index) {
    payload[index] = static_cast<uint8_t>(index + 3U);
  }
  const auto frames = Codec::Encode(4U, payload);
  assert(frames.size() == 6U);
  Codec::Assembly assembly{};
  Bytes40 output{};
  for (size_t index : {3U, 0U, 5U, 1U, 4U, 2U}) {
    assert(Codec::Push(assembly, index, frames[index], 200U + index, output) ==
           (index == 2U ? Codec::PushResult::COMPLETE
                        : Codec::PushResult::INCOMPLETE));
  }
  assert(output == payload);
  assert(Codec::Push(assembly, 2U, frames[2], 210U, output) ==
         Codec::PushResult::DUPLICATE);
}

void ReassemblesSentryInfoAndExpiresItIndependently() {
  const std::array<uint8_t, 14> referee_payload{
      0x49U, 0x00U, 0x00U, 0x00U, 0x57U, 0xF3U, 1U, 2U, 3U, 0U, 4U, 5U, 6U, 0U};
  Referee::SentryInfo input{};
  static_assert(sizeof(input) == referee_payload.size());
  LibXR::Memory::FastCopy(&input, referee_payload.data(),
                          referee_payload.size());
  const auto frames = Codec::Encode(6U, input);
  static_assert(frames.size() == 2U);
  Codec::Assembly assembly{};
  Referee::SentryInfo output{0U, 0x2000U, 0U};
  assert(Codec::Push(assembly, 0U, frames[0], 100U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(output.info_2 == 0x2000U);
  auto unmatched = frames[1];
  unmatched.sequence = 7U;
  assert(Codec::Push(assembly, 1U, unmatched, 101U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(output.info_2 == 0x2000U);
  assert(Codec::Push(assembly, 1U, frames[1], 102U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(Codec::Push(assembly, 0U, frames[0], 103U, output) ==
         Codec::PushResult::COMPLETE);
  assert(output.info_1 == 73U && output.info_2 == 0xF357U &&
         output.info_3 == 0x0006050400030201ULL &&
         output.CurrentStance() == 3U);
  Referee::SourceTimes received{};
  received[11] = 100U;
  received[3] = 3000U;
  constexpr uint16_t valid = Referee::SOURCE_SENTRY_INFO | Referee::SOURCE_ROBOT_STATUS;
  assert(Referee::FreshSourceMask(valid, received, 3100U) == valid);
  assert(Referee::FreshSourceMask(valid, received, 3101U) == Referee::SOURCE_ROBOT_STATUS);
  assert(Referee::FreshSourceMask(0U, received, 3100U) == 0U);
}

void EncodesBasicStanceWithoutOverwritingNeighboringCommandFields() {
  constexpr std::array<uint8_t, 3> expected_stance_byte{0x35U, 0x55U, 0x75U};
  for (uint32_t stance = 1U; stance <= 3U; ++stance) {
    Referee::SentryDecisionData request{};
    request.buy_bullet_num = 0x123U;
    request.confirm_resurrection = 1U;
    request.remote_buy_bullet_times = 8U;
    request.romote_buy_hp_times = 10U;
    request.comfirm_mech = 1U;
    request.current_state = stance;
    std::array<uint8_t, 4> encoded{};
    static_assert(sizeof(request) == encoded.size());
    LibXR::Memory::FastCopy(encoded.data(), &request, encoded.size());
    assert((encoded == std::array<uint8_t, 4>{0x8DU, 0x04U,
                                              expected_stance_byte[stance - 1U],
                                              0x01U}));
  }
}

}  // namespace

int main() {
  static_assert(Codec::GAME_STATUS_ID_OFFSET == 0x02U);
  static_assert(Codec::ROBOT_HP_ID_OFFSET == 0x04U);
  static_assert(Codec::ROBOT_POS_ID_OFFSET == 0x14U);
  static_assert(Codec::SENTRY_INFO_ID_OFFSET == 0x21U);
  static_assert(Codec::LINK_STATUS_ID_OFFSET == 0x1DU);
  ReassemblesOnlyAfterAllFragments();
  RejectsMixedSequences();
  IgnoresDuplicateCompletion();
  ExpiresIncompleteGroupAfterTwentyMilliseconds();
  LinkStatusCannotValidateMissingLocalGroups();
  ReassemblesFortyBytePositionPayload();
  ReassemblesSentryInfoAndExpiresItIndependently();
  EncodesBasicStanceWithoutOverwritingNeighboringCommandFields();
}
