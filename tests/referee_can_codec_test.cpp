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
  for (size_t index = 0U; index < frames.size(); ++index)
    assert(Codec::Push(assembly, index, frames[index], 100U + index, output) ==
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

void ExpiresIncompleteGroupAcrossClockWrap() {
  const auto frames = Codec::Encode(5U, MakePayload());
  Codec::Assembly assembly{};
  Bytes20 output{};
  assert(Codec::Push(assembly, 0U, frames[0], 0xfffffff8U, output) ==
         Codec::PushResult::INCOMPLETE);
  assert(!Codec::Expire(assembly, 12U));
  assert(Codec::Expire(assembly, 13U));
}

void RejectsConflictingDuplicateFragment() {
  const auto frames = Codec::Encode(8U, MakePayload());
  Codec::Assembly assembly{};
  Bytes20 output{};
  assert(Codec::Push(assembly, 0U, frames[0], 100U, output) ==
         Codec::PushResult::INCOMPLETE);
  auto changed = frames[0];
  changed.data[3] ^= 0x80U;
  assert(Codec::Push(assembly, 0U, changed, 101U, output) ==
         Codec::PushResult::INVALID);
  assert(Codec::Push(assembly, 1U, frames[1], 102U, output) ==
         Codec::PushResult::DUPLICATE);
  assert(Codec::Push(assembly, 2U, frames[2], 103U, output) ==
         Codec::PushResult::DUPLICATE);
  assert(Codec::Push(assembly, 0U, changed, 104U, output) ==
         Codec::PushResult::DUPLICATE);
  assert(output == Bytes20{});
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
  uint32_t received_ms = 200U;
  for (size_t index : {3U, 0U, 5U, 1U, 4U, 2U}) {
    assert(Codec::Push(assembly, index, frames[index], received_ms++, output) ==
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
  constexpr uint16_t valid =
      Referee::SOURCE_SENTRY_INFO | Referee::SOURCE_ROBOT_STATUS;
  assert(Referee::FreshSourceMask(valid, received, 3100U) == valid);
  assert(Referee::FreshSourceMask(valid, received, 3101U) ==
         Referee::SOURCE_ROBOT_STATUS);
  assert(Referee::FreshSourceMask(0U, received, 3100U) == 0U);
}

void Preserves2027ExpandedPayloads() {
  const std::array<uint8_t, 24U> hp_bytes{
      0x01U, 0x02U, 0x99U, 0x88U, 0x03U, 0x04U, 0x05U, 0x06U,
      0x07U, 0x08U, 0x09U, 0x0aU, 0x34U, 0x12U, 0x0bU, 0x0cU,
      0x78U, 0x56U, 0x0dU, 0x0eU, 0xbcU, 0x9aU, 0xf0U, 0xdeU};
  Referee::RobotHP hp{};
  LibXR::Memory::FastCopy(&hp, hp_bytes.data(), hp_bytes.size());
  const auto hp_frames = Codec::Encode(3U, hp);
  static_assert(hp_frames.size() == 4U);
  Codec::Assembly hp_assembly{};
  Referee::RobotHP hp_result{};
  uint32_t now = 10U;
  for (size_t i : {3U, 0U, 2U, 1U})
    Codec::Push(hp_assembly, i, hp_frames[i], now++, hp_result);
  assert(hp_result.r_outpost_hp == 0x1234U &&
         hp_result.b_outpost_hp == 0x5678U);
  assert(hp_result.ally_control_degree == 0x9abcU &&
         hp_result.enemy_control_degree == 0xdef0U);
  assert(std::memcmp(&hp_result, hp_bytes.data(), hp_bytes.size()) == 0);

  const std::array<uint8_t, 8U> event_bytes{0x12U, 0x34U, 0x56U, 0x78U,
                                            0x9aU, 0xbcU, 0xdeU, 0xf0U};
  Referee::FieldEvents event{};
  LibXR::Memory::FastCopy(&event, event_bytes.data(), event_bytes.size());
  const auto events = Codec::Encode(4U, event);
  static_assert(events.size() == 2U);
  Codec::Assembly event_assembly{};
  Referee::FieldEvents event_result{};
  Codec::Push(event_assembly, 1U, events[1], 20U, event_result);
  assert(Codec::Push(event_assembly, 0U, events[0], 21U, event_result) ==
         Codec::PushResult::COMPLETE);
  assert(event_result.event_data == 0x78563412U &&
         event_result.event_data_2 == 0xf0debc9aU);

  for (uint16_t sequence : {0x1234U, 0x1235U}) {
    Referee::RobotDamage damage{};
    const std::array<uint8_t, 7U> damage_bytes{
        0x21U, static_cast<uint8_t>(sequence), 0x12U, 0x00U, 0x80U, 0xffU,
        0x7fU};
    LibXR::Memory::FastCopy(&damage, damage_bytes.data(), damage_bytes.size());
    const auto frames = Codec::Encode(static_cast<uint8_t>(sequence), damage);
    static_assert(frames.size() == 1U);
    Codec::Assembly assembly{};
    Referee::RobotDamage result{};
    assert(Codec::Push(assembly, 0U, frames[0], 30U, result) ==
           Codec::PushResult::COMPLETE);
    assert(result.hit_sequence == sequence && result.hit_x == -32768 &&
           result.hit_y == 32767);
  }
  Referee::RFID rfid{0x12345678U, 0x0100U};
  const auto rfid_frames = Codec::Encode(5U, rfid);
  static_assert(rfid_frames.size() == 1U);
  Codec::Assembly rfid_assembly{};
  Referee::RFID rfid_result{};
  assert(Codec::Push(rfid_assembly, 0U, rfid_frames[0], 40U, rfid_result) ==
         Codec::PushResult::COMPLETE);
  assert(rfid_result.rfid_status == 0x12345678U &&
         rfid_result.rfid_status_2 == 0x0100U);
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
  static_assert(Codec::FIELD_EVENT_ID_OFFSET == 0x23U);
  static_assert(Codec::RFID_ID_OFFSET == 0x03U);
  static_assert(0x311U + Codec::RFID_ID_OFFSET == 0x314U);
  static_assert(0x312U + Pldx::DualBoardControl::USE_CAPACITOR_ID_OFFSET ==
                0x323U);
  static_assert(Codec::HasUniqueCanIds());
  static_assert(!Codec::HasUniqueCanIds(0x311U, 0x311U));
  static_assert(!Codec::HasUniqueCanIds(0x7f0U, 0x7f1U));
  static_assert(Codec::LAYOUT_VERSION == 2U);
  static_assert(Codec::ROBOT_HP_ID_OFFSET == 0x25U);
  static_assert(Codec::ROBOT_POS_ID_OFFSET == 0x14U);
  static_assert(Codec::SENTRY_INFO_ID_OFFSET == 0x21U);
  static_assert(Codec::LINK_STATUS_ID_OFFSET == 0x1DU);
  ReassemblesOnlyAfterAllFragments();
  RejectsMixedSequences();
  IgnoresDuplicateCompletion();
  ExpiresIncompleteGroupAfterTwentyMilliseconds();
  ExpiresIncompleteGroupAcrossClockWrap();
  RejectsConflictingDuplicateFragment();
  LinkStatusCannotValidateMissingLocalGroups();
  ReassemblesFortyBytePositionPayload();
  ReassemblesSentryInfoAndExpiresItIndependently();
  Preserves2027ExpandedPayloads();
  EncodesBasicStanceWithoutOverwritingNeighboringCommandFields();
}
