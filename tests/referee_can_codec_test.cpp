#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "RefereeCanCodec.hpp"

namespace {

using Codec = RefereeCanCodec;
using Bytes20 = std::array<uint8_t, 20U>;

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

}  // namespace

int main() {
  static_assert(Codec::GAME_STATUS_ID_OFFSET == 0x02U);
  static_assert(Codec::ROBOT_HP_ID_OFFSET == 0x04U);
  static_assert(Codec::LINK_STATUS_ID_OFFSET == 0x16U);
  ReassemblesOnlyAfterAllFragments();
  RejectsMixedSequences();
  IgnoresDuplicateCompletion();
  ExpiresIncompleteGroupAfterTwentyMilliseconds();
  LinkStatusCannotValidateMissingLocalGroups();
}
