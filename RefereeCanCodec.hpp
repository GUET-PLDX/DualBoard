#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "libxr_mem.hpp"

class RefereeCanCodec {
 public:
  static constexpr uint32_t GAME_STATUS_ID_OFFSET = 0x02U;
  static constexpr uint32_t FIELD_EVENT_ID_OFFSET = 0x03U;
  static constexpr uint32_t ROBOT_HP_ID_OFFSET = 0x04U;
  static constexpr uint32_t ROBOT_STATUS_ID_OFFSET = 0x07U;
  static constexpr uint32_t POWER_HEAT_ID_OFFSET = 0x0AU;
  static constexpr uint32_t ROBOT_BUFF_ID_OFFSET = 0x0CU;
  static constexpr uint32_t BULLET_REMAIN_ID_OFFSET = 0x0EU;
  static constexpr uint32_t RFID_ID_OFFSET = 0x12U;
  static constexpr uint32_t ROBOT_DAMAGE_ID_OFFSET = 0x13U;
  // Position payload uses six dedicated classic-CAN frames (40 bytes).
  static constexpr uint32_t ROBOT_POS_ID_OFFSET = 0x14U;
  static constexpr uint32_t LINK_STATUS_ID_OFFSET = 0x1EU;
  static constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 20U;
  static constexpr size_t FRAGMENT_DATA_SIZE = 7U;
  // RobotPosForSentry is ten IEEE-754 floats (40 bytes); leave room for the
  // complete payload while retaining the seven-byte data area per CAN frame.
  static constexpr size_t MAX_DATA_SIZE = 42U;

  struct __attribute__((packed)) FragmentFrame {
    uint8_t sequence = 0U;
    uint8_t data[FRAGMENT_DATA_SIZE]{};
  };

  struct Assembly {
    uint8_t sequence = 0U;
    uint8_t received_mask = 0U;
    uint8_t published_sequence = 0U;
    uint32_t last_update_ms = 0U;
    uint8_t data[MAX_DATA_SIZE]{};
    bool active = false;
    bool has_published = false;
  };

  enum class PushResult : uint8_t {
    INVALID,
    INCOMPLETE,
    COMPLETE,
    DUPLICATE,
  };

  template <typename Data>
  static auto Encode(uint8_t sequence, const Data& data) {
    static_assert(std::is_trivially_copyable_v<Data>);
    static_assert(sizeof(Data) <= MAX_DATA_SIZE);
    constexpr size_t FRAGMENT_COUNT =
        (sizeof(Data) + FRAGMENT_DATA_SIZE - 1U) / FRAGMENT_DATA_SIZE;
    std::array<FragmentFrame, FRAGMENT_COUNT> frames{};
    const auto* bytes = reinterpret_cast<const uint8_t*>(&data);
    for (size_t index = 0U; index < frames.size(); ++index) {
      frames[index].sequence = sequence;
      const size_t byte_offset = index * FRAGMENT_DATA_SIZE;
      const size_t byte_count = std::min(
          FRAGMENT_DATA_SIZE, static_cast<size_t>(sizeof(Data) - byte_offset));
      LibXR::Memory::FastCopy(frames[index].data, bytes + byte_offset,
                              byte_count);
    }
    return frames;
  }

  template <typename Data>
  static PushResult Push(Assembly& assembly, size_t fragment_index,
                         const FragmentFrame& frame, uint32_t now_ms,
                         Data& output) {
    static_assert(std::is_trivially_copyable_v<Data>);
    static_assert(sizeof(Data) <= MAX_DATA_SIZE);
    constexpr size_t FRAGMENT_COUNT =
        (sizeof(Data) + FRAGMENT_DATA_SIZE - 1U) / FRAGMENT_DATA_SIZE;
    static_assert(FRAGMENT_COUNT <= 8U);
    if (fragment_index >= FRAGMENT_COUNT) {
      return PushResult::INVALID;
    }

    Expire(assembly, now_ms);
    if (!assembly.active || assembly.sequence != frame.sequence) {
      assembly = {};
      assembly.active = true;
      assembly.sequence = frame.sequence;
    }
    if (now_ms >= assembly.last_update_ms) assembly.last_update_ms = now_ms;

    const size_t byte_offset = fragment_index * FRAGMENT_DATA_SIZE;
    const size_t byte_count = std::min(
        FRAGMENT_DATA_SIZE, static_cast<size_t>(sizeof(Data) - byte_offset));
    LibXR::Memory::FastCopy(assembly.data + byte_offset, frame.data,
                            byte_count);
    assembly.received_mask |= static_cast<uint8_t>(1U << fragment_index);
    constexpr uint8_t EXPECTED_MASK =
        static_cast<uint8_t>((1U << FRAGMENT_COUNT) - 1U);
    if (assembly.received_mask != EXPECTED_MASK) {
      return PushResult::INCOMPLETE;
    }
    if (assembly.has_published &&
        assembly.published_sequence == frame.sequence) {
      return PushResult::DUPLICATE;
    }

    LibXR::Memory::FastCopy(&output, assembly.data, sizeof(Data));
    assembly.has_published = true;
    assembly.published_sequence = frame.sequence;
    return PushResult::COMPLETE;
  }

  static bool Expire(Assembly& assembly, uint32_t now_ms) {
    if (!assembly.active || now_ms < assembly.last_update_ms ||
        now_ms - assembly.last_update_ms <= REASSEMBLY_TIMEOUT_MS) {
      return false;
    }
    assembly.active = false;
    assembly.received_mask = 0U;
    return true;
  }

  static constexpr uint16_t IntersectValidity(bool online, uint16_t local_mask,
                                              uint16_t upstream_mask,
                                              uint16_t supported_mask) {
    return online ? static_cast<uint16_t>(local_mask & upstream_mask &
                                          supported_mask)
                  : 0U;
  }
};

static_assert(sizeof(RefereeCanCodec::FragmentFrame) == 8U);
