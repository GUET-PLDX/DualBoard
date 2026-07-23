#pragma once

#include <cstdint>

struct __attribute__((packed)) SentryDecisionFrame {
  uint8_t version;
  uint8_t sequence;
  uint8_t valid_mask;
  uint8_t state;
  uint16_t buy_bullet_delta;
  uint8_t remote_request_counts;
  uint8_t flags;
};

static_assert(sizeof(SentryDecisionFrame) == 8U);

namespace SentryDecision {

constexpr uint8_t VERSION = 1U;
constexpr uint8_t STATE_VALID = 0x01U;
constexpr uint8_t BUY_BULLET_VALID = 0x02U;
constexpr uint8_t REMOTE_BULLET_VALID = 0x04U;
constexpr uint8_t REMOTE_HP_VALID = 0x08U;
constexpr uint8_t BUY_RESURRECTION_VALID = 0x10U;
constexpr uint8_t KNOWN_VALID_MASK = 0x1fU;
constexpr uint8_t BUY_RESURRECTION_FLAG = 0x01U;
constexpr uint32_t RX_TIMEOUT_MS = 100U;
constexpr uint32_t RETRY_PERIOD_MS = 10U;
constexpr uint8_t REQUIRED_SEND_SUCCESSES = 5U;

constexpr uint8_t PackRemoteCounts(uint8_t bullet, uint8_t hp) {
  return static_cast<uint8_t>((bullet & 0x0fU) | ((hp & 0x0fU) << 4U));
}

constexpr uint8_t RemoteBulletCount(uint8_t value) { return value & 0x0fU; }

constexpr uint8_t RemoteHpCount(uint8_t value) {
  return static_cast<uint8_t>((value >> 4U) & 0x0fU);
}

inline bool Validate(const SentryDecisionFrame& frame) {
  if (frame.version != VERSION ||
      (frame.valid_mask & static_cast<uint8_t>(~KNOWN_VALID_MASK)) != 0U ||
      (frame.flags & static_cast<uint8_t>(~BUY_RESURRECTION_FLAG)) != 0U) {
    return false;
  }

  if ((frame.valid_mask & STATE_VALID) != 0U) {
    if (frame.state < 1U || frame.state > 3U) {
      return false;
    }
  } else if (frame.state != 0U) {
    return false;
  }

  if ((frame.valid_mask & BUY_BULLET_VALID) != 0U) {
    if (frame.buy_bullet_delta < 1U || frame.buy_bullet_delta > 2047U) {
      return false;
    }
  } else if (frame.buy_bullet_delta != 0U) {
    return false;
  }

  if ((frame.valid_mask & REMOTE_BULLET_VALID) == 0U &&
      RemoteBulletCount(frame.remote_request_counts) != 0U) {
    return false;
  }
  if ((frame.valid_mask & REMOTE_HP_VALID) == 0U &&
      RemoteHpCount(frame.remote_request_counts) != 0U) {
    return false;
  }
  if ((frame.valid_mask & BUY_RESURRECTION_VALID) == 0U && frame.flags != 0U) {
    return false;
  }

  return true;
}

class SequenceTracker {
 public:
  bool Accept(uint8_t sequence, uint32_t now_ms) {
    if (valid_ && sequence == last_sequence_ &&
        now_ms - last_rx_ms_ <= RX_TIMEOUT_MS) {
      ObserveDuplicate(now_ms);
      return false;
    }

    valid_ = true;
    last_sequence_ = sequence;
    last_rx_ms_ = now_ms;
    return true;
  }

  void ObserveDuplicate(uint32_t now_ms) { last_rx_ms_ = now_ms; }

 private:
  bool valid_ = false;
  uint8_t last_sequence_ = 0U;
  uint32_t last_rx_ms_ = 0U;
};

class RetryController {
 public:
  bool Begin(const SentryDecisionFrame& frame, uint8_t sequence,
             uint32_t now_ms) {
    if (active_) {
      return false;
    }

    frame_ = frame;
    frame_.sequence = sequence;
    active_ = true;
    successes_ = 0U;
    next_send_ms_ = now_ms;
    return true;
  }

  bool Due(uint32_t now_ms) const {
    return active_ && (now_ms - next_send_ms_) < 0x80000000U;
  }

  void OnSendResult(bool sent, uint32_t now_ms) {
    if (!active_) {
      return;
    }

    if (sent) {
      ++successes_;
      if (successes_ >= REQUIRED_SEND_SUCCESSES) {
        active_ = false;
        return;
      }
    }
    next_send_ms_ = now_ms + RETRY_PERIOD_MS;
  }

  bool Active() const { return active_; }
  uint8_t Successes() const { return successes_; }
  const SentryDecisionFrame& Frame() const { return frame_; }

 private:
  SentryDecisionFrame frame_{};
  bool active_ = false;
  uint8_t successes_ = 0U;
  uint32_t next_send_ms_ = 0U;
};

}  // namespace SentryDecision
