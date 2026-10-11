#include <cassert>
#include <cstdint>

#include "DualBoard.hpp"

namespace {

using SentryDecision::BUY_BULLET_VALID;
using SentryDecision::BUY_RESURRECTION_FLAG;
using SentryDecision::BUY_RESURRECTION_VALID;
using SentryDecision::REMOTE_BULLET_VALID;
using SentryDecision::REMOTE_HP_VALID;
using SentryDecision::STATE_VALID;

void ValidatesFrameContract() {
  static_assert(sizeof(SentryDecisionFrame) == 8U);
  static_assert(SentryDecision::PackRemoteCounts(15U, 15U) == 0xffU);
  static_assert(SentryDecision::RemoteBulletCount(0xa5U) == 5U);
  static_assert(SentryDecision::RemoteHpCount(0xa5U) == 10U);

  assert(SentryDecision::Validate({1U, 1U, 0x01U, 1U, 0U, 0U, 0U}));
  assert(!SentryDecision::Validate({2U, 1U, 0x01U, 1U, 0U, 0U, 0U}));
  assert(!SentryDecision::Validate({1U, 1U, 0x80U, 1U, 0U, 0U, 0U}));

  assert(SentryDecision::Validate({1U, 2U, BUY_BULLET_VALID, 0U, 1U, 0U, 0U}));
  assert(
      SentryDecision::Validate({1U, 3U, BUY_BULLET_VALID, 0U, 2047U, 0U, 0U}));
  assert(!SentryDecision::Validate({1U, 4U, BUY_BULLET_VALID, 0U, 0U, 0U, 0U}));
  assert(
      !SentryDecision::Validate({1U, 5U, BUY_BULLET_VALID, 0U, 2048U, 0U, 0U}));
  assert(!SentryDecision::Validate({1U, 6U, 0U, 0U, 1U, 0U, 0U}));

  for (uint8_t state = 1U; state <= 3U; ++state) {
    assert(
        SentryDecision::Validate({1U, state, STATE_VALID, state, 0U, 0U, 0U}));
  }
  assert(!SentryDecision::Validate({1U, 7U, STATE_VALID, 0U, 0U, 0U, 0U}));
  assert(!SentryDecision::Validate({1U, 8U, STATE_VALID, 4U, 0U, 0U, 0U}));
  assert(!SentryDecision::Validate({1U, 9U, 0U, 1U, 0U, 0U, 0U}));

  assert(SentryDecision::Validate({1U, 10U, REMOTE_BULLET_VALID, 0U, 0U,
                                   SentryDecision::PackRemoteCounts(15U, 0U),
                                   0U}));
  assert(!SentryDecision::Validate({1U, 11U, 0U, 0U, 0U, 1U, 0U}));
  assert(!SentryDecision::Validate(
      {1U, 12U, REMOTE_BULLET_VALID, 0U, 0U, 0x10U, 0U}));
  assert(
      !SentryDecision::Validate({1U, 13U, REMOTE_HP_VALID, 0U, 0U, 0x01U, 0U}));

  assert(SentryDecision::Validate(
      {1U, 14U, BUY_RESURRECTION_VALID, 0U, 0U, 0U, BUY_RESURRECTION_FLAG}));
  assert(!SentryDecision::Validate(
      {1U, 15U, BUY_RESURRECTION_VALID, 0U, 0U, 0U, 0x02U}));
  assert(!SentryDecision::Validate(
      {1U, 16U, 0U, 0U, 0U, 0U, BUY_RESURRECTION_FLAG}));
}

void TracksSequenceFreshnessAndWrap() {
  SentryDecision::SequenceTracker tracker;
  assert(tracker.Accept(255U, 100U));
  assert(tracker.Accept(0U, 101U));
  assert(!tracker.Accept(0U, 120U));
  assert(!tracker.Accept(0U, 220U));
  assert(tracker.Accept(0U, 321U));
  assert(tracker.Accept(1U, 322U));
}

void AggregatesPendingUpdatesWithoutChangingRetryFrame() {
  using SentryDecision::UpdateKind;
  SentryDecisionFrame pending{1U, 0U, 0U, 0U, 0U, 0U, 0U};
  SentryDecision::accumulate(pending, UpdateKind::BUY_BULLET, 2000U);
  SentryDecision::accumulate(pending, UpdateKind::BUY_BULLET, 100U);
  SentryDecision::accumulate(pending, UpdateKind::REMOTE_BUY_BULLET, 14U);
  SentryDecision::accumulate(pending, UpdateKind::REMOTE_BUY_BULLET, 2U);
  SentryDecision::accumulate(pending, UpdateKind::REMOTE_BUY_HP, 3U);
  SentryDecision::accumulate(pending, UpdateKind::BUY_RESURRECTION, 1U);
  SentryDecision::accumulate(pending, UpdateKind::BUY_RESURRECTION, 0U);
  SentryDecision::accumulate(pending, UpdateKind::STATE, 1U);
  SentryDecision::accumulate(pending, UpdateKind::STATE, 3U);
  assert(pending.buy_bullet_delta == 2047U);
  assert(pending.remote_request_counts == 0x3fU);
  assert(pending.state == 3U);
  assert(pending.flags == 0U);
  assert(pending.valid_mask == 0x1fU);
  assert(SentryDecision::Validate(pending));

  SentryDecision::RetryController retry;
  assert(retry.Begin(pending, 10U, 100U));
  pending = {1U, 0U, 0U, 0U, 0U, 0U, 0U};
  SentryDecision::accumulate(pending, UpdateKind::BUY_BULLET, 5U);
  SentryDecision::accumulate(pending, UpdateKind::REMOTE_BUY_HP, 20U);
  assert(pending.buy_bullet_delta == 5U);
  assert(pending.remote_request_counts == 0xf0U);
  assert(retry.Frame().buy_bullet_delta == 2047U);
  assert(retry.Frame().remote_request_counts == 0x3fU);
  assert(retry.Frame().sequence == 10U);
}

void RetriesUntilFiveSuccessfulSends() {
  SentryDecision::RetryController retry;
  SentryDecisionFrame first{1U, 0U, STATE_VALID, 1U, 0U, 0U, 0U};
  SentryDecisionFrame queued{1U, 0U, STATE_VALID, 2U, 0U, 0U, 0U};

  assert(retry.Begin(first, 254U, 100U));
  assert(retry.Active());
  assert(retry.Frame().sequence == 254U);
  assert(retry.Due(100U));
  assert(!retry.Begin(queued, 255U, 100U));
  assert(retry.Frame().state == 1U);

  first = {};
  assert(retry.Frame().state == 1U);
  assert(retry.Frame().valid_mask == STATE_VALID);

  for (uint32_t now_ms = 100U; now_ms < 140U; now_ms += 10U) {
    assert(retry.Due(now_ms));
    retry.OnSendResult(false, now_ms);
    assert(retry.Active());
    assert(retry.Successes() == 0U);
    assert(!retry.Due(now_ms + 9U));
  }

  for (uint8_t success = 1U; success <= 5U; ++success) {
    const uint32_t now_ms = 130U + static_cast<uint32_t>(success) * 10U;
    assert(retry.Due(now_ms));
    retry.OnSendResult(true, now_ms);
    assert(retry.Successes() == success);
    assert(retry.Active() == (success < 5U));
  }

  assert(!retry.Due(190U));
  assert(retry.Begin(queued, 255U, 190U));
  assert(retry.Frame().sequence == 255U);
  assert(retry.Frame().state == 2U);
  assert(retry.Successes() == 0U);
  assert(retry.Due(190U));
}

void SchedulesRetriesAcrossClockWrap() {
  SentryDecision::RetryController retry;
  const SentryDecisionFrame frame{1U, 0U, STATE_VALID, 1U, 0U, 0U, 0U};

  assert(retry.Begin(frame, 0U, UINT32_MAX - 5U));
  assert(retry.Due(UINT32_MAX - 5U));
  retry.OnSendResult(false, UINT32_MAX - 5U);
  assert(!retry.Due(UINT32_MAX));
  assert(retry.Due(4U));
}

}  // namespace

int main() {
  ValidatesFrameContract();
  TracksSequenceFreshnessAndWrap();
  AggregatesPendingUpdatesWithoutChangingRetryFrame();
  RetriesUntilFiveSuccessfulSends();
  SchedulesRetriesAcrossClockWrap();
}
