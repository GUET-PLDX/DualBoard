#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-DualBoard.hpp}"
CONTRACT="${SENTRY_DECISION_CONTRACT:-SentryDecisionFrame.hpp}"
SENTRY_PROTOCOL_HEADER="${SENTRY_PROTOCOL_HEADER:-../SentryProtocol/SentryProtocol.hpp}"
CHASSIS_YAML="${SENTRY_CHASSIS_YAML:-../../User/RobotConfig/sentry_chassis.yaml}"

need() {
  rg -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

forbid_in() {
  local body="$1"
  local pattern="$2"
  local description="$3"
  if rg -q -- "$pattern" <<<"$body"; then
    echo "forbidden: $description" >&2
    exit 1
  fi
}

need_in() {
  local body="$1"
  local pattern="$2"
  local description="$3"
  rg -q -- "$pattern" <<<"$body" || {
    echo "missing: $description" >&2
    exit 1
  }
}

extract_body() {
  local function_name="$1"
  sed -n "/void ${function_name}/,/^  }/p" "$HEADER"
}

dispatch_body="$(extract_body HandleCanFrame)"
need_in "$dispatch_body" 'offset == DECISION_ID_OFFSET' \
  'sentry decision CAN frame dispatch'
need_in "$dispatch_body" 'HandleDecisionFrame\(pack\)' \
  'sentry decision frame handler dispatch'

need 'DECISION_ID_OFFSET = 0x1fU' 'collision-free decision CAN offset'
need 'SentryDecisionFrame.hpp' 'sentry decision frame contract include'
need 'struct DecisionUpdate' 'decision callback update payload'
need 'DECISION_UPDATE_QUEUE_CAPACITY = 32U' \
  'fixed decision update drain budget'
need_in "$(tr '\n' ' ' <"$HEADER")" \
  'LibXR::MPMCQueue<DecisionUpdate> decision_updates_\{ *DECISION_UPDATE_QUEUE_CAPACITY\}' \
  'bounded decision update queue'
need 'std::atomic<uint32_t> decision_update_drops_\{0\}' \
  'atomic decision update drop counter'
need 'SentryDecisionFrame pending_decision_' 'pending decision frame'
need 'SentryDecisionFrame active_decision_' 'active decision frame'
need 'SentryDecision::RetryController decision_retry_' \
  'decision retry controller'
need 'SentryDecision::SequenceTracker decision_sequence_tracker_' \
  'decision sequence tracker'
need 'uint32_t last_decision_rx_time_ms_' 'separate decision freshness timestamp'

register_body="$(extract_body RegisterDecisionTopics)"
register_flat="$(tr '\n' ' ' <<<"$register_body")"
for contract in \
  'FindOrCreate<uint16_t>.*sentry_buy_bullet_num_topic_name_, nullptr\)' \
  'FindOrCreate<uint8_t>.*sentry_remote_buy_bullet_times_topic_name_, nullptr\)' \
  'FindOrCreate<uint8_t>.*sentry_remote_buy_hp_times_topic_name_, nullptr\)' \
  'FindOrCreate<bool>.*sentry_buy_resurrection_topic_name_, nullptr\)' \
  'FindOrCreate<uint8_t>.*sentry_state_topic_name_, nullptr\)'; do
  need_in "$register_flat" "$contract" \
    "typed decision topic registration: $contract"
done
forbid_in "$register_flat" 'FindOrCreate<.*true' \
  'decision topics opt into multi-publisher mutex mode'

sentry_protocol_flat="$(tr '\n' ' ' <"$SENTRY_PROTOCOL_HEADER")"
for type_and_name in \
  'uint16_t.*buy_bullet_topic_name' \
  'uint8_t.*remote_buy_bullet_times_topic_name' \
  'uint8_t.*remote_buy_hp_times_topic_name' \
  'bool.*buy_resurrection_topic_name' \
  'uint8_t.*state_topic_name'; do
  rg -q -- "CreateTopic<${type_and_name}" <<<"$sentry_protocol_flat" || {
    echo "missing: SentryProtocol single-publisher topic: $type_and_name" >&2
    exit 1
  }
done
if rg -q -- 'CreateTopic<[^)]*, *true\)' <<<"$sentry_protocol_flat"; then
  echo 'forbidden: SentryProtocol decision topic uses multi-publisher mode' >&2
  exit 1
fi

sentry_protocol_line="$(rg -n 'name: SentryProtocol' "$CHASSIS_YAML" | head -1 | cut -d: -f1)"
dual_board_line="$(rg -n 'name: DualBoard' "$CHASSIS_YAML" | head -1 | cut -d: -f1)"
if [[ -z "$sentry_protocol_line" || -z "$dual_board_line" ||
      "$sentry_protocol_line" -ge "$dual_board_line" ]]; then
  echo 'missing: current chassis startup order with SentryProtocol before DualBoard' >&2
  exit 1
fi
for callback in OnSentryBuyBullet OnSentryRemoteBuyBullet \
  OnSentryRemoteBuyHp OnSentryBuyResurrection OnSentryState; do
  need_in "$register_body" "$callback" "gimbal decision callback: $callback"
  callback_body="$(extract_body "$callback")"
  need_in "$callback_body" 'EnqueueDecisionUpdate' \
    "$callback only enqueues an update"
  forbid_in "$callback_body" 'pending_decision_|active_decision_|data_mutex_' \
    "$callback mutates owner-only decision state"
done

enqueue_body="$(extract_body EnqueueDecisionUpdate)"
need_in "$enqueue_body" 'decision_updates_\.Push' 'decision update queue push'
need_in "$enqueue_body" 'decision_update_drops_\.fetch_add' \
  'observable decision update overflow'
forbid_in "$enqueue_body" 'pending_decision_|active_decision_|data_mutex_' \
  'enqueue path mutates owner-only decision state'

drain_body="$(extract_body DrainDecisionUpdates)"
drain_flat="$(tr '\n' ' ' <<<"$drain_body")"
need_in "$drain_flat" \
  'for \(size_t processed = 0U; processed < DECISION_UPDATE_QUEUE_CAPACITY;' \
  'bounded decision update drain loop'
need_in "$drain_body" 'decision_updates_\.Pop' 'protocol-thread decision queue drain'
need_in "$drain_body" '2047U' 'saturating bullet aggregation'
need_in "$drain_body" '15U' 'saturating remote-count aggregation'
need_in "$drain_body" 'pending_decision_' 'pending decision aggregation'

send_body="$(extract_body SendDecisionFrameIfDue)"
need_in "$send_body" 'decision_retry_\.Due\(now_ms\)' '10 ms retry scheduling'
need_in "$send_body" 'SendClassicFrame\(tx_id_ \+ DECISION_ID_OFFSET, active_decision_\)' \
  'decision CAN transmission'
need_in "$send_body" 'decision_retry_\.OnSendResult' \
  'successful-send retry accounting'

protocol_body="$(extract_body RunProtocolThread)"
need_in "$protocol_body" 'DrainDecisionUpdates\(\)' 'protocol-thread update ownership'
need_in "$protocol_body" 'SendDecisionFrameIfDue\(now_ms\)' \
  'protocol-thread decision retry service'
need_in "$protocol_body" 'ReportDecisionUpdateDrops\(now_ms\)' \
  'rate-limited protocol-thread overflow reporting'
need 'XR_LOG_WARN' 'decision update overflow warning'

handle_body="$(extract_body HandleDecisionFrame)"
need_in "$handle_body" 'SentryDecision::Validate\(frame\)' 'decision frame validation'
need_in "$handle_body" 'last_decision_rx_time_ms_ = now_ms' \
  'valid decision freshness update'
need_in "$handle_body" 'decision_sequence_tracker_\.Accept' \
  'decision sequence deduplication'
forbid_in "$handle_body" 'last_rx_time_ms_|online_|safe_state_published_' \
  'decision traffic mutates motion-link safety state'
for topic in sentry_buy_bullet_num_topic_ sentry_remote_buy_bullet_times_topic_ \
  sentry_remote_buy_hp_times_topic_ sentry_buy_resurrection_topic_ \
  sentry_state_topic_; do
  need_in "$handle_body" "${topic}.*Publish" "legacy decision publication: $topic"
done

rg -q -- 'RETRY_PERIOD_MS = 10U' "$CONTRACT" || {
  echo 'missing: 10 ms retry interval' >&2
  exit 1
}
rg -q -- 'REQUIRED_SEND_SUCCESSES = 5U' "$CONTRACT" || {
  echo 'missing: five successful decision sends' >&2
  exit 1
}
rg -q -- 'RX_TIMEOUT_MS = 100U' "$CONTRACT" || {
  echo 'missing: 100 ms decision sequence expiry' >&2
  exit 1
}

if [[ "${SENTRY_DECISION_MUTATION_CHILD:-0}" != "1" ]]; then
  mutant_dir="$(mktemp -d)"
  trap 'rm -rf "$mutant_dir"' EXIT

  sed 's/DECISION_ID_OFFSET = 0x1fU/DECISION_ID_OFFSET = 0x20U/' "$HEADER" \
    >"$mutant_dir/wrong_offset.hpp"
  if SENTRY_DECISION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/wrong_offset.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: colliding decision CAN offset' >&2
    exit 1
  fi

  sed '/void HandleDecisionFrame/,/^  }/ s/last_decision_rx_time_ms_/last_rx_time_ms_/' \
    "$HEADER" >"$mutant_dir/motion_timestamp.hpp"
  if SENTRY_DECISION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/motion_timestamp.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: decision refreshed motion timestamp' >&2
    exit 1
  fi

  sed '/void OnSentryState/,/^  }/ s/EnqueueDecisionUpdate/ApplyDecisionUpdate/' \
    "$HEADER" >"$mutant_dir/callback_owner_violation.hpp"
  if SENTRY_DECISION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/callback_owner_violation.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: callback bypassed decision queue' >&2
    exit 1
  fi

  sed '/void HandleDecisionFrame/,/^  }/ s/SentryDecision::Validate(frame)/true/' \
    "$HEADER" >"$mutant_dir/no_validation.hpp"
  if SENTRY_DECISION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/no_validation.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: receiver skipped validation' >&2
    exit 1
  fi

  sed '/void RegisterDecisionTopics/,/^  }/ s/nullptr))/nullptr, true))/' \
    "$HEADER" >"$mutant_dir/multi_publisher_topics.hpp"
  if SENTRY_DECISION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/multi_publisher_topics.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: decision topics enabled multi-publisher mutex mode' >&2
    exit 1
  fi

  sed '/void DrainDecisionUpdates/,/^  }/ s/for (size_t processed/while (size_t processed/' \
    "$HEADER" >"$mutant_dir/unbounded_drain.hpp"
  if SENTRY_DECISION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/unbounded_drain.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: decision update drain lost its fixed budget' >&2
    exit 1
  fi

  sed '/DECISION_UPDATE_QUEUE_CAPACITY = 32U/d' "$HEADER" \
    >"$mutant_dir/missing_drain_budget.hpp"
  if SENTRY_DECISION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/missing_drain_budget.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: decision update drain budget was removed' >&2
    exit 1
  fi

  echo 'PASS: representative sentry decision mutations rejected'
fi

echo 'PASS: DualBoard sentry decision static regression checks'
