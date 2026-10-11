#!/usr/bin/env bash
set -euo pipefail

header="${1:-DualBoard.hpp}"

extract_body() {
  local function_name="$1"
  sed -n "/void ${function_name}/,/^  }/p" "$header"
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

forbid_in() {
  local body="$1"
  local pattern="$2"
  local description="$3"
  if rg -q -- "$pattern" <<<"$body"; then
    echo "forbidden: $description" >&2
    exit 1
  fi
}

register_body="$(extract_body RegisterCmdEvent)"
need_in "$register_body" 'CMD_EVENT_LOST_CTRL' \
  'lost-control event registration'
need_in "$register_body" 'CMD_EVENT_START_CTRL' \
  'start-control event registration'
need_in "$register_body" 'ClearLocalModeRelax\(\)' \
  'start-control callback clears the RELAX latch'

clear_body="$(extract_body ClearLocalModeRelax)"
need_in "$clear_body" 'LibXR::Mutex::LockGuard lock\(data_mutex_\)' \
  'RELAX latch clear is synchronized'
need_in "$clear_body" 'state_\.output_relax_ = false' \
  'RELAX latch clear assignment'

send_body="$(extract_body SendGimbalControlFrames)"
need_in "$send_body" \
  'state_\.output_relax_ || \(cmd_ != nullptr && !cmd_->Online\(\)\)' \
  'control output remains RELAX while the latch or CMD offline state is active'
forbid_in "$send_body" 'state_\.output_relax_ = false' \
  'control loop clears the safety latch without a start-control event'

echo 'PASS: DualBoard control recovery static regression checks'
