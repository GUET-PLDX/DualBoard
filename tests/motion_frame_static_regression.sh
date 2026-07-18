#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-DualBoard.hpp}"

need() {
  rg -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
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

need 'struct __attribute__\(\(packed\)\) MotionFrame' 'packed MotionFrame'
need 'int16_t gyro_z_q;' 'gyro z payload'
need 'uint8_t gyro_valid;' 'gyro validity byte'
need 'uint8_t reserved\[5\];' 'reserved payload'
need 'static_assert\(sizeof\(MotionFrame\) == 8' '8-byte MotionFrame'
need 'GYRO_SCALE = 900\.0f' 'gyro fixed-point scale'
need 'tx_id_ \+ ANGLE_ID_OFFSET' '0x10 direction-specific CAN offset'
need 'CONTROL_PERIOD_MS = 10' '10 ms send period'
need 'FindOrCreate<ChassisMotionState>' 'typed chassis motion state Topic lookup'
need 'ChassisMotionState.hpp' 'shared chassis motion state contract include'
need 'CHASSIS_MOTION_STATE_TOPIC_MULTI_PUBLISHER' 'centralized chassis motion state Topic attributes'
forbid 'FindOrCreate<float>' 'raw gyro Topic lookup'
forbid 'chassis_gyro_z_topic_' 'raw gyro Topic member'
need '"chassis_gyro"' 'bottom BMI gyro subscription'
forbid 'gyro_x_q' 'gyro x payload'
forbid 'gyro_y_q' 'gyro y payload'
forbid 'chassis_alpha_z' 'chassis angular acceleration'
forbid 'mode_and_flags' 'mode payload'

motion_body="$(extract_body HandleMotionFrame)"
motion_flat="$(tr '\n' ' ' <<<"$motion_body")"
need_in "$motion_flat" \
  'last_rx_time_ms_ = now_ms' \
  'MotionFrame refreshes the existing DualBoard link timestamp'
need_in "$motion_body" 'online_ = true' \
  'MotionFrame establishes the existing DualBoard online state'
need_in "$motion_body" 'safe_state_published_ = false' \
  'MotionFrame re-arms the existing DualBoard offline safe state'

motion_definition="$(sed -n \
  '/struct __attribute__((packed)) MotionFrame/,/^  };/p' "$HEADER")"
if rg -q 'gyro_[xy]_q|sequence|mode_and_flags|chassis_alpha_z|replay|fade|timeout' \
    <<<"$motion_definition"; then
  echo 'forbidden: MotionFrame contains non-minimal payload or state' >&2
  exit 1
fi

send_body="$(extract_body SendMotionFrameIfDue)"
send_flat="$(tr '\n' ' ' <<<"$send_body")"
need_in "$send_body" 'IsDue\(now_ms, next_control_tx_ms_, CONTROL_PERIOD_MS\)' \
  'MotionFrame reuses the 10 ms chassis scheduler'
need_in "$send_body" 'std::isfinite\(gyro_z\)' \
  'MotionFrame rejects non-finite gyro samples'
need_in "$send_flat" 'gyro_z_q >=.*numeric_limits<int16_t>::min' \
  'MotionFrame rejects gyro underflow'
need_in "$send_flat" 'gyro_z_q <=.*numeric_limits<int16_t>::max' \
  'MotionFrame rejects gyro overflow'
need_in "$send_body" 'frame\.gyro_valid = 1U' \
  'MotionFrame marks only encodable gyro samples valid'
need_in "$send_body" 'SendClassicFrame\(tx_id_ \+ ANGLE_ID_OFFSET, frame\)' \
  'MotionFrame uses the direction-specific 0x10 CAN ID'

callback_body="$(extract_body OnLocalChassisGyro)"
need_in "$callback_body" 'LibXR::Mutex::LockGuard lock\(data_mutex_\)' \
  'chassis gyro callback locks shared data'
need_in "$callback_body" 'local_chassis_gyro_ = gyro' \
  'chassis gyro callback stores the latest sample'

register_body="$(extract_body RegisterRoleTopics)"
need_in "$register_body" 'RegisterTopicCallback<Eigen::Matrix<float, 3, 1>,' \
  'chassis role subscribes to the BMI088 gyro vector'
need_in "$register_body" 'FindOrCreate<ChassisMotionState>' \
  'gimbal role directly finds or creates the semantic motion state Topic'
need_in "$register_body" 'CHASSIS_MOTION_STATE_TOPIC_NAME' \
  'semantic motion state Topic uses centralized contract'

protocol_body="$(extract_body RunProtocolThread)"
need_in "$protocol_body" 'SendMotionFrameIfDue\(now_ms\)' \
  'chassis protocol thread schedules MotionFrame transmission'

dispatch_body="$(extract_body HandleCanFrame)"
need_in "$dispatch_body" 'offset == ANGLE_ID_OFFSET' \
  'gimbal dispatch recognizes the MotionFrame CAN offset'
need_in "$dispatch_body" 'HandleMotionFrame\(pack\)' \
  'gimbal dispatch invokes MotionFrame decoding'

need_in "$motion_body" 'std::memcpy\(&frame, pack\.data, sizeof\(frame\)\)' \
  'MotionFrame is decoded from the classic CAN payload'
need_in "$motion_body" 'frame\.gyro_valid == 1U' \
  'MotionFrame validity flag controls semantic validity'
need_in "$motion_body" 'DecodeSigned\(frame\.gyro_z_q, GYRO_SCALE\)' \
  'valid MotionFrames decode the fixed-point yaw rate'
need_in "$motion_body" 'motion_state_\.yaw_rate_valid' \
  'decoded validity is published in the semantic state'
need_in "$motion_body" 'motion_state_\.online = true' \
  'received MotionFrame publishes online state'
need_in "$motion_body" 'std::isfinite\(gyro_z\)' \
  'decoded MotionFrame rejects non-finite samples'
need_in "$motion_flat" \
  'LibXR::Mutex::LockGuard lock\(data_mutex_\).*PublishMotionStateLocked\(\)' \
  'MotionFrame semantic publication is serialized by data_mutex_'
need_in "$motion_flat" 'motion_state_\.yaw_rate_rad_s =.*\? gyro_z : 0\.0f' \
  'invalid MotionFrames publish zero rate'

offline_body="$(extract_body PublishInvalidLauncherFeedback)"
offline_flat="$(tr '\n' ' ' <<<"$offline_body")"
need_in "$offline_body" 'motion_state_ = \{\}' \
  'full-link offline handling clears the complete motion state'
need_in "$offline_body" 'PublishMotionStateLocked\(\)' \
  'full-link offline handling publishes invalid motion state'
need_in "$offline_body" 'LibXR::Mutex::LockGuard lock\(data_mutex_\)' \
  'offline semantic publication is serialized by data_mutex_'
need_in "$offline_body" 'launcher_ref_topic_\.Publish\(launcher_pack\)' \
  'offline launcher publication remains available'
need_in "$offline_body" 'PublishMotionStateLocked\(\)' \
  'offline semantic state is published'

if [[ "${MOTION_REGRESSION_MUTATION_CHILD:-0}" != "1" ]]; then
  mutant_dir="$(mktemp -d)"
  trap 'rm -rf "$mutant_dir"' EXIT

  sed 's/SendClassicFrame(tx_id_ + ANGLE_ID_OFFSET, frame);/SendClassicFrame(tx_id_ + CONTROL_ID_OFFSET, frame);/' \
    "$HEADER" >"$mutant_dir/wrong_offset.hpp"
  if MOTION_REGRESSION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/wrong_offset.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: wrong MotionFrame CAN offset' >&2
    exit 1
  fi

  sed 's/frame.gyro_valid == 1U/true/g' "$HEADER" \
    >"$mutant_dir/ignored_validity.hpp"
  if MOTION_REGRESSION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/ignored_validity.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: ignored MotionFrame validity flag' >&2
    exit 1
  fi

  sed '/void HandleMotionFrame/,/^  }/ s/LibXR::Mutex::LockGuard lock(data_mutex_);//' \
    "$HEADER" >"$mutant_dir/unlocked_motion_publish.hpp"
  if MOTION_REGRESSION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/unlocked_motion_publish.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: unlocked MotionFrame gyro publication' >&2
    exit 1
  fi

  sed '/void HandleMotionFrame/,/^  }/{
    /last_rx_time_ms_ =/d
  }' "$HEADER" >"$mutant_dir/missing_motion_timestamp.hpp"
  if MOTION_REGRESSION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/missing_motion_timestamp.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: MotionFrame does not refresh link timestamp' >&2
    exit 1
  fi

  sed '/void PublishInvalidLauncherFeedback/,/^  }/ s/LibXR::Mutex::LockGuard lock(data_mutex_);//' \
    "$HEADER" >"$mutant_dir/unlocked_offline_publish.hpp"
  if MOTION_REGRESSION_MUTATION_CHILD=1 bash "$0" \
      "$mutant_dir/unlocked_offline_publish.hpp" >/dev/null 2>&1; then
    echo 'mutation survived: unlocked offline semantic publication' >&2
    exit 1
  fi

  echo 'PASS: representative MotionFrame mutations rejected'
fi

echo 'PASS: DualBoard motion static regression checks'
