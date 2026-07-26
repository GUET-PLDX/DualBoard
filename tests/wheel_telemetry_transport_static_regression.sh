#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
header="$module_dir/DualBoard.hpp"

python3 - "$header" <<'PY'
from pathlib import Path
import re
import sys

source = Path(sys.argv[1]).read_text()
required = {
    "meta ID": "WHEEL_TELEMETRY_META_ID_OFFSET = 0x17U",
    "pair01 ID": "WHEEL_TELEMETRY_PAIR01_ID_OFFSET = 0x18U",
    "diagnostic ID": "WHEEL_TELEMETRY_DIAGNOSTIC_ID_OFFSET = 0x19U",
    "pair23 ID": "WHEEL_TELEMETRY_PAIR23_ID_OFFSET = 0x1AU",
    "20 ms assembly timeout": "WHEEL_ASSEMBLY_TIMEOUT_MS = 20U",
    "50 ms stream timeout": "WHEEL_STREAM_TIMEOUT_MS = 50U",
    "10 Hz heartbeat": "WHEEL_STALE_HEARTBEAT_MS = 100U",
    "chassis timestamp preservation": "telemetry.sample_time_us & 0xFFFFFFFFU",
    "semantic topic": '"chassis_wheel_telemetry"',
    "required fragment mask": "WHEEL_REQUIRED_MASK",
}
for description, token in required.items():
    if token not in source:
        raise SystemExit(f"missing {description}: {token}")
for layout in (
    "sizeof(WheelTelemetryMetaFrame) == 8",
    "sizeof(WheelTelemetryPairFrame) == 8",
    "sizeof(WheelTelemetryDiagnosticFrame) == 8",
):
    if layout not in source:
        raise SystemExit(f"missing packed layout assertion: {layout}")

send_start = source.index("  void SendWheelTelemetryIfPending()")
send_end = source.index("  void SendGimbalControlFrames", send_start)
send = source[send_start:send_end]
if "target_motor_omega_" in send:
    raise SystemExit("target wheel speed substituted for measured telemetry")
if "telemetry.wheel_angular_velocity" not in send:
    raise SystemExit("measured wheel telemetry is not encoded")
if not re.search(r"ENCODING_SATURATED.*?~ChassisWheelTelemetry::FRESH", source, re.S):
    raise SystemExit("Q8.8 saturation must set saturation and clear freshness")
if "CheckWheelTelemetryWatchdog(now_ms);\n        CheckOffline(now_ms);" not in source:
    raise SystemExit("dedicated wheel watchdog missing from gimbal protocol loop")

wheel_rx_start = source.index("  bool HandleWheelTelemetryFrame")
wheel_rx_end = source.index("  void HandleDecisionFrame", wheel_rx_start)
wheel_rx = source[wheel_rx_start:wheel_rx_end]
if re.search(r"\b(?:online_|last_rx_time_ms_)\s*=", wheel_rx):
    raise SystemExit("wheel stream must not broaden general DualBoard online state")

mutant = source.replace("WHEEL_TELEMETRY_PAIR23_ID_OFFSET = 0x1AU",
                        "WHEEL_TELEMETRY_PAIR23_ID_OFFSET = 0x1FU")
collision_guard = source[source.index("static_assert(WHEEL_TELEMETRY_META_ID_OFFSET"):
                         source.index("static void RxThreadEntry")]
if "DECISION_ID_OFFSET" not in collision_guard or mutant == source:
    raise SystemExit("CAN ID collision mutation guard missing")
PY

printf 'PASS: DualBoard wheel telemetry static contract\n'
