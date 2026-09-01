#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
header="$module_dir/DualBoard.hpp"

python3 - "$header" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1]).read_text(encoding="utf-8")
required = (
    "struct __attribute__((packed)) ChassisYawFrame",
    "int16_t yaw_q;",
    "uint8_t valid;",
    "uint8_t sequence;",
    "uint32_t sample_time_ms;",
    "CHASSIS_YAW_ID_OFFSET = 0x1FU",
    "CHASSIS_YAW_TIMEOUT_MS = 100U",
    "sizeof(ChassisYawFrame) == 8",
    "SendChassisYawFrameIfDue(now_ms);",
    "HandleChassisYawFrame(pack);",
    "chassis_imu_yaw_valid_topic_.Publish(false)",
    "local_chassis_yaw_time_ms_",
    "now_ms - chassis_yaw_time_ms <=",
    "const bool accepted = valid && std::isfinite(yaw);",
)
for token in required:
    if token not in source:
        raise SystemExit(f"missing chassis yaw transport contract: {token}")
print("PASS: DualBoard chassis yaw transport static contract")
PY
