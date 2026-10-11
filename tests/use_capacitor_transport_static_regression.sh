#!/usr/bin/env bash
set -euo pipefail

module_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
header="$module_dir/DualBoard.hpp"
contract="$header"

python3 - "$header" "$contract" <<'PY'
from pathlib import Path
import sys

header = Path(sys.argv[1]).read_text(encoding="utf-8")
contract = Path(sys.argv[2]).read_text(encoding="utf-8")
required_header = (
    "USE_CAPACITOR_ID_OFFSET",
    "Pldx::DualBoardControl::USE_CAPACITOR_ID_OFFSET",
    "OnNavChassisMode",
    "OnLocalUseCapacitor",
    "HandleUseCapacitorFrame(pack)",
    "SendClassicFrame(tx_id_ + USE_CAPACITOR_ID_OFFSET, use_capacitor_frame)",
    "void OnLocalModeEvent(uint32_t mode)",
    "local_use_capacitor_ = true",
    'CreateTopic<uint32_t>(Pldx::DualBoardControl::NAV_CHASSIS_MODE_TOPIC)',
    'CreateTopic<bool>(Pldx::DualBoardControl::USE_CAPACITOR_TOPIC)',
)
for token in required_header:
    if token not in header:
        raise SystemExit(f"missing DualBoard use_capacitor contract: {token}")
if "RegisterTopicCallback<uint32_t, &DualBoard::OnLocalModeEvent>(mode_topic_name_)" in header:
    raise SystemExit("forbidden: DualBoard subscribed mode_topic_ from nav")
required_contract = (
    "USE_CAPACITOR_ID_OFFSET = 0x11U",
    'NAV_CHASSIS_MODE_TOPIC[] = "nav_chassis_mode"',
    'USE_CAPACITOR_TOPIC[] = "use_capacitor"',
    "struct __attribute__((packed)) UseCapacitorCommand",
    "RESERVED_MASK = 0xF0U",
)
for token in required_contract:
    if token not in contract:
        raise SystemExit(f"missing DualBoardControl use_capacitor contract: {token}")
print("PASS: DualBoard use_capacitor transport static contract")
PY
