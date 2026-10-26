#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "${test_dir}/run_host_test.sh" chassis_feedforward_frame_test.cpp
echo "PASS: chassis feedforward frame tests"
