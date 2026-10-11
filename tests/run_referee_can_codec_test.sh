#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "${test_dir}/run_host_test.sh" referee_can_codec_test.cpp
echo "PASS: referee CAN codec tests"
