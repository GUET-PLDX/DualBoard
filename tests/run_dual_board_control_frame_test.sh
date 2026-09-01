#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_dir="$(cd "${test_dir}/.." && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/dual-board-control-frame-test.XXXXXX")"
trap 'rm -f "${binary}"' EXIT

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror -pedantic \
  -I"${module_dir}" "${test_dir}/dual_board_control_frame_test.cpp" \
  -o "${binary}"
"${binary}"

echo "PASS: dual-board control frame tests"
