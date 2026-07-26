#!/usr/bin/env bash
set -euo pipefail

test_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
binary=$(mktemp /tmp/wheel_telemetry_transport_test.XXXXXX)
trap 'rm -f "$binary"' EXIT

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror -pedantic \
  "$test_dir/wheel_telemetry_transport_test.cpp" -o "$binary"
"$binary"
printf 'PASS: DualBoard wheel telemetry transport model\n'
