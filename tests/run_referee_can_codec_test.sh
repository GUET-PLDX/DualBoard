#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_dir="$(cd "${test_dir}/.." && pwd)"
workspace_root="$(cd "${module_dir}/../.." && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/referee-can-codec-test.XXXXXX")"
trap 'rm -f "${binary}"' EXIT

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror -pedantic \
  -I"${module_dir}" \
  -I"${workspace_root}/Middlewares/Third_Party/LibXR/src/core" \
  "${test_dir}/referee_can_codec_test.cpp" \
  "${workspace_root}/Middlewares/Third_Party/LibXR/src/core/libxr_mem_o3.cpp" \
  -o "${binary}"
"${binary}"

echo "PASS: referee CAN codec tests"
