#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_dir="$(cd "${test_dir}/.." && pwd)"
workspace_root="$(cd "${module_dir}/../.." && pwd)"
libxr_dir="${workspace_root}/Middlewares/Third_Party/LibXR"
binary="$(mktemp "${TMPDIR:-/tmp}/dual-board-host-test.XXXXXX")"
trap 'rm -f "${binary}"' EXIT

includes=(-I"${module_dir}" -isystem "${libxr_dir}/system/linux"
          -isystem "${libxr_dir}/lib/Eigen")
while IFS= read -r directory; do
  includes+=(-isystem "${directory}")
done < <(find "${libxr_dir}/src" -type d)
for module in CMD Chassis Motor PowerControl Referee RMMotor SuperPower; do
  includes+=(-isystem "${workspace_root}/Modules/${module}")
done

"${CXX:-c++}" -std=c++20 -Wall -Wextra -Werror -pedantic \
  -DXR_LOG_MESSAGE_MAX_LEN=64 -DLIBXR_DEFAULT_SCALAR=double \
  "${includes[@]}" "${test_dir}/${1:?test source required}" \
  "${libxr_dir}/src/core/libxr_mem_o3.cpp" \
  "${libxr_dir}/src/structure/list.cpp" \
  "${libxr_dir}/system/linux/mutex.cpp" -pthread -o "${binary}"
"${binary}"
