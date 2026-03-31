#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/.." && pwd)

toolchain_bin_default="/opt/st/stm32cubeide_1.19.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.linux64_1.0.0.202410170706/tools/bin"
toolchain_bin=${TOOLCHAIN_BIN:-$toolchain_bin_default}

export PATH="$toolchain_bin:$PATH"

CC=${CC:-arm-none-eabi-gcc}
SIZE=${SIZE:-arm-none-eabi-size}
NM=${NM:-arm-none-eabi-nm}
READELF=${READELF:-arm-none-eabi-readelf}
OBJDUMP=${OBJDUMP:-arm-none-eabi-objdump}

for required_tool in "$CC" "$SIZE" "$NM" "$READELF" "$OBJDUMP" cppcheck make python3; do
  if ! command -v "$required_tool" >/dev/null 2>&1; then
    echo "Required tool not found in PATH: $required_tool" >&2
    exit 1
  fi
done

out_dir=${1:-/tmp/cw-divergent-audit}
debug_report_dir="$out_dir/debug"
release_build_dir="$out_dir/release-build"
release_report_dir="$out_dir/release"

rm -rf "$out_dir"
mkdir -p "$debug_report_dir" "$release_build_dir" "$release_report_dir"

common_includes=(
  -I"$repo_root/Core/Inc"
  -I"$repo_root/Drivers/STM32L0xx_HAL_Driver/Inc"
  -I"$repo_root/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy"
  -I"$repo_root/Drivers/CMSIS/Device/ST/STM32L0xx/Include"
  -I"$repo_root/Drivers/CMSIS/Include"
)
common_arch_flags=(
  -mcpu=cortex-m0plus
  -mthumb
  -mfloat-abi=soft
)
common_c_flags=(
  -std=gnu11
  -DUSE_HAL_DRIVER
  -DSTM32L073xx
  -ffunction-sections
  -fdata-sections
)
release_c_flags=(
  -Os
  -g0
  -Wall
  -Wextra
  -Wconversion
  -fstack-usage
  -fcyclomatic-complexity
)
strict_warning_flags=(
  -Wall
  -Wextra
  -Wshadow
  -Wformat=2
  -Wundef
  -Wconversion
  -Werror
)

link_flags=(
  -mcpu=cortex-m0plus
  -T"$repo_root/STM32L073RZTX_FLASH.ld"
  --specs=nosys.specs
  "-Wl,-Map="
  -Wl,--gc-sections
  -static
  --specs=nano.specs
  -mfloat-abi=soft
  -mthumb
  -Wl,--start-group
  -lc
  -lm
  -Wl,--end-group
)

mapfile -t c_sources < <(cd "$repo_root" && find Core Drivers -type f -name '*.c' | sort)
mapfile -t asm_sources < <(cd "$repo_root" && find Core Drivers -type f \( -name '*.s' -o -name '*.S' \) | sort)
mapfile -t app_c_sources < <(cd "$repo_root" && find Core/Src -type f -name '*.c' | sort)

run_app_warning_pass() {
  local report_path=$1

  : >"$report_path"
  for src in "${app_c_sources[@]}"; do
    "$CC" \
      "${common_arch_flags[@]}" \
      "${common_includes[@]}" \
      "${common_c_flags[@]}" \
      "${strict_warning_flags[@]}" \
      -fsyntax-only \
      "$repo_root/$src" >>"$report_path" 2>&1
  done
}

run_cppcheck() {
  local report_path=$1

  cppcheck \
    --enable=warning,performance,portability \
    --std=c11 \
    --inline-suppr \
    --template=gcc \
    --error-exitcode=1 \
    -D__GNUC__=13 \
    -D__GNUC_MINOR__=3 \
    -D__GNUC_PATCHLEVEL__=1 \
    -D__ARM_ARCH_6M__=1 \
    -D__thumb__=1 \
    -DUSE_HAL_DRIVER \
    -DSTM32L073xx \
    -I"$repo_root/Core/Inc" \
    -I"$repo_root/Drivers/STM32L0xx_HAL_Driver/Inc" \
    -I"$repo_root/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" \
    -I"$repo_root/Drivers/CMSIS/Device/ST/STM32L0xx/Include" \
    -I"$repo_root/Drivers/CMSIS/Include" \
    "$repo_root/Core/Src" >"$report_path" 2>&1
}

archive_debug_reports() {
  "$SIZE" "$repo_root/Debug/cw-divergent.elf" >"$debug_report_dir/size.txt"
  "$NM" -C "$repo_root/Debug/cw-divergent.elf" >"$debug_report_dir/nm.txt"
  "$READELF" -A "$repo_root/Debug/cw-divergent.elf" >"$debug_report_dir/readelf-A.txt"
  "$OBJDUMP" -h -S "$repo_root/Debug/cw-divergent.elf" >"$debug_report_dir/cw-divergent.list"
  bash "$script_dir/check_elf_policy.sh" \
    "$repo_root/Debug/cw-divergent.elf" \
    "$repo_root/Debug/cw-divergent.map" >"$debug_report_dir/policy.txt"
  python3 "$script_dir/summarize_stack_usage.py" "$repo_root/Debug" >"$debug_report_dir/stack-summary.txt"
}

build_release() {
  local object_dir="$release_build_dir/obj"
  local elf_path="$release_build_dir/cw-divergent.elf"
  local map_path="$release_build_dir/cw-divergent.map"

  mkdir -p "$object_dir"

  for src in "${c_sources[@]}"; do
    local obj_path="$object_dir/${src%.c}.o"
    mkdir -p "$(dirname "$obj_path")"
    "$CC" \
      "${common_arch_flags[@]}" \
      "${common_includes[@]}" \
      "${common_c_flags[@]}" \
      "${release_c_flags[@]}" \
      -c "$repo_root/$src" \
      -o "$obj_path"
  done

  for src in "${asm_sources[@]}"; do
    local obj_path="$object_dir/${src%.*}.o"
    mkdir -p "$(dirname "$obj_path")"
    "$CC" \
      "${common_arch_flags[@]}" \
      "${common_includes[@]}" \
      -x assembler-with-cpp \
      -c "$repo_root/$src" \
      -o "$obj_path"
  done

  mapfile -t object_files < <(find "$object_dir" -type f -name '*.o' | sort)

  "$CC" \
    -o "$elf_path" \
    "${object_files[@]}" \
    -mcpu=cortex-m0plus \
    -T"$repo_root/STM32L073RZTX_FLASH.ld" \
    --specs=nosys.specs \
    "-Wl,-Map=$map_path" \
    -Wl,--gc-sections \
    -static \
    --specs=nano.specs \
    -mfloat-abi=soft \
    -mthumb \
    -Wl,--start-group \
    -lc \
    -lm \
    -Wl,--end-group
}

archive_release_reports() {
  local elf_path="$release_build_dir/cw-divergent.elf"
  local map_path="$release_build_dir/cw-divergent.map"

  "$SIZE" "$elf_path" >"$release_report_dir/size.txt"
  "$NM" -C "$elf_path" >"$release_report_dir/nm.txt"
  "$READELF" -A "$elf_path" >"$release_report_dir/readelf-A.txt"
  "$OBJDUMP" -h -S "$elf_path" >"$release_report_dir/cw-divergent.list"
  bash "$script_dir/check_elf_policy.sh" "$elf_path" "$map_path" >"$release_report_dir/policy.txt"
  python3 "$script_dir/summarize_stack_usage.py" "$release_build_dir" >"$release_report_dir/stack-summary.txt"
}

echo "Building Debug configuration with managed makefiles"
make -C "$repo_root/Debug" clean all -j"$(nproc)"
archive_debug_reports

echo "Running strict app warning pass"
run_app_warning_pass "$out_dir/app-warnings.txt"

echo "Running cppcheck on application sources"
run_cppcheck "$out_dir/cppcheck.txt"

echo "Building Release audit configuration"
build_release
archive_release_reports

echo "Audit artifacts written to $out_dir"
