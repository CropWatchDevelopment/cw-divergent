#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: check_elf_policy.sh <elf> [map]

Checks the built ELF against production policy:
- no heap / formatter / soft-float helper symbols
- .noinit section present
- __StackTop and __StackLimit exported
- static SRAM headroom above the configured minimum
- linker heap reservation is zero when a map file is provided
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage >&2
  exit 2
fi

elf_path=$1
map_path=${2:-}

if [[ ! -f "$elf_path" ]]; then
  echo "ELF not found: $elf_path" >&2
  exit 1
fi

nm_tool=${NM:-arm-none-eabi-nm}
readelf_tool=${READELF:-arm-none-eabi-readelf}
size_tool=${SIZE:-arm-none-eabi-size}

for required_tool in "$nm_tool" "$readelf_tool" "$size_tool"; do
  if ! command -v "$required_tool" >/dev/null 2>&1; then
    echo "Required tool not found in PATH: $required_tool" >&2
    exit 1
  fi
done

SRAM_BYTES=${SRAM_BYTES:-20480}
STACK_BYTES=${STACK_BYTES:-5120}
HEAP_BYTES=${HEAP_BYTES:-0}
MIN_STATIC_HEADROOM_BYTES=${MIN_STATIC_HEADROOM_BYTES:-5120}

tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT

"$nm_tool" -C "$elf_path" >"$tmpdir/nm.txt"
"$readelf_tool" -S "$elf_path" >"$tmpdir/sections.txt"
"$size_tool" "$elf_path" >"$tmpdir/size.txt"

forbidden_regex='^(_malloc_r|_free_r|_realloc_r|_calloc_r|malloc|free|realloc|_sbrk|_sbrk_r|_printf_float|_scanf_float|__aeabi_f[A-Za-z0-9_]*|__aeabi_d[A-Za-z0-9_]*|vsnprintf|snprintf|_svfiprintf_r)$'

mapfile -t forbidden_symbols < <(
  awk '{print $NF}' "$tmpdir/nm.txt" |
    grep -E "$forbidden_regex" |
    sort -u
)

if (( ${#forbidden_symbols[@]} > 0 )); then
  echo "Forbidden symbols found in $(basename "$elf_path"):" >&2
  printf '  %s\n' "${forbidden_symbols[@]}" >&2
  exit 1
fi

if ! grep -q '\.noinit' "$tmpdir/sections.txt"; then
  echo "Missing .noinit section in $(basename "$elf_path")" >&2
  exit 1
fi

for required_symbol in __StackTop __StackLimit; do
  if ! awk '{print $NF}' "$tmpdir/nm.txt" | grep -qx "$required_symbol"; then
    echo "Missing required linker symbol $required_symbol in $(basename "$elf_path")" >&2
    exit 1
  fi
done

read -r text_bytes data_bytes bss_bytes _ < <(awk 'NR == 2 { print $1, $2, $3, $4 }' "$tmpdir/size.txt")
static_sram_bytes=$((data_bytes + bss_bytes))
reserved_sram_bytes=$((static_sram_bytes + STACK_BYTES + HEAP_BYTES))
static_headroom_bytes=$((SRAM_BYTES - reserved_sram_bytes))

if (( static_headroom_bytes < MIN_STATIC_HEADROOM_BYTES )); then
  echo "Static SRAM headroom too small in $(basename "$elf_path"): ${static_headroom_bytes} bytes" >&2
  exit 1
fi

if [[ -n "$map_path" ]]; then
  if [[ ! -f "$map_path" ]]; then
    echo "Map file not found: $map_path" >&2
    exit 1
  fi

  if ! grep -Eq '_Min_Heap_Size *= *0x0+\b' "$map_path"; then
    echo "Linker heap reservation is not zero in $(basename "$map_path")" >&2
    exit 1
  fi
fi

echo "ELF policy check passed: $elf_path"
echo "  text=${text_bytes} data=${data_bytes} bss=${bss_bytes}"
echo "  static_sram=${static_sram_bytes} reserved_sram=${reserved_sram_bytes} static_headroom=${static_headroom_bytes}"
