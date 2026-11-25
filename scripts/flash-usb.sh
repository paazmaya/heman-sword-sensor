#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "USB firmware flash failed: $*" >&2
  exit 1
}

[[ $# == 2 ]] || fail "usage: $0 firmware-elf mounted-XIAO-UF2-volume"
image="$1"
volume="$2"
[[ -f "$image" ]] || fail "firmware ELF not found: $image"
[[ -f "$volume/INFO_UF2.TXT" ]] || fail "$volume is not a mounted UF2 volume"
grep -qi 'XIAO' "$volume/INFO_UF2.TXT" || fail "$volume is not identified as a XIAO UF2 volume"

host=$(rustc -vV | sed -n 's/^host: //p')
objcopy="$(rustc --print sysroot)/lib/rustlib/$host/bin/llvm-objcopy"
[[ -x "$objcopy" ]] || fail "missing $objcopy; install with: rustup component add llvm-tools-preview"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
temp_dir=$(mktemp -d)
trap 'rm -rf -- "$temp_dir"' EXIT

hex="$temp_dir/firmware.hex"
uf2="$temp_dir/firmware.uf2"
"$objcopy" --output-target=ihex "$image" "$hex"
python3 "$script_dir/hex_to_uf2.py" "$hex" "$uf2"
cat "$uf2" > "$volume/NEW.UF2"
printf 'Sent firmware to %s/NEW.UF2; the XIAO should restart after the UF2 write completes.\n' "$volume"
