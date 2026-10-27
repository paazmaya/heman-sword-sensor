#!/usr/bin/env bash
# Validate the linked artifact, not just Cargo's exit code.
set -euo pipefail

fail() {
  echo "Firmware layout check failed: $*" >&2
  exit 1
}

image="${1:-target/thumbv7em-none-eabihf/release/xiao_nrf52840_sword}"
host=$(rustc -vV | sed -n 's/^host: //p')
tools="$(rustc --print sysroot)/lib/rustlib/$host/bin"
entry=$("$tools/llvm-readobj" --file-headers "$image" | awk '/Entry:/ {print $2}')
if [[ -z "$entry" ]] || ! (( entry >= 0x26001 && entry < 0xEF000 && (entry & 1) == 1 )); then
  echo "Invalid firmware entry point: $entry" >&2
  exit 1
fi
sections=$("$tools/llvm-objdump" -h "$image")
vector=$(printf '%s\n' "$sections" | awk '$2 == ".vector_table" {print $4}')
text_size=$(printf '%s\n' "$sections" | awk '$2 == ".text" {print $3}')
text_start=$(printf '%s\n' "$sections" | awk '$2 == ".text" {print $4}')
if [[ "$vector" != 00026000 || -z "$text_size" ]] || ! (( 16#$text_size > 1024 )); then
  echo "Missing application vector table or executable content" >&2
  exit 1
fi
(( (entry & ~1) >= 16#$text_start && (entry & ~1) < 16#$text_start + 16#$text_size )) \
  || fail "entry is outside executable code"
reset=$("$tools/llvm-nm" --defined-only "$image" | awk '$3 == "Reset" {print $1}')
[[ -n "$reset" ]] || fail "missing Reset handler"
(( (entry & ~1) == 16#$reset )) || fail "entry does not point to Reset"
vector_words=$("$tools/llvm-objdump" -s --section=.vector_table "$image" | awk '$1 == "26000" {print $2, $3}')
read -r sp reset_word <<< "$vector_words"
[[ ${#sp} == 8 && ${#reset_word} == 8 ]] || fail "missing initial vectors"
sp=$((16#${sp:6:2}${sp:4:2}${sp:2:2}${sp:0:2}))
reset_word=$((16#${reset_word:6:2}${reset_word:4:2}${reset_word:2:2}${reset_word:0:2}))
(( sp > 0x20010000 && sp <= 0x20040000 && sp % 8 == 0 )) || fail "invalid initial stack pointer"
(( reset_word == entry )) || fail "vector Reset differs from ELF entry"

while read -r name size vma lma; do
  size=$((16#$size)); vma=$((16#$vma)); lma=$((16#$lma))
  (( size == 0 )) && continue
  case "$name" in
    .vector_table|.text|.rodata|.data|.gnu.sgstubs)
      (( lma >= 0x26000 && lma + size <= 0xEF000 )) || fail "$name overlaps reserved flash" ;;
    .bss|.uninit)
      (( vma >= 0x20010000 && vma + size <= 0x20040000 )) || fail "$name exceeds application RAM" ;;
  esac
done < <(printf '%s\n' "$sections" | awk '$2 ~ /^\./ {print $2, $3, $4, $5}')

programs=$("$tools/llvm-objdump" -p "$image")
loads=$(printf '%s\n' "$programs" | awk '/LOAD off/ {v=$5; p=$7; getline; print v, p, $2, $4}')
[[ -n "$loads" ]] || fail "missing load segments"
while read -r vma lma filesz memsz; do
  vma=$((vma)); lma=$((lma)); filesz=$((filesz)); memsz=$((memsz))
  if (( filesz > 0 )); then
    (( lma >= 0x26000 && lma + filesz <= 0xEF000 )) || fail "LOAD overlaps reserved flash"
  fi
  if (( vma >= 0x20000000 )); then
    (( vma >= 0x20010000 && vma + memsz <= 0x20040000 )) || fail "LOAD exceeds application RAM"
  else
    (( vma >= 0x26000 && vma + memsz <= 0xEF000 )) || fail "LOAD exceeds application flash"
  fi
done <<< "$loads"

"$tools/llvm-size" "$image"
printf 'Verified entry %s, vector table, executable content and memory reservations.\n' "$entry"
