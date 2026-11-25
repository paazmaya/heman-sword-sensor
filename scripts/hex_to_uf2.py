#!/usr/bin/env python3
"""Convert the application's Intel HEX into an nRF52840 UF2 image."""

import argparse
import struct
from pathlib import Path

FLASH_START = 0x27000
FLASH_END = 0xED000
UF2_FAMILY_ID = 0xADA52840
UF2_BLOCK_SIZE = 512
UF2_PAYLOAD_SIZE = 256
UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID = 0x00002000


def parse_ihex(contents: str) -> dict[int, int]:
    memory: dict[int, int] = {}
    upper_address = 0
    eof_seen = False

    for line_number, line in enumerate(contents.splitlines(), start=1):
        line = line.strip()
        if not line:
            continue
        if eof_seen or not line.startswith(":"):
            raise ValueError(f"invalid Intel HEX record on line {line_number}")

        try:
            record = bytes.fromhex(line[1:])
        except ValueError as error:
            raise ValueError(f"invalid Intel HEX record on line {line_number}") from error
        if len(record) < 5 or len(record) != record[0] + 5 or sum(record) & 0xFF:
            raise ValueError(f"invalid Intel HEX length or checksum on line {line_number}")

        address = (record[1] << 8) | record[2]
        record_type = record[3]
        data = record[4:-1]

        if record_type == 0:
            start = upper_address + address
            for offset, value in enumerate(data):
                current = start + offset
                if not FLASH_START <= current < FLASH_END:
                    raise ValueError(f"address 0x{current:08x} is outside application flash")
                if current in memory and memory[current] != value:
                    raise ValueError(f"conflicting data at address 0x{current:08x}")
                memory[current] = value
        elif record_type == 1:
            if address or data:
                raise ValueError(f"invalid EOF record on line {line_number}")
            eof_seen = True
        elif record_type == 2:
            if len(data) != 2:
                raise ValueError(f"invalid segment address on line {line_number}")
            upper_address = int.from_bytes(data, "big") << 4
        elif record_type == 4:
            if len(data) != 2:
                raise ValueError(f"invalid linear address on line {line_number}")
            upper_address = int.from_bytes(data, "big") << 16
        elif record_type in (3, 5):
            if len(data) != 4:
                raise ValueError(f"invalid start address on line {line_number}")
        else:
            raise ValueError(f"unsupported Intel HEX record type {record_type}")

    if not eof_seen or not memory:
        raise ValueError("Intel HEX must contain application data and an EOF record")
    if min(memory) != FLASH_START:
        raise ValueError(f"application vectors must start at 0x{FLASH_START:08x}")
    return memory


def encode_uf2(memory: dict[int, int]) -> bytes:
    blocks: dict[int, bytearray] = {}
    for address, value in memory.items():
        block_address = address & ~(UF2_PAYLOAD_SIZE - 1)
        block = blocks.setdefault(block_address, bytearray([0xFF] * UF2_PAYLOAD_SIZE))
        block[address - block_address] = value

    block_count = len(blocks)
    output = bytearray()
    for block_number, address in enumerate(sorted(blocks)):
        header = struct.pack(
            "<8I",
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            UF2_FLAG_FAMILY_ID,
            address,
            UF2_PAYLOAD_SIZE,
            block_number,
            block_count,
            UF2_FAMILY_ID,
        )
        output.extend(header)
        output.extend(blocks[address])
        output.extend(bytes(UF2_BLOCK_SIZE - 4 - len(header) - UF2_PAYLOAD_SIZE))
        output.extend(struct.pack("<I", UF2_MAGIC_END))
    return bytes(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="application Intel HEX file")
    parser.add_argument("output", type=Path, help="output UF2 file")
    arguments = parser.parse_args()

    try:
        memory = parse_ihex(arguments.input.read_text(encoding="ascii"))
        image = encode_uf2(memory)
        arguments.output.write_bytes(image)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    print(
        f"Wrote {len(image)} bytes in {len(image) // UF2_BLOCK_SIZE} UF2 blocks "
        f"for flash starting at 0x{FLASH_START:08x}: {arguments.output}"
    )


if __name__ == "__main__":
    main()
