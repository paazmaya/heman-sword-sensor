import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import hex_to_uf2


def ihex_record(address: int, record_type: int, data: bytes = b"") -> str:
    record = bytes((len(data), address >> 8, address & 0xFF, record_type)) + data
    checksum = (-sum(record)) & 0xFF
    return ":" + (record + bytes((checksum,))).hex().upper()


class HexToUf2Tests(unittest.TestCase):
    def test_encodes_application_data_with_nrf52840_family(self) -> None:
        contents = "\n".join(
            (
                ihex_record(0, 4, b"\x00\x02"),
                ihex_record(0x7000, 0, b"\x01\x02\x03\x04"),
                ihex_record(0, 1),
            )
        )

        memory = hex_to_uf2.parse_ihex(contents)
        image = hex_to_uf2.encode_uf2(memory)
        header = struct.unpack("<8I", image[:32])

        self.assertEqual(len(image), 512)
        self.assertEqual(header[:4], (0x0A324655, 0x9E5D5157, 0x2000, 0x27000))
        self.assertEqual(header[4:], (256, 0, 1, 0xADA52840))
        self.assertEqual(image[32:36], b"\x01\x02\x03\x04")
        self.assertEqual(struct.unpack("<I", image[-4:])[0], 0x0AB16F30)

    def test_rejects_bad_checksum(self) -> None:
        with self.assertRaisesRegex(ValueError, "checksum"):
            hex_to_uf2.parse_ihex(":020000040002F9\n:00000001FF\n")

    def test_rejects_data_outside_application_flash(self) -> None:
        contents = "\n".join(
            (
                ihex_record(0, 4, b"\x00\x02"),
                ihex_record(0x6000, 0, b"\x00"),
                ihex_record(0, 1),
            )
        )

        with self.assertRaisesRegex(ValueError, "outside application flash"):
            hex_to_uf2.parse_ihex(contents)

    def test_rejects_data_in_seeed_bootloader_reservation(self) -> None:
        contents = "\n".join(
            (
                ihex_record(0, 4, b"\x00\x0E"),
                ihex_record(0xD000, 0, b"\x00"),
                ihex_record(0, 1),
            )
        )

        with self.assertRaisesRegex(ValueError, "outside application flash"):
            hex_to_uf2.parse_ihex(contents)


if __name__ == "__main__":
    unittest.main()
