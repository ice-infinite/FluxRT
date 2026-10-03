import json
import struct
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "protocol" / "native" / "schema-v1.json"
GOLDEN = ROOT / "protocol" / "native" / "golden-v1.json"


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (0x82F63B78 & mask)) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


class NativeProtocolContractTests(unittest.TestCase):
    def test_schema_sizes_and_message_ids_are_unique(self):
        schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        self.assertEqual(schema["magic_hex"], "4652")
        self.assertEqual(schema["header_size"], 20)
        self.assertEqual(schema["trailer_size"], 4)
        self.assertEqual(schema["maximum_frame_size"], 280)
        values = list(schema["message_types"].values())
        self.assertEqual(len(values), len(set(values)))

    def test_discovery_golden_decodes_and_crc_matches(self):
        golden = json.loads(GOLDEN.read_text(encoding="utf-8"))["vectors"][0]
        frame = bytes.fromhex(golden["frame_hex"])
        self.assertEqual(len(frame), 24)
        self.assertEqual(frame[:2], b"FR")
        version, header_size = frame[2], frame[3]
        fields = struct.unpack_from("<HHHHHHI", frame, 4)
        flags, message_type, source, destination, axis, length, sequence = fields
        self.assertEqual(version, 1)
        self.assertEqual(header_size, 20)
        self.assertEqual(
            (flags, message_type, source, destination, axis, length, sequence),
            (1, 1, 42, 65535, 65535, 0, 0x78563412),
        )
        self.assertEqual(struct.unpack_from("<I", frame, 20)[0], crc32c(frame[:20]))

    def test_every_single_bit_mutation_breaks_crc_or_header(self):
        frame = bytearray.fromhex(
            json.loads(GOLDEN.read_text(encoding="utf-8"))["vectors"][0]["frame_hex"]
        )
        for byte_index in range(len(frame)):
            for bit in range(8):
                mutated = bytearray(frame)
                mutated[byte_index] ^= 1 << bit
                header_valid = (
                    mutated[:2] == b"FR"
                    and mutated[2] == 1
                    and mutated[3] == 20
                    and struct.unpack_from("<H", mutated, 14)[0] == 0
                )
                checksum_valid = (
                    struct.unpack_from("<I", mutated, 20)[0] == crc32c(mutated[:20])
                )
                self.assertFalse(header_valid and checksum_valid)


if __name__ == "__main__":
    unittest.main()
