import json
import math
import struct
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "protocol" / "native" / "payload-schema-v1.json"
GOLDEN = ROOT / "protocol" / "native" / "payload-golden-v1.json"


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            mask = -(crc & 1) & 0xFFFFFFFF
            crc = ((crc >> 1) ^ (0x82F63B78 & mask)) & 0xFFFFFFFF
    return (~crc) & 0xFFFFFFFF


class NativePayloadContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
        self.vectors = {
            item["name"]: item
            for item in json.loads(GOLDEN.read_text(encoding="utf-8"))["vectors"]
        }

    def test_schema_is_fixed_width_and_forbids_memory_copy(self):
        self.assertEqual(self.schema["byte_order"], "little-endian")
        self.assertEqual(self.schema["rules"]["float_format"], "IEEE-754-binary32")
        self.assertTrue(self.schema["rules"]["struct_memory_copy_forbidden"])
        expected = {
            "identity": (48, 12),
            "capabilities": (64, 16),
            "status": (64, 16),
            "command_result": (32, 8),
            "product_command": (104, 26),
        }
        for name, (size, fields) in expected.items():
            payload = self.schema["payloads"][name]
            self.assertEqual(payload["size"], size)
            self.assertEqual(len(payload["fields"]), fields)
            self.assertEqual(size, fields * 4)

    def test_every_framed_payload_matches_header_length_crc_and_embedded_payload(self):
        for vector in self.vectors.values():
            if "frame_hex" not in vector:
                continue
            frame = bytes.fromhex(vector["frame_hex"])
            payload = bytes.fromhex(vector["payload_hex"])
            self.assertEqual(frame[:4], b"FR\x01\x14")
            self.assertEqual(struct.unpack_from("<H", frame, 6)[0], vector["message_type"])
            payload_length = struct.unpack_from("<H", frame, 14)[0]
            self.assertEqual(payload_length, len(payload))
            self.assertEqual(frame[20 : 20 + payload_length], payload)
            self.assertEqual(len(frame), 24 + payload_length)
            self.assertEqual(
                struct.unpack_from("<I", frame, 20 + payload_length)[0],
                crc32c(frame[: 20 + payload_length]),
            )

    def test_identity_capabilities_status_and_result_values_are_unambiguous(self):
        identity = struct.unpack("<12I", bytes.fromhex(self.vectors["identity-response"]["payload_hex"]))
        self.assertEqual(identity, (1, 0x10000, 0x150000, 0x40000, 0x43116001,
                                    0x28041007, 0x160001, 7, 0x20602,
                                    0x12345678, 1, 1))
        capabilities = struct.unpack(
            "<16I", bytes.fromhex(self.vectors["capabilities-response"]["payload_hex"])
        )
        self.assertEqual(capabilities[13:16], (256, 10, 0))
        status = struct.unpack(
            "<12I4f", bytes.fromhex(self.vectors["status-response"]["payload_hex"])
        )
        self.assertEqual(status[:12], (1, 3, 12345, 7, 0, 1, 0, 0, 0, 0, 0, 0x800))
        self.assertTrue(math.isclose(status[12], 12.3, rel_tol=0.0, abs_tol=1e-6))
        self.assertEqual(status[13:], (0.0, 0.0, 0.0))
        result = struct.unpack(
            "<8I", bytes.fromhex(self.vectors["read-only-command-result"]["payload_hex"])
        )
        self.assertEqual(result, (1, 9, 55, 0, 8, 0, 0, 0))

    def test_product_command_uses_declared_fields_not_native_padding(self):
        payload = bytes.fromhex(self.vectors["release-product-command"]["payload_hex"])
        words = struct.unpack("<13I13f", payload)
        self.assertEqual(words[:13], (104, 1, 0, 55, 4, 100, 120, 0, 0, 0, 0, 0, 0))
        self.assertEqual(words[13:], (0.0,) * 13)


if __name__ == "__main__":
    unittest.main()
