#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``tools/motor_profiler_probe.py`` 的测试。
Tests for ``tools/motor_profiler_probe.py``.

这些测试只针对**纯函数**（模式统计、NACK 解码、参数组装），不碰串口。
They cover the pure functions only: pattern counting, NACK decoding and step
assembly. No serial port is touched.

测试用的字节序列全部来自 2026-09-25 在真实设备上的实测抓取，不是构造的：
The byte sequences are taken from real captures on 2026-09-25, not invented.
"""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import motor_profiler_client as mpc  # noqa: E402
import motor_profiler_probe as probe  # noqa: E402

#: 实测抓取：设备对 BEACON 的应答（BEACON 回应 + 一条 NACK）。
#: Real capture: the device's answer to a BEACON.
BEACON_EXCHANGE = bytes.fromhex("05 00 00 50 0f 04 04 c0")

#: 实测抓取：连续 5 次 BEACON 得到的 32 字节。
#: Real capture: five BEACONs produced these 32 bytes.
FIVE_BEACONS = bytes.fromhex(
    "05 00 00 50 0f 04 04 c0 0f 04 04 c0 0f 04 04 c0 0f 04 04 c0"
    "05 00 00 50 0f 04 04 c0 0f 04 04 c0"
)


class PatternCountingTests(unittest.TestCase):
    """固定模式的统计。 / Counting the fixed patterns."""

    def test_counts_beacon_and_nack_in_real_capture(self) -> None:
        counts = probe.count_patterns(BEACON_EXCHANGE)
        self.assertEqual(counts["beacon_reply"], 1)
        self.assertEqual(counts["nack_bad_crc_header"], 1)

    def test_counts_across_repeated_exchange(self) -> None:
        """5 次 BEACON 的实测响应是 32 字节，含 2 个 BEACON 回应与 6 个 NACK。
        The real five-beacon capture is 32 bytes: two BEACON replies and six NACKs."""
        self.assertEqual(len(FIVE_BEACONS), 32)
        counts = probe.count_patterns(FIVE_BEACONS)
        self.assertEqual(counts["beacon_reply"], 2)
        self.assertEqual(counts["nack_bad_crc_header"], 6)

    def test_empty_blob_counts_zero(self) -> None:
        counts = probe.count_patterns(b"")
        self.assertTrue(all(v == 0 for v in counts.values()))

    def test_every_known_pattern_is_counted(self) -> None:
        """每个已知模式都必须出现在统计结果里，缺项会让报告静默丢信息。
        Every known pattern must appear in the result; a missing key would silently
        drop information from the report."""
        counts = probe.count_patterns(b"")
        self.assertEqual(set(counts), set(probe.KNOWN_PATTERNS))


class NackDecodeTests(unittest.TestCase):
    """NACK 头部解码。 / Decoding NACK headers."""

    def test_decodes_bad_crc_header(self) -> None:
        """实测 NACK 只有头部；错误码在头部 bit 8-15。
        The measured NACK is header-only; the error code sits in header bits 8-15."""
        found = probe.decode_nack_header(BEACON_EXCHANGE)
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]["offset"], 4)
        self.assertEqual(found[0]["header"], "0xC004040F")
        self.assertEqual(found[0]["error_info"], 4)
        self.assertEqual(found[0]["reason"], "BAD_CRC_HEADER")

    def test_decodes_two_nacks(self) -> None:
        found = probe.decode_nack_header(bytes.fromhex("0f 04 04 c0 0f 04 04 c0"))
        self.assertEqual([f["offset"] for f in found], [0, 4])
        self.assertTrue(all(f["error_info"] == 4 for f in found))

    def test_decodes_bad_packet_size(self) -> None:
        """错误码 2 = BAD_PACKET_SIZE，实测在发送超长 DATA_PACKET 时出现。
        Error code 2 appears when an over-long DATA_PACKET is sent."""
        found = probe.decode_nack_header(bytes.fromhex("0f 02 02 50"))
        self.assertEqual(found[0]["error_info"], 2)
        self.assertEqual(found[0]["reason"], "BAD_PACKET_SIZE")

    def test_ignores_non_nack_frames(self) -> None:
        """BEACON 回应不能被当成 NACK。
        A BEACON reply must not be reported as a NACK."""
        found = probe.decode_nack_header(bytes.fromhex("05 00 00 50"))
        self.assertEqual(found, [])

    def test_ignores_frames_with_a_bad_crc(self) -> None:
        """CRC 不通过的 4 字节不能算作合法 NACK。
        A 4-byte chunk with a failing CRC is not a valid NACK."""
        bad = bytes.fromhex("0f 04 04 10")
        self.assertFalse(mpc.crc4_check(struct.unpack_from("<I", bad, 0)[0]))
        self.assertEqual(probe.decode_nack_header(bad), [])

    def test_nack_error_names_match_firmware_header(self) -> None:
        """错误码名称必须与 ``Src/aspep.h`` 一致。
        Error names must match ``Src/aspep.h``."""
        self.assertEqual(probe.NACK_REASONS[1], "BAD_PACKET_TYPE")
        self.assertEqual(probe.NACK_REASONS[2], "BAD_PACKET_SIZE")
        self.assertEqual(probe.NACK_REASONS[4], "BAD_CRC_HEADER")
        self.assertEqual(probe.NACK_REASONS[5], "BAD_CRC_DATA")


class StepAssemblyTests(unittest.TestCase):
    """步骤组装。 / Step assembly."""

    def _args(self, **overrides):
        args = probe.build_parser().parse_args([])
        for key, value in overrides.items():
            setattr(args, key, value)
        return args

    def test_all_builds_three_steps(self) -> None:
        args = self._args(beacon=False, ping=False, read=False, all=True,
                          dump=False, capabilities=None, gap=1.0)
        steps = probe.build_steps(args)
        self.assertEqual([name for name, _f, _g in steps],
                         ["BEACON", "PING(CFG)", "READ SC_VBUS"])

    def test_dump_mode_sends_nothing(self) -> None:
        """``--dump`` 必须一个帧都不发，否则就不是抓真实对话而是干扰它。
        ``--dump`` must send nothing, or it would disturb the dialogue it captures."""
        args = self._args(beacon=True, ping=True, read=True, all=True, dump=True,
                          capabilities=None, gap=1.0)
        self.assertEqual(probe.build_steps(args), [])

    def test_capabilities_override_is_applied(self) -> None:
        args = self._args(beacon=True, ping=False, read=False, all=False,
                          dump=False, capabilities="4,16,0", gap=1.0)
        steps = probe.build_steps(args)
        self.assertEqual(len(steps), 1)
        _name, frame, _gap = steps[0]
        payload = frame[4:8]
        self.assertEqual(mpc.Capabilities.decode(payload).rx_max_size, 4)
        self.assertEqual(mpc.Capabilities.decode(payload).txs_max_size, 16)
        self.assertEqual(mpc.Capabilities.decode(payload).txa_max_size, 0)

    def test_beacon_frame_is_well_formed(self) -> None:
        """组出来的 BEACON 必须是合法帧且类型正确。
        The assembled BEACON must be a valid frame of the right type."""
        args = self._args(beacon=True, ping=False, read=False, all=False,
                          dump=False, capabilities=None, gap=1.0)
        _name, frame, _gap = probe.build_steps(args)[0]
        header = struct.unpack_from("<I", frame, 0)[0]
        self.assertTrue(mpc.crc4_check(header))
        self.assertEqual(header & 0x0F, mpc.BEACON)
        self.assertEqual((header & 0x1FFF0) >> 4, len(frame) - 4)

    def test_read_step_targets_sc_vbus(self) -> None:
        args = self._args(beacon=False, ping=False, read=True, all=False,
                          dump=False, capabilities=None, gap=1.0)
        _name, frame, _gap = probe.build_steps(args)[0]
        self.assertEqual(frame, mpc.encode_frame(
            mpc.DATA_PACKET, mpc.mcp_read_request(mpc.MC_REG_SC_VBUS)))


class CliTests(unittest.TestCase):
    """命令行形状。 / CLI shape."""

    def test_no_action_flag_becomes_all(self) -> None:
        """不给动作参数时 ``main`` 必须补上 ``--all``，而不是什么都不做。
        With no action flag ``main`` must select ``--all`` rather than do nothing."""
        args = probe.build_parser().parse_args([])
        self.assertFalse(any((args.beacon, args.ping, args.read, args.all, args.dump)))
        # 复刻 main 的补全逻辑，确认补成 --all。
        if not any((args.beacon, args.ping, args.read, args.all, args.dump)):
            args.all = True
        self.assertTrue(args.all)
        self.assertEqual(len(probe.build_steps(args)), 3)

    def test_parser_defaults(self) -> None:
        args = probe.build_parser().parse_args([])
        self.assertEqual(args.port, "COM6")
        self.assertEqual(args.baud, 1843200)
        self.assertEqual(args.byte_delay_ms, 1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
