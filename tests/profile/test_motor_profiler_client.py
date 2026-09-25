#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``tools/motor_profiler_client.py`` 的离线测试。
Offline tests for ``tools/motor_profiler_client.py``.

这些测试**完全不碰串口和硬件**，只验证协议实现本身。协议一旦写错，实机表现是
"连不上"或"读到垃圾值"，很难定位；所以在接线之前必须先把帧格式钉死。

These tests never touch a serial port. A wrong frame format shows up on hardware
only as "cannot connect" or "garbage values", which is expensive to debug, so the
framing is pinned down here first.

重点覆盖 / What is covered:

- CRC-4 查表与固件逐值一致，且"含 CRC 的头部校验结果为 0"这条自洽性成立
- 头部位域与 ``aspep.c`` 的 TX/RX 两侧提取表达式一致
- BEACON / PING / NACK 编解码往返
- 握手状态机（含 ``ASPEP_CheckBeacon`` 的严格相等判定与 DATA_CRC 归零）
- MCP 寄存器读写与 float/u16/string 解码
- 母线电压阈值伏特↔数字量换算
"""

from __future__ import annotations

import hashlib
import json
import struct
import time
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import motor_profiler_client as mpc  # noqa: E402


def encode_ping_frame(packet_number: int = 0) -> bytes:
    """构造一个完整的 PING 帧，供假设备的直接状态测试使用。
    Build a complete PING frame for the fake performer's direct state tests."""
    ping = mpc.Ping(c_bit=1, n_bit=0, ip_id=0, packet_number=packet_number)
    return mpc.encode_frame(mpc.PING, ping.encode(), packet_number=packet_number)


class Crc4Tests(unittest.TestCase):
    """CRC-4 与固件的一致性。 / CRC-4 agreement with the firmware."""

    def test_lookup_tables_have_official_shape(self) -> None:
        """两张查表必须是 256 / 16 项且值域 0..15。
        Both tables must have 256 / 16 entries with values in 0..15."""
        self.assertEqual(len(mpc.CRC4_LOOKUP8), 256)
        self.assertEqual(len(mpc.CRC4_LOOKUP4), 16)
        self.assertTrue(all(0 <= v <= 0x0F for v in mpc.CRC4_LOOKUP8))
        self.assertTrue(all(0 <= v <= 0x0F for v in mpc.CRC4_LOOKUP4))

    def test_lookup8_first_row_matches_source(self) -> None:
        """抽查 aspep.c 里第一行与最后一行，防止抄表时整体错位。
        Spot-check the first and last rows of the table as written in aspep.c."""
        self.assertEqual(
            list(mpc.CRC4_LOOKUP8[0:16]),
            [0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
             0x07, 0x05, 0x03, 0x01, 0x0F, 0x0D, 0x0B, 0x09],
        )
        self.assertEqual(
            list(mpc.CRC4_LOOKUP8[240:256]),
            [0x03, 0x01, 0x07, 0x05, 0x0B, 0x09, 0x0F, 0x0D,
             0x04, 0x06, 0x00, 0x02, 0x0C, 0x0E, 0x08, 0x0A],
        )
        self.assertEqual(
            list(mpc.CRC4_LOOKUP4),
            [0x00, 0x07, 0x0E, 0x09, 0x0B, 0x0C, 0x05, 0x02,
             0x01, 0x06, 0x0F, 0x08, 0x0A, 0x0D, 0x04, 0x03],
        )

    def test_tables_match_official_source_byte_for_byte(self) -> None:
        """两张查表必须与官方 ``Src/aspep.c`` 的 ``CRC4_Lookup8`` / ``CRC4_Lookup4``
        逐字节一致。
        Both tables must match ``CRC4_Lookup8`` / ``CRC4_Lookup4`` in the official
        ``Src/aspep.c`` byte for byte.

        手抄 256 项查表极易出错，而且**单看数值发现不了**：一个抄错的 CRC 表只会
        表现为"实机连不上"。所以这里用整表指纹钉死。指纹是用官方源文件解析出的
        256 字节算出来的（解析脚本删注释后提取所有 ``0xNN`` 字面量）。
        A mis-transcribed CRC table shows up on hardware only as "cannot connect",
        so the whole table is pinned by fingerprint. The digests come from parsing
        the official source (comments stripped, all ``0xNN`` literals extracted).
        """
        self.assertEqual(
            hashlib.sha256(bytes(mpc.CRC4_LOOKUP8)).hexdigest(),
            "f5e816d1fc53e9141b69eefaecfa4c7fa2779fa2d955d5bcb4765ed9ba52e108",
            "CRC4_LOOKUP8 与官方 aspep.c 不一致 / byte table differs from aspep.c",
        )
        self.assertEqual(
            hashlib.sha256(bytes(mpc.CRC4_LOOKUP4)).hexdigest(),
            "7b25a3469ea4878899fd2f70f7f8c230787084f979af847bc291566b95f62458",
            "CRC4_LOOKUP4 与官方 aspep.c 不一致 / nibble table differs from aspep.c",
        )

    def test_byte_table_is_composed_from_nibble_table(self) -> None:
        """256 项表必须能由 16 项表按固件的查表语义复算出来。
        The 256-entry table must be reproducible from the 16-entry one using the
        firmware's lookup semantics.

        查表语义是 ``新crc = TABLE[旧crc ^ 输入]``。固件注释说明字节表是把 nibble
        从最低位当作被除数高位处理，实测对应的复算规则是：
        ``byte_table[b] = N[N[0 ^ 低nibble] ^ 高nibble]`` —— **低 nibble 先入**。
        这与源码注释一致；反过来（先高后低）只能匹配 32/256。
        The lookup semantics are ``new_crc = TABLE[old_crc ^ input]``. The byte table
        is ``N[N[0 ^ low_nibble] ^ high_nibble]`` — low nibble first, matching the
        source comment. The reverse order matches only 32/256.
        """
        nibble = list(mpc.CRC4_LOOKUP4)
        rebuilt = [
            nibble[nibble[byte & 0x0F] ^ ((byte >> 4) & 0x0F)]
            for byte in range(256)
        ]
        self.assertEqual(
            rebuilt, list(mpc.CRC4_LOOKUP8),
            "字节表无法由 nibble 表复算 / byte table is not the nibble composition",
        )

    def test_header_with_crc_checks_to_zero(self) -> None:
        """含 CRC 的头部再校验必须回 0，这是固件的判定方式。
        A header carrying its CRC must verify to zero, which is how the
        firmware decides validity."""
        for packet_type in (mpc.DATA_PACKET, mpc.PING, mpc.BEACON, mpc.NACK):
            for length in (0, 1, 4, 16, 255, 8191):
                header = mpc.build_header(packet_type, length)
                self.assertTrue(
                    mpc.crc4_check(header),
                    f"CRC 自洽失败 / self-check failed: type={packet_type} len={length}",
                )

    def test_corrupted_header_is_rejected(self) -> None:
        """翻转任一数据位都必须让校验失败。
        Flipping any data bit must break verification."""
        header = mpc.build_header(mpc.DATA_PACKET, 40)
        for bit in range(28):
            self.assertFalse(
                mpc.crc4_check(header ^ (1 << bit)),
                f"翻转 bit {bit} 后仍通过校验 / corruption at bit {bit} not detected",
            )


class HeaderFieldTests(unittest.TestCase):
    """头部位域必须与 aspep.c 的提取表达式一致。
    Header fields must match the extraction expressions in aspep.c."""

    def test_length_field_matches_firmware_expression(self) -> None:
        """复刻 aspep.c:858 的 ``(hdr & 0x1FFF0) >> 4``。
        Reproduce ``(hdr & 0x1FFF0) >> 4`` from aspep.c."""
        for length in (0, 1, 2, 39, 255, 1000, 8191):
            header = mpc.build_header(mpc.DATA_PACKET, length)
            self.assertEqual((header & 0x1FFF0) >> 4, length)
            self.assertEqual(mpc.parse_header(header)["payload_len"], length)

    def test_packet_type_is_low_nibble(self) -> None:
        """复刻 aspep.c:852 的 ``rxHeader[0] & 0xF``。
        Reproduce ``rxHeader[0] & 0xF`` from aspep.c."""
        for packet_type in (mpc.DATA_PACKET, mpc.PING, mpc.BEACON, mpc.NACK):
            header = mpc.build_header(packet_type, 8)
            self.assertEqual(header & 0xF, packet_type)
            self.assertEqual(mpc.parse_header(header)["packet_type"], packet_type)

    def test_packet_number_field_matches_firmware_expression(self) -> None:
        """复刻 aspep.c:713 的 ``(hdr & 0x0FFFF000) >> 12``。
        Reproduce ``(hdr & 0x0FFFF000) >> 12`` from aspep.c."""
        for number in (0, 1, 0x1234, 0xFFFF):
            header = mpc.build_header(mpc.PING, 4, packet_number=number)
            self.assertEqual((header & 0x0FFFF000) >> 12, number)
            self.assertEqual(mpc.parse_header(header)["packet_number"], number)

    def test_header_is_little_endian_on_the_wire(self) -> None:
        """帧头按小端上线，与 ``struct.pack("<I", header)`` 一致。
        The header goes out little-endian."""
        payload = bytes(range(0x2B))
        frame = mpc.encode_frame(mpc.DATA_PACKET, payload)
        # 用**同一负载长度**独立构造头部再比字节，避免拿空负载自证。
        # Build the header independently from the same payload length.
        header = mpc.build_header(mpc.DATA_PACKET, len(payload))
        self.assertEqual(frame[:4], struct.pack("<I", header))
        self.assertEqual((header & 0x1FFF0) >> 4, 0x2B)
        self.assertEqual(frame[4:], payload)

    def test_length_beyond_13_bits_is_rejected(self) -> None:
        """超过 13 位长度必须拒绝而不是静默截断。
        Over-long payloads must be rejected, not silently truncated."""
        with self.assertRaises(ValueError):
            mpc.build_header(mpc.DATA_PACKET, 0x2000)

    def test_parse_rejects_bad_crc(self) -> None:
        """坏 CRC 的头必须被 parse_header 拒绝。
        parse_header must reject a corrupted header."""
        header = mpc.build_header(mpc.DATA_PACKET, 12)
        with self.assertRaises(ValueError):
            mpc.parse_header(header ^ 0xDEADBEEF)


class ControlFrameTests(unittest.TestCase):
    """BEACON / PING / NACK 编解码往返。 / Control-frame round trips."""

    def test_beacon_round_trip(self) -> None:
        caps = mpc.Capabilities(version=2, data_crc=0, rx_max_size=3,
                                txs_max_size=16, txa_max_size=8)
        decoded = mpc.Capabilities.decode(caps.encode())
        self.assertEqual(decoded, caps)

    def test_beacon_bit_layout_matches_firmware(self) -> None:
        """复刻 aspep.c:218-223 的 BEACON 组包表达式。
        Reproduce the BEACON packing expression from aspep.c."""
        caps = mpc.Capabilities(version=1, data_crc=1, rx_max_size=5,
                                txs_max_size=9, txa_max_size=17)
        value = struct.unpack("<I", caps.encode())[0]
        expected = (
            (mpc.BEACON & 0xF)
            | (1 << 4)
            | (1 << 7)
            | (5 << 8)
            | (9 << 14)
            | (17 << 21)
        )
        self.assertEqual(value, expected)
        # 复刻 aspep.c:361-365 的解包表达式。
        self.assertEqual((value & 0x70) >> 4, 1)
        self.assertEqual(value >> 7 & 0x1, 1)  # rxHeader[0] >> 7
        self.assertEqual((value >> 8) & 0x3F, 5)
        self.assertEqual((value & 0x01FC000) >> 14, 9)
        self.assertEqual((value & 0xFE00000) >> 21, 17)

    def test_ping_round_trip(self) -> None:
        ping = mpc.Ping(c_bit=1, n_bit=0, ip_id=3, packet_number=0x0ABC)
        decoded = mpc.Ping.decode(ping.encode())
        self.assertEqual(decoded.c_bit, 1)
        self.assertEqual(decoded.n_bit, 0)
        self.assertEqual(decoded.ip_id, 3)
        self.assertEqual(decoded.packet_number, 0x0ABC)

    def test_nack_round_trip(self) -> None:
        for error_info in (1, 2, 4, 5):
            decoded = mpc.Nack.decode(mpc.Nack(error_info).encode())
            self.assertEqual(decoded.error_info, error_info)

    def test_control_frames_are_four_bytes(self) -> None:
        """控制负载恒为 ASPEP_CTRL_SIZE=4 字节。
        Control payloads are always ASPEP_CTRL_SIZE = 4 bytes."""
        self.assertEqual(len(mpc.Capabilities().encode()), mpc.ASPEP_CTRL_SIZE)
        self.assertEqual(len(mpc.Ping().encode()), mpc.ASPEP_CTRL_SIZE)
        self.assertEqual(len(mpc.Nack().encode()), mpc.ASPEP_CTRL_SIZE)


class McpPayloadTests(unittest.TestCase):
    """MCP 负载编码。 / MCP payload encoding."""

    def test_read_request_layout(self) -> None:
        """读请求 = 命令码 + element ID，共 4 字节。
        A read request is command + element id, 4 bytes."""
        payload = mpc.mcp_read_request(mpc.MC_REG_SC_RS)
        self.assertEqual(len(payload), 4)
        command, element = struct.unpack("<HH", payload)
        self.assertEqual(command, mpc.GET_DATA_ELEMENT)
        # element ID 必须原样带上寄存器号与 32 位类型编码。
        # 这里刻意**不**套 REG_MASK：固件是用 REG_MASK 从完整 element ID 里抽
        # 寄存器号的，组包时套它会连类型位一起抹掉。
        self.assertEqual(element & mpc.REG_MASK, mpc.MC_REG_SC_RS & mpc.REG_MASK)
        self.assertEqual(element & mpc.TYPE_MASK, mpc.TYPE_DATA_32BIT)
        self.assertEqual(element & mpc.MOTOR_MASK, 1)
        self.assertEqual(element, mpc.MC_REG_SC_RS | 1)

    def test_register_ids_match_firmware_definitions(self) -> None:
        """逐条核对 register_interface.h 里的寄存器定义。
        Check the register definitions against register_interface.h."""
        cases = {
            mpc.MC_REG_SC_RS: (91, mpc.TYPE_DATA_32BIT),
            mpc.MC_REG_SC_LS: (92, mpc.TYPE_DATA_32BIT),
            mpc.MC_REG_SC_KE: (93, mpc.TYPE_DATA_32BIT),
            mpc.MC_REG_SC_VBUS: (94, mpc.TYPE_DATA_32BIT),
            mpc.MC_REG_SC_MEAS_NOMINALSPEED: (95, mpc.TYPE_DATA_32BIT),
            mpc.MC_REG_SC_J: (101, mpc.TYPE_DATA_32BIT),
            mpc.MC_REG_SC_F: (102, mpc.TYPE_DATA_32BIT),
            mpc.MC_REG_OVERVOLTAGETHRESHOLD: (112, mpc.TYPE_DATA_16BIT),
            mpc.MC_REG_UNDERVOLTAGETHRESHOLD: (113, mpc.TYPE_DATA_16BIT),
        }
        for element_id, (index, type_code) in cases.items():
            with self.subTest(element=element_id):
                self.assertEqual(element_id >> mpc.ELT_IDENTIFIER_POS, index)
                self.assertEqual(element_id & mpc.TYPE_MASK, type_code)

    def test_type_codes_match_firmware(self) -> None:
        """类型编码是 ``N << 3``，与 register_interface.h 一致。
        Type codes are ``N << 3``."""
        self.assertEqual(mpc.TYPE_DATA_8BIT, 0x08)
        self.assertEqual(mpc.TYPE_DATA_16BIT, 0x10)
        self.assertEqual(mpc.TYPE_DATA_32BIT, 0x18)
        self.assertEqual(mpc.TYPE_DATA_STRING, 0x20)
        self.assertEqual(mpc.TYPE_DATA_RAW, 0x28)

    def test_every_register_definition_matches_firmware(self) -> None:
        """逐个核对本模块用到的每个寄存器：寄存器号与类型位都必须与固件
        ``register_interface.h`` 的宏定义完全一致。
        Check every register this module uses: both the register number and the type
        bits must match the firmware's ``register_interface.h`` macros.

        这是防住整类 bug 的关键测试。已经真实发生过一次：``SC_NOMINAL_SPEED``
        被按 32 位读，结果只拿回一个响应码字节；根因是官方 Motor Pilot 的寄存器
        清单把它标成 ``S32``，而固件是 16 位。只测"读得通"抓不住这种错误，
        必须逐条比对类型位。
        This guards a whole bug class. It already happened once:
        ``SC_NOMINAL_SPEED`` was read as 32-bit and returned only a status byte,
        because Motor Pilot's register list says S32 while the firmware says 16-bit.
        Round-trip success alone does not catch that; the type bits must be compared.
        """
        # (元素, 期望寄存器号, 期望类型) 全部取自 register_interface.h。
        expected = {
            "MC_REG_SC_RS": (91, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_LS": (92, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_KE": (93, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_VBUS": (94, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_MEAS_NOMINALSPEED": (95, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_CURRENT": (96, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_LDLQRATIO": (98, mpc.TYPE_DATA_32BIT),
            # 固件为 16 位；Motor Pilot 清单标 S32，此处按固件。
            "MC_REG_SC_NOMINAL_SPEED": (99, mpc.TYPE_DATA_16BIT),
            "MC_REG_SC_J": (101, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_F": (102, mpc.TYPE_DATA_32BIT),
            "MC_REG_SC_MAX_CURRENT": (103, mpc.TYPE_DATA_32BIT),
            "MC_REG_OVERVOLTAGETHRESHOLD": (112, mpc.TYPE_DATA_16BIT),
            "MC_REG_UNDERVOLTAGETHRESHOLD": (113, mpc.TYPE_DATA_16BIT),
            # 以下属于官方 Motor Pilot 清单（MCPV2 profiler 寄存器集）。
            "MC_REG_SC_STATE": (16, mpc.TYPE_DATA_8BIT),
            "MC_REG_SC_STEPS": (17, mpc.TYPE_DATA_8BIT),
            "MC_REG_SC_PP": (18, mpc.TYPE_DATA_8BIT),
            "MC_REG_SC_COMPLETED": (20, mpc.TYPE_DATA_8BIT),
            "MC_REG_STATUS": (1, mpc.TYPE_DATA_8BIT),
            "MC_REG_BUS_VOLTAGE": (22, mpc.TYPE_DATA_16BIT),
            "MC_REG_RESISTOR_OFFSET": (116, mpc.TYPE_DATA_32BIT),
            "MC_REG_FAULTS_FLAGS": (0, mpc.TYPE_DATA_32BIT),
            "MC_REG_FW_NAME": (0, mpc.TYPE_DATA_STRING),
            "MC_REG_CTRL_STAGE_NAME": (1, mpc.TYPE_DATA_STRING),
            "MC_REG_PWR_STAGE_NAME": (2, mpc.TYPE_DATA_STRING),
            "MC_REG_MOTOR_NAME": (3, mpc.TYPE_DATA_STRING),
        }
        for name, (index, type_code) in expected.items():
            with self.subTest(register=name):
                element_id = getattr(mpc, name)
                self.assertEqual(
                    element_id >> mpc.ELT_IDENTIFIER_POS, index,
                    f"{name} 的寄存器号不对 / wrong register number",
                )
                self.assertEqual(
                    element_id & mpc.TYPE_MASK, type_code,
                    f"{name} 的类型位不对 / wrong type bits",
                )

    def test_nominal_speed_is_sixteen_bit(self) -> None:
        """``SC_NOMINAL_SPEED`` 必须按 16 位访问，并锁定这个官方定义冲突。
        ``SC_NOMINAL_SPEED`` must be accessed as 16-bit; pin the official conflict.

        按 32 位访问时读回的负载只有一个响应码字节，客户端会报"长度异常"，
        所以这条断言直接对应一次真实的读失败。
        Accessed as 32-bit, the payload is a single status byte and the client
        reports an odd length, which is exactly the failure that was observed.
        """
        self.assertEqual(mpc.MC_REG_SC_NOMINAL_SPEED & mpc.TYPE_MASK, mpc.TYPE_DATA_16BIT)
        self.assertNotEqual(mpc.MC_REG_SC_NOMINAL_SPEED, mpc.reg_id(99, mpc.TYPE_DATA_32BIT))
        # 真机上按 16 位能读回 1572（写进去的极对数/最高转速之一）。
        performer = mpc.FakePerformer(register_map={mpc.MC_REG_SC_NOMINAL_SPEED: 1572})
        client = mpc.MotorProfilerClient(
            port="FAKE", serial_factory=mpc.FakeSerialFactory(performer)
        )
        client.open()
        client.connect()
        try:
            self.assertEqual(client.read_u16(mpc.MC_REG_SC_NOMINAL_SPEED), 1572)
        finally:
            client.close()

    def test_command_codes_match_firmware(self) -> None:
        """命令码逐条核对 mcp.h。 / Command codes against mcp.h."""
        self.assertEqual(mpc.GET_MCP_VERSION, 0x00)
        self.assertEqual(mpc.SET_DATA_ELEMENT, 0x08)
        self.assertEqual(mpc.GET_DATA_ELEMENT, 0x10)
        self.assertEqual(mpc.START_MOTOR, 0x18)
        self.assertEqual(mpc.STOP_MOTOR, 0x20)
        self.assertEqual(mpc.STOP_RAMP, 0x28)
        self.assertEqual(mpc.START_STOP, 0x30)
        self.assertEqual(mpc.SW_RESET, 0x78)
        self.assertEqual(mpc.MCP_CMD_OK, 0x00)
        self.assertEqual(mpc.MCP_CMD_NOK, 0x01)
        self.assertEqual(mpc.CMD_MASK, 0xFFF8)
        self.assertEqual(mpc.MCP_HEADER_SIZE, 2)


class FloatDecodeTests(unittest.TestCase):
    """寄存器值解码。 / Register value decoding."""

    def test_float_decode(self) -> None:
        for value in (0.0, 5.292, -1.5, 1.058e-3, 4.964, 1e6):
            raw = struct.pack("<f", value)
            self.assertAlmostEqual(mpc.decode_float(raw), value, places=6)

    def test_u16_decode(self) -> None:
        for value in (0, 1, 8688, 18618, 65535):
            self.assertEqual(mpc.decode_u16(struct.pack("<H", value)), value)

    def test_short_buffers_raise(self) -> None:
        """负载不足必须报错，不能静默补零。
        Truncated payloads must raise rather than be zero-padded."""
        with self.assertRaises(ValueError):
            mpc.decode_float(b"\x00\x01")
        with self.assertRaises(ValueError):
            mpc.decode_u16(b"\x01")


class VbusConversionTests(unittest.TestCase):
    """母线电压换算。 / Bus-voltage conversion."""

    def test_full_scale_is_52_8_volts(self) -> None:
        """满量程 = 3.3 V / 0.0625 = 52.8 V。这正是阈值对话框标 "%" 却
        实际是伏特的原因。
        Full scale is 3.3 / 0.0625 = 52.8 V, which is why the threshold dialog's
        "percent" label is really volts."""
        self.assertAlmostEqual(mpc.VBUS_FULL_SCALE_V, 52.8, places=6)

    def test_firmware_default_thresholds(self) -> None:
        """固件默认 7 V / 15 V 对应的数字量，以及回读能否还原。
        The counts for the firmware's 7 V / 15 V defaults and their round trip."""
        under = mpc.volts_to_vbus_counts(7.0)
        over = mpc.volts_to_vbus_counts(15.0)
        self.assertEqual(under, 8688)
        self.assertEqual(over, 18618)
        self.assertAlmostEqual(mpc.vbus_counts_to_volts(under), 7.0, places=3)
        self.assertAlmostEqual(mpc.vbus_counts_to_volts(over), 15.0, places=3)

    def test_round_trip_across_range(self) -> None:
        for volts in (6.0, 7.0, 12.3, 15.0, 16.0, 24.0, 48.0):
            counts = mpc.volts_to_vbus_counts(volts)
            self.assertAlmostEqual(mpc.vbus_counts_to_volts(counts), volts, places=2)

    def test_a_percent_style_value_would_be_wrong(self) -> None:
        """把 50 当成"50%"填进去，实际会变成 50 V —— 记录这个陷阱。
        Entering 50 thinking "50 percent" actually means 50 V; pin that trap."""
        counts = mpc.volts_to_vbus_counts(50.0)
        self.assertAlmostEqual(counts, 62059, delta=1)
        self.assertGreater(mpc.vbus_counts_to_volts(counts), 45.0)


class HandshakeTests(unittest.TestCase):
    """握手状态机（用假串口，无硬件）。 / Handshake state machine, no hardware."""

    def _client(self, performer: mpc.FakePerformer) -> mpc.MotorProfilerClient:
        return mpc.MotorProfilerClient(
            port="FAKE", serial_factory=mpc.FakeSerialFactory(performer)
        )

    def test_connect_reaches_connected(self) -> None:
        performer = mpc.FakePerformer()
        with self._client(performer) as client:
            caps = client.connect()
            self.assertTrue(client.connected)
            self.assertEqual(caps.version, performer.caps.version)
            self.assertEqual(performer.state, "CONNECTED")

    def test_fake_performer_defaults_match_real_firmware(self) -> None:
        """假设备的默认能力必须等于官方 ``Src/mcp_config.c`` 里 ``aspepOverUartA``
        的真实值，否则离线测试证明的是一个不存在的设备。
        The fake performer's defaults must equal the real ``aspepOverUartA`` values,
        otherwise the offline tests prove things about a device that does not exist.

        官方定义 / official definitions:
        ``RX_maxSize = (256>>5)-1 = 7``、``TXS_maxSize = (256>>5)-1 = 7``、
        ``TXA_maxSize = 2048>>6 = 32``、``DATA_CRC = 0``、``version = 0``。
        """
        performer = mpc.FakePerformer()
        self.assertEqual(performer.caps.data_crc, 0)
        self.assertEqual(performer.caps.version, 0)
        self.assertEqual(performer.caps.rx_max_size, (256 >> 5) - 1)
        self.assertEqual(performer.caps.txs_max_size, (256 >> 5) - 1)
        self.assertEqual(performer.caps.txa_max_size, 2048 >> 6)

    def test_client_sends_first_beacon(self) -> None:
        """控制端必须先发 BEACON：固件从不主动发包，等对方先说话会永远超时。
        The controller must send the first BEACON: the firmware never speaks
        unsolicited, so waiting for it times out forever.
        """
        performer = mpc.FakePerformer()
        # 先确认假设备开局是静默的，与真实固件一致。
        performer.open()
        self.assertEqual(bytes(performer.tx), b"")
        performer.tx.clear()

        with self._client(performer) as client:
            client.connect()
        # 假设备必须收到过一个 BEACON 才可能进入 CONFIGURED。
        self.assertTrue(performer.beacon_checks)
        self.assertEqual(performer.state, "CONNECTED")

    def test_default_host_capabilities_match_firmware(self) -> None:
        """控制端默认能力必须等于固件的 ``aspepOverUartA``，否则严格相等判定
        会让连接永远谈不成。
        The controller's default capabilities must equal the firmware's
        ``aspepOverUartA``; otherwise the strict-equality check can never pass.
        """
        caps = mpc.default_host_capabilities()
        self.assertEqual(caps.version, 0)
        self.assertEqual(caps.data_crc, 0)
        self.assertEqual(caps.rx_max_size, (256 >> 5) - 1)
        self.assertEqual(caps.txs_max_size, (256 >> 5) - 1)
        self.assertEqual(caps.txa_max_size, 2048 >> 6)

    def test_client_echoes_peer_sizes_exactly(self) -> None:
        """客户端必须**原样回送** peer 的尺寸，因为固件用的是严格相等而不是取小。
        The client must echo the peer's sizes exactly: the firmware compares with
        strict equality, not a minimum.
        """
        performer = mpc.FakePerformer()
        with self._client(performer) as client:
            caps = client.connect()
        self.assertEqual(caps.rx_max_size, performer.caps.rx_max_size)
        self.assertEqual(caps.txs_max_size, performer.caps.txs_max_size)
        self.assertEqual(caps.txa_max_size, performer.caps.txa_max_size)
        self.assertEqual(performer.state, "CONNECTED")

    def test_data_crc_negotiates_to_zero(self) -> None:
        """DATA_CRC 必须为 0：官方固件在使能时的数据段 CRC 是固定常量的桩实现
        （写死 ``0xCA 0xFE``），协商成 1 会让每次数据包校验都失败。
        DATA_CRC must be 0: the official data CRC is a constant stub, so
        negotiating it to 1 would fail every data packet.

        真实固件的初始能力本来就是 0（``mcp_config.c``），所以这里同时锁定
        "客户端回送 0" 与 "固件广告 0" 两个事实。
        """
        performer = mpc.FakePerformer()
        with self._client(performer) as client:
            caps = client.connect()
        self.assertEqual(performer.caps.data_crc, 0)
        self.assertEqual(caps.data_crc, 0)

    def test_capability_mismatch_is_reported_with_reason(self) -> None:
        """能力不匹配必须让客户端报错，且假设备要能说清是哪一项不匹配。
        A capability mismatch must make the client fail, and the fake performer must
        report which field mismatched.

        注意这里测的是**假设备自身的判定逻辑**：客户端会原样回送 peer 广告的尺寸，
        所以正常情况下它总会匹配成功 —— 这本身就是 ``ASPEP_CheckBeacon`` 的设计
        （取小之后再要求严格相等，等价于要求控制端照抄）。因此这里直接构造一个
        与假设备能力不符的 BEACON 来验证判定逻辑，而不是指望客户端必然失败。
        This exercises the fake performer's own decision logic. The client echoes the
        peer's sizes, so a mismatch cannot arise from the client alone; that is the
        point of ``ASPEP_CheckBeacon`` (min, then strict equality, i.e. the
        controller must copy). The check is therefore driven directly.
        """
        performer = mpc.FakePerformer()
        # 直接投一个 TXS 大小不符的 BEACON。
        wrong = mpc.Capabilities(version=0, data_crc=0, rx_max_size=7,
                                 txs_max_size=3, txa_max_size=32)
        header = mpc.build_header(mpc.BEACON, len(wrong.encode()))
        performer.write(struct.pack("<I", header) + wrong.encode())
        self.assertEqual(performer.state, "IDLE")
        self.assertIn("txs_max_size", performer.beacon_checks[-1]["reasons"])

        # 再投一个 data_crc=1 的 BEACON：固件实际广告 0，所以必须被判为不匹配。
        bad_crc = mpc.Capabilities(version=0, data_crc=1, rx_max_size=7,
                                   txs_max_size=7, txa_max_size=32)
        header = mpc.build_header(mpc.BEACON, len(bad_crc.encode()))
        performer.write(struct.pack("<I", header) + bad_crc.encode())
        self.assertEqual(performer.state, "IDLE")
        self.assertIn("data_crc", performer.beacon_checks[-1]["reasons"])

    def test_connect_fails_when_performer_never_accepts_ping(self) -> None:
        """若 performer 因能力不符始终不接受完成连接的 PING，客户端必须报错而不是
        假装连上。
        If the performer never accepts the completing PING because the capabilities
        do not line up, the client must fail instead of pretending to be connected.
        """
        performer = mpc.FakePerformer()
        performer.reject_all_beacons = True
        performer.refuse_completing_ping = True
        with self._client(performer) as client:
            with self.assertRaises(mpc.ProfilerError):
                client.connect(timeout_s=1.0)
            self.assertFalse(client.connected)
        # 确认假设备确实判定了"拒绝"，而不是别的地方恰好超时。
        self.assertTrue(performer.beacon_checks)
        self.assertEqual(performer.beacon_checks[-1]["reasons"], ["forced_rejection"])
        self.assertEqual(performer.state, "IDLE")

    def test_connect_succeeds_when_performer_accepts_ping(self) -> None:
        """对照组：同样的假设备只要接受 PING 就能连上，证明上一个用例失败的原因
        是 PING 未被接受，而不是握手根本没走通。
        Control case: the same fake connects as soon as it accepts the PING, proving
        the previous test fails because of the PING, not because the handshake never
        ran.
        """
        performer = mpc.FakePerformer()
        with self._client(performer) as client:
            client.connect(timeout_s=2.0)
            self.assertTrue(client.connected)
        self.assertEqual(performer.state, "CONNECTED")

    def test_ping_only_connects_from_configured_state(self) -> None:
        """固件只在 CONFIGURED 状态接受 PING 完成连接；IDLE 收到 PING 不会进
        CONNECTED。这条状态转移是握手的关键，必须能被测出来。
        The firmware only accepts the completing PING while CONFIGURED; a PING in
        IDLE must not reach CONNECTED. This transition is the crux of the handshake.
        """
        # 情况一：从 IDLE 直接发 PING → 不连接。
        performer = mpc.FakePerformer()
        performer.pending_beacon_on_open = False
        performer.open()
        performer.tx.clear()
        performer.write(encode_ping_frame(packet_number=0))
        self.assertEqual(performer.state, "IDLE")

        # 情况二：先做一次成功的 BEACON 交换进入 CONFIGURED，再发 PING → 连接。
        performer = mpc.FakePerformer()
        performer.pending_beacon_on_open = False
        performer.open()
        performer.tx.clear()
        good = mpc.Capabilities(version=0, data_crc=0, rx_max_size=7,
                                txs_max_size=7, txa_max_size=32)
        performer.write(mpc.encode_frame(mpc.BEACON, good.encode()))
        self.assertEqual(performer.state, "CONFIGURED")
        performer.tx.clear()
        performer.write(encode_ping_frame(packet_number=0))
        self.assertEqual(performer.state, "CONNECTED")


class RegisterAccessTests(unittest.TestCase):
    """经假串口做寄存器读写。 / Register access through the fake port."""

    def _connected(self, register_map: dict):
        performer = mpc.FakePerformer(register_map=register_map)
        client = mpc.MotorProfilerClient(
            port="FAKE", serial_factory=mpc.FakeSerialFactory(performer)
        )
        client.open()
        client.connect()
        return client

    def test_read_float_register(self) -> None:
        client = self._connected({mpc.MC_REG_SC_RS: 5.292})
        try:
            self.assertAlmostEqual(client.read_float(mpc.MC_REG_SC_RS), 5.292, places=5)
        finally:
            client.close()

    def test_read_u16_register(self) -> None:
        client = self._connected({mpc.MC_REG_OVERVOLTAGETHRESHOLD: 18618})
        try:
            self.assertEqual(client.read_u16(mpc.MC_REG_OVERVOLTAGETHRESHOLD), 18618)
        finally:
            client.close()

    def test_read_string_register(self) -> None:
        client = self._connected({mpc.MC_REG_FW_NAME: "STM32G431"})
        try:
            self.assertEqual(client.read_string(mpc.MC_REG_FW_NAME), "STM32G431")
        finally:
            client.close()

    def test_read_results_returns_all_fields(self) -> None:
        """read_results 必须把每个识别字段都读出来且数值不串位。
        read_results must fetch every identified field without mixing them up."""
        register_map = {
            mpc.MC_REG_SC_RS: 5.29,
            mpc.MC_REG_SC_LS: 1.058e-3,
            mpc.MC_REG_SC_KE: 4.964,
            mpc.MC_REG_SC_VBUS: 12.3,
            mpc.MC_REG_SC_MEAS_NOMINALSPEED: 1572.0,
            mpc.MC_REG_SC_NOMINAL_SPEED: 1500.0,
            mpc.MC_REG_SC_LDLQRATIO: 1.0,
            mpc.MC_REG_SC_J: 2.91e-5,
            mpc.MC_REG_SC_F: 9.37e-6,
            mpc.MC_REG_SC_MAX_CURRENT: 0.8,
            mpc.MC_REG_SC_CURRENT: 0.8,
        }
        client = self._connected(register_map)
        try:
            results = client.read_results()
        finally:
            client.close()
        for key, expected in (
            ("rs_ohm", 5.29), ("ls_h", 1.058e-3), ("ke", 4.964),
            ("vbus_v", 12.3), ("meas_nominal_speed", 1572.0),
            ("inertia_j", 2.91e-5), ("friction_f", 9.37e-6),
        ):
            with self.subTest(field=key):
                self.assertAlmostEqual(results[key], expected, places=5)

    def test_read_of_missing_register_raises(self) -> None:
        """读一个固件没有的寄存器必须报错，不能返回 0。
        Reading an absent register must raise rather than return zero."""
        client = self._connected({})
        try:
            with self.assertRaises(mpc.ProfilerError):
                client.read_float(mpc.MC_REG_SC_RS)
        finally:
            client.close()

    def test_read_thresholds_converts_to_volts(self) -> None:
        client = self._connected({
            mpc.MC_REG_OVERVOLTAGETHRESHOLD: 18618,
            mpc.MC_REG_UNDERVOLTAGETHRESHOLD: 8688,
        })
        try:
            thresholds = client.read_thresholds()
        finally:
            client.close()
        self.assertAlmostEqual(thresholds["under_threshold_v"], 7.0, places=2)
        self.assertAlmostEqual(thresholds["over_threshold_v"], 15.0, places=2)

    def test_transact_before_connect_raises(self) -> None:
        """未连接就发负载必须报错，避免在半握手状态下误发命令。
        Sending before connecting must raise, so no command can slip out during a
        half-finished handshake."""
        client = mpc.MotorProfilerClient(port="FAKE",
                                        serial_factory=mpc.FakeSerialFactory())
        client.open()
        try:
            with self.assertRaises(mpc.ProfilerError):
                client.transact(mpc.mcp_command_request(mpc.STOP_MOTOR))
        finally:
            client.close()


class RecordingPort:
    """只记录 ``write`` 调用大小的假串口，用于单独验证 ``_write`` 的节奏。
    A minimal port that only records write sizes so ``_write`` can be tested in
    isolation, without a frame parser in the way."""

    def __init__(self) -> None:
        self.calls: list = []
        self.flushes = 0

    def write(self, data: bytes) -> int:
        self.calls.append(len(data))
        return len(data)

    def flush(self) -> None:
        self.flushes += 1

    def read(self, count: int = 1) -> bytes:
        return b""

    def close(self) -> None:
        pass


class ControlFrameFallbackTests(unittest.TestCase):
    """控制帧长度字段为 0 时的兜底读取。
    Fallback reading when a control frame's length field is zero.

    实测依据：官方 ``ASPEP_sendBeacon`` 会把 4 写进头部长度字段，但**真实设备发的是
    长度字段为 0 的 4 字节帧**。严格按长度读会丢掉 BEACON 的能力负载，从而永远
    谈不成握手。
    Measured basis: the official source writes 4 into the length field, but the real
    device sends a 4-byte frame with a zero length field. Reading strictly by the
    length would discard the BEACON's capability payload and the handshake could never
    complete.
    """

    def _client(self, performer: mpc.FakePerformer) -> mpc.MotorProfilerClient:
        return mpc.MotorProfilerClient(
            port="FAKE", serial_factory=mpc.FakeSerialFactory(performer)
        )

    def test_zero_length_beacon_is_read_as_four_bytes(self) -> None:
        """长度字段为 0 的 BEACON 必须按 4 字节读，能力要能解出来。
        A zero-length BEACON must be read as 4 bytes so its capabilities decode."""
        performer = mpc.FakePerformer()
        client = self._client(performer)
        client.open()
        try:
            caps = mpc.Capabilities(version=0, data_crc=0, rx_max_size=4,
                                    txs_max_size=16, txa_max_size=0)
            wire_header = mpc.build_header(mpc.BEACON, 0)
            performer.tx.extend(struct.pack("<I", wire_header) + caps.encode())
            ptype, payload = client._read_frame(
                time.monotonic() + 1.0, client._CONTROL_PAYLOAD_SIZE
            )
            self.assertEqual(ptype, mpc.BEACON)
            self.assertEqual(len(payload), 4)
            self.assertEqual(mpc.Capabilities.decode(payload), caps)
        finally:
            client.close()

    def test_zero_length_data_packet_stays_empty(self) -> None:
        """0 长度的 DATA_PACKET 是合法的空负载，**不能**兜底成 4 字节。
        A zero-length DATA_PACKET is a valid empty payload and must not be padded."""
        performer = mpc.FakePerformer()
        client = self._client(performer)
        client.open()
        try:
            performer.tx.extend(struct.pack("<I", mpc.build_header(mpc.DATA_PACKET, 0)))
            ptype, payload = client._read_frame(
                time.monotonic() + 1.0, client._CONTROL_PAYLOAD_SIZE
            )
            self.assertEqual(ptype, mpc.DATA_PACKET)
            self.assertEqual(payload, b"")
        finally:
            client.close()

    def test_best_effort_accepts_a_header_only_nack(self) -> None:
        """真实设备的 NACK 只有 4 字节头部、长度字段为 0；best_effort 必须接受它。
        The real device's NACK is header-only with a zero length field; best_effort
        must accept it.

        实测抓到的 NACK 就是 4 字节 ``0f 04 04 c0``，其头部长度字段为 0。
        The captured NACK is the 4-byte ``0f 04 04 c0``, whose length field is 0.
        """
        performer = mpc.FakePerformer()
        client = self._client(performer)
        client.open()
        try:
            header = mpc.build_header(mpc.NACK, 0)
            self.assertEqual((header & 0x1FFF0) >> 4, 0)
            performer.tx.extend(struct.pack("<I", header))
            ptype, payload = client._read_frame(
                time.monotonic() + 1.0, client._CONTROL_PAYLOAD_SIZE, best_effort=True
            )
            self.assertEqual(ptype, mpc.NACK)
            # 兜底去读 4 字节负载，但设备一个都没发，所以必须是空而不是抛异常。
            # The fallback asks for 4 bytes but the device sent none, so the result
            # must be empty rather than a raised error.
            self.assertEqual(payload, b"")
        finally:
            client.close()

    def test_captured_nack_bytes_decode_to_bad_crc_header(self) -> None:
        """实测 NACK 的原始字节本身就要能解出错误码；这条把抓包固定下来。
        The captured NACK bytes must decode to an error code; this pins the capture.

        注意实测 NACK 的**长度字段是 64 而不是 0**，但设备只发 4 字节就结束。也就是说
        真实固件既没有按源码那样填 4（控制帧负载长度），也没有把长度清零，而是把
        错误码塞进了头部的高位。所以对控制帧**不能信任长度字段**。
        Note the captured NACK's length field is 64, not 0, yet the device sends only 4
        bytes. The real firmware neither writes 4 (the source's control payload length)
        nor zeroes it; it packs the error code into the header's upper bits. The length
        field therefore cannot be trusted for control frames.
        """
        captured = bytes.fromhex("0f 04 04 c0")
        header = struct.unpack_from("<I", captured, 0)[0]
        self.assertTrue(mpc.crc4_check(header))
        self.assertEqual(header & 0x0F, mpc.NACK)
        self.assertEqual((header & 0x1FFF0) >> 4, 64, "实测长度字段为 64")
        self.assertEqual((header >> 8) & 0xFF, mpc.ASPEP_BAD_CRC_HEADER)
        # 设备实际只发了 4 字节，所以按长度字段读必然读不齐 —— 这正是 best_effort
        # 存在的理由。
        # The device sent 4 bytes total, so trusting the length field can never
        # complete, which is exactly why best_effort exists.
        self.assertLess(len(captured), 4 + ((header & 0x1FFF0) >> 4))

    def test_strict_read_still_raises_on_a_short_payload(self) -> None:
        """非 best_effort 时读不齐必须报错，不能静默返回残包。
        Without best_effort a short payload must raise, not return silently."""
        performer = mpc.FakePerformer()
        client = self._client(performer)
        client.open()
        try:
            performer.tx.extend(struct.pack("<I", mpc.build_header(mpc.DATA_PACKET, 4)) + b"\x01")
            with self.assertRaises(mpc.ProfilerError):
                client._read_frame(time.monotonic() + 0.4)
        finally:
            client.close()


class WritePacingTests(unittest.TestCase):
    """字节间延时的实测依据与实现。 / The measured basis and implementation of pacing.

    实机观察：在 1843200 baud 下把一帧 8 字节**一次写完**，目标有时处理、有时完全
    无反应；逐字节加 1 ms 间隔时回应稳定得多。这是 ST-Link VCP 的转发时序问题，
    不是波特率错误——设备能正确解出错误码就说明波特率是对的。
    Measured: writing a whole 8-byte frame in one go at 1843200 baud is sometimes
    processed and sometimes ignored; 1 ms per byte is far more reliable. This is a
    VCP forwarding effect, not a baud-rate error.
    """

    def _client_with_port(self, delay: float):
        client = mpc.MotorProfilerClient(port="FAKE", inter_byte_delay_s=delay)
        port = RecordingPort()
        client._ser = port
        return client, port

    def test_default_has_no_pacing(self) -> None:
        """库默认不延时，否则离线测试要等真实时间。
        The library defaults to no pacing, or offline tests would wait on real time."""
        self.assertEqual(mpc.MotorProfilerClient(port="FAKE").inter_byte_delay_s, 0.0)

    def test_fast_path_writes_whole_buffer_at_once(self) -> None:
        """无延时时必须一次写完，不能被拆成逐字节。
        With no delay the frame must go out in one call, not byte by byte."""
        client, port = self._client_with_port(0.0)
        client._write(b"\x01\x02\x03\x04")
        self.assertEqual(port.calls, [4])

    def test_paced_path_writes_one_byte_per_call(self) -> None:
        """设了延时时必须逐字节写、每字节 flush，且只等 n-1 个间隔。
        With pacing each byte must be its own write, each must be flushed, and there
        must be exactly n-1 gaps.

        这里统计 ``time.sleep`` 的调用而不是量墙钟：墙钟会让测试受机器负载与
        首次导入开销影响，实测出现过偶发失败。
        This counts ``time.sleep`` calls instead of measuring wall-clock time, which
        proved flaky under machine load and first-import cost.
        """
        client, port = self._client_with_port(0.001)
        sleeps: list = []
        original_sleep = mpc.time.sleep

        def recording_sleep(seconds: float) -> None:
            sleeps.append(seconds)

        mpc.time.sleep = recording_sleep  # type: ignore[assignment]
        try:
            client._write(b"\x01\x02\x03\x04")
        finally:
            mpc.time.sleep = original_sleep  # type: ignore[assignment]

        self.assertEqual(port.calls, [1, 1, 1, 1])
        self.assertEqual(port.flushes, 4)
        # 4 字节 => 3 个间隔，不是 4 个（末字节后不等待）。
        # Four bytes give three gaps, not four; no sleep after the last byte.
        self.assertEqual(sleeps, [0.001, 0.001, 0.001])

    def test_paced_path_honours_the_configured_delay(self) -> None:
        """延时值必须真的用上去，不能被写死。
        The configured delay must actually be used, not hard-coded."""
        client, port = self._client_with_port(0.0025)
        sleeps: list = []
        original_sleep = mpc.time.sleep
        mpc.time.sleep = lambda s: sleeps.append(s)  # type: ignore[assignment]
        try:
            client._write(b"\x01\x02")
        finally:
            mpc.time.sleep = original_sleep  # type: ignore[assignment]
        self.assertEqual(port.calls, [1, 1])
        self.assertEqual(sleeps, [0.0025])

    def test_single_byte_frame_never_sleeps(self) -> None:
        """单字节帧没有间隔，不能引入无谓延时。
        A one-byte frame has no gap and must not sleep."""
        client, port = self._client_with_port(0.001)
        sleeps: list = []
        original_sleep = mpc.time.sleep
        mpc.time.sleep = lambda s: sleeps.append(s)  # type: ignore[assignment]
        try:
            client._write(b"\xAA")
        finally:
            mpc.time.sleep = original_sleep  # type: ignore[assignment]
        self.assertEqual(port.calls, [1])
        self.assertEqual(sleeps, [])

    def test_pacing_works_without_a_flush_method(self) -> None:
        """串口对象没有 ``flush`` 时不能崩：假串口与部分实现确实没有。
        Pacing must not crash when the port has no ``flush``; fake ports and some
        real implementations lack it."""
        class NoFlushPort(RecordingPort):
            flush = None  # type: ignore[assignment]

        client = mpc.MotorProfilerClient(port="FAKE", inter_byte_delay_s=0.001)
        port = NoFlushPort()
        client._ser = port
        client._write(b"\x01\x02\x03\x04")
        self.assertEqual(port.calls, [1, 1, 1, 1])


class CliTests(unittest.TestCase):
    """命令行接口的基本形状。 / Basic shape of the CLI."""

    def test_parser_accepts_probe(self) -> None:
        parser = mpc.build_parser()
        args = parser.parse_args(["probe", "--simulate"])
        self.assertEqual(args.command, "probe")
        self.assertTrue(args.simulate)
        self.assertEqual(args.port, "COM6")
        self.assertEqual(args.baud, 1843200)

    def test_probe_simulation_runs_offline(self) -> None:
        """``probe --simulate`` 必须在没有硬件时跑通，并且**真的解出数值**。
        ``probe --simulate`` must run without hardware and actually decode values.

        只断言退出码是不够的：之前的版本返回全 1 字节的垃圾也能"通过"。所以这里
        把 stdout 抓下来解析 JSON，核对身份、阈值伏特数和识别结果。
        Asserting the exit code alone is too weak: an earlier version returned
        one-byte garbage and still "passed". The JSON on stdout is parsed and its
        identity, threshold volts and identified results are checked.
        """
        import contextlib
        import io

        buffer = io.StringIO()
        # 日志走 stderr，所以只抓 stdout 就能拿到纯 JSON。
        # Logs go to stderr, so capturing stdout yields pure JSON.
        with contextlib.redirect_stdout(buffer), contextlib.redirect_stderr(io.StringIO()):
            rc = mpc.main(["probe", "--simulate"])
        self.assertEqual(rc, 0)
        report = json.loads(buffer.getvalue())

        self.assertEqual(report["identity"]["controller"], "NUCLEO-G431RB")
        self.assertEqual(report["identity"]["power_board"], "X-NUCLEO-IHM16M1")
        self.assertEqual(report["identity"]["motor"], "GBM2804H-100T")
        self.assertAlmostEqual(report["thresholds"]["under_threshold_v"], 7.0, places=2)
        self.assertAlmostEqual(report["thresholds"]["over_threshold_v"], 15.0, places=2)
        self.assertAlmostEqual(report["results"]["rs_ohm"], 5.292, places=4)
        self.assertAlmostEqual(report["results"]["ls_h"], 1.058e-3, places=7)
        self.assertAlmostEqual(report["results"]["ke"], 4.964, places=4)
        self.assertAlmostEqual(report["raw"]["bus_voltage_v"], 12.3, places=2)
        self.assertEqual(report["raw"]["pole_pairs"], 7)
        self.assertEqual(report["raw"]["profiler_completed"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
