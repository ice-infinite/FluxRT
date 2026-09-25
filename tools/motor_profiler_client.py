#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ST Motor Profiler 固件的 ASPEP/MCP 串口客户端。
Serial (ASPEP/MCP) client for the ST Motor Profiler firmware.

用途 / Purpose
--------------
让 ST Motor Pilot 那套"电机参数识别"可以脱离图形界面、脚本化地跑：
连接 profiler 固件、读回识别结果（Rs / Ls / Ke / 最高转速）、按电流档位重复多次
并统计重复性，产出可直接喂给 FluxRT 参数候选流程的 JSON。

Drive the ST Motor Pilot "motor profiling" workflow without the GUI: connect to
the profiler firmware, read back the identified parameters (Rs / Ls / Ke /
max speed), repeat across current levels and report repeatability, emitting JSON
that FluxRT's parameter-candidate flow can consume.

协议来源 / Protocol provenance
------------------------------
本文件不是猜的，全部位域、掩码、CRC 查表和命令码都逐行取自官方工程源码：

- ``Src/aspep.c``、``Inc/aspep.h``   帧格式、CRC-4 查表、连接状态机
- ``Src/mcp.c``、``Inc/mcp.h``       命令码、MCP 响应码
- ``Inc/register_interface.h``       寄存器 ID 与类型编码、各类掩码
- ``Inc/parameters_conversion.h``    母线电压换算因子

工程路径：``C:\\Users\\<user>\\.st_workbench\\Projects2\\Motor_Profiler``

Every bit field, mask, CRC table and command code below was taken line by line
from the official sources listed above; nothing here is guessed.

两个必须知道的官方实现细节 / Two official quirks you must know
-------------------------------------------------------------
1. **数据段 CRC 是未实现的桩**。``aspep.c`` 在 `DATA_CRC` 使能时写入的是固定的
   ``0xCA 0xFE`` 而不是真 CRC（源码里就是 ``/* TODO : Compute real CRC*/``）。
   所以协商阶段必须把 ``DATA_CRC`` 压成 0，否则双方校验都会失败。
   The data-payload CRC is an unimplemented stub in the official firmware: it
   writes a constant ``0xCA 0xFE``. Negotiation must therefore drive
   ``DATA_CRC`` to 0.

2. **电压阈值的百分比标注是错的**。``SCC_Set*Threshold`` 收的是"数字量"，
   换算为 ``value * 65535 / 52.8``，所以界面上标着 ``%`` 的输入框实际是
   **伏特**，量程 0–52.8 V。``52.8 = ADC_REFERENCE_VOLTAGE(3.3) /
   VBUS_PARTITIONING_FACTOR(0.0625)`` 是 ADC 满量程电压，不是母线电压。
   The threshold dialog's ``%`` label is wrong: the value is volts on a
   0–52.8 V scale (the ADC full-scale, not the bus voltage).

安全 / Safety
-------------
识别过程中电机会被开环强拖转动。本模块**只在显式传入 ``--allow-motor-run``
时**才会下发启动命令；默认所有会驱动功率级的操作都被拒绝。
Profiling spins the motor open-loop. This module only issues start commands when
``--allow-motor-run`` is passed explicitly.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Optional, Sequence

# ---------------------------------------------------------------------------
# ASPEP 常量 / ASPEP constants  —— 来源 Inc/aspep.h
# ---------------------------------------------------------------------------

ASPEP_HEADER_SIZE = 4
ASPEP_CTRL_SIZE = 4
ASPEP_DATACRC_SIZE = 2

ID_MASK = 0xF
DATA_PACKET = 0x9
PING = 0x6
BEACON = 0x5
NACK = 0xF
ACK = 0xA

# 头部位域 / Header bit fields（TX 与 RX 两侧互相印证）
#   bits  0..3   包类型 packet type
#   bits  4..16  负载长度，13 位 payload length, 13 bits
#   bits 17..27  保留 / 包序号 reserved / packet number
#   bits 28..31  CRC-4（覆盖低 28 位）CRC-4 over the low 28 bits
HDR_TYPE_SHIFT = 0
HDR_LEN_SHIFT = 4
HDR_LEN_MASK = 0x1FFF
HDR_PKTNUM_SHIFT = 12
HDR_PKTNUM_MASK = 0x0FFFF000
HDR_CRC_SHIFT = 28

# 握手能力位域 / Beacon capability bit fields —— 来源 Src/aspep.c
BEACON_VER_SHIFT = 4
BEACON_DATACRC_SHIFT = 7
BEACON_RXMAX_SHIFT = 8
BEACON_TXSMAX_SHIFT = 14
BEACON_TXAMAX_SHIFT = 21

ASPEP_PING_RESET = 0
ASPEP_PING_CFG = 1

# 同步 / 异步标志即包类型 / sync-async flag IS the packet type
MCTL_SYNC = DATA_PACKET
MCTL_ASYNC = BEACON
MCTL_CTRL = 0

# ---------------------------------------------------------------------------
# MCP 常量 / MCP constants —— 来源 Inc/mcp.h、Inc/register_interface.h
# ---------------------------------------------------------------------------

MCP_VERSION = 0x1
MCP_HEADER_SIZE = 2
CMD_MASK = 0xFFF8

GET_MCP_VERSION = 0x0
SET_DATA_ELEMENT = 0x8
GET_DATA_ELEMENT = 0x10
START_MOTOR = 0x18
STOP_MOTOR = 0x20
STOP_RAMP = 0x28
START_STOP = 0x30
SW_RESET = 0x78

MCP_CMD_OK = 0x00
MCP_CMD_NOK = 0x01
MCP_ERROR_BAD_DATA_TYPE = 0x07
MCP_ERROR_BAD_RAW_FORMAT = 0x0A

# 寄存器 ID 编码 / Element-ID encoding
ELT_IDENTIFIER_POS = 6
TYPE_POS = 3
TYPE_MASK = 0x38
MOTOR_MASK = 0x7
REG_MASK = 0xFFF8

TYPE_DATA_SEG_END = 0 << TYPE_POS
TYPE_DATA_8BIT = 1 << TYPE_POS
TYPE_DATA_16BIT = 2 << TYPE_POS
TYPE_DATA_32BIT = 3 << TYPE_POS
TYPE_DATA_STRING = 4 << TYPE_POS
TYPE_DATA_RAW = 5 << TYPE_POS
TYPE_DATA_FLAG = 6 << TYPE_POS
TYPE_DATA_SEG_BEG = 7 << TYPE_POS

MCP_ID_SIZE = 2


def reg_id(identifier: int, type_code: int) -> int:
    """把"寄存器号 + 类型编码"拼成 16 位 element ID。
    Compose the 16-bit element ID from a register number and a type code."""
    return (identifier << ELT_IDENTIFIER_POS) | type_code


# Profiler 结果与配置寄存器 / Profiler result and configuration registers
# 来源 Inc/register_interface.h。float 型寄存器一律用 TYPE_DATA_32BIT。
MC_REG_SC_RS = reg_id(91, TYPE_DATA_32BIT)
MC_REG_SC_LS = reg_id(92, TYPE_DATA_32BIT)
MC_REG_SC_KE = reg_id(93, TYPE_DATA_32BIT)
MC_REG_SC_VBUS = reg_id(94, TYPE_DATA_32BIT)
MC_REG_SC_MEAS_NOMINALSPEED = reg_id(95, TYPE_DATA_32BIT)
MC_REG_SC_CURRENT = reg_id(96, TYPE_DATA_32BIT)
MC_REG_SC_LDLQRATIO = reg_id(98, TYPE_DATA_32BIT)
# 注意：``SC_NOMINAL_SPEED`` 的类型在两处官方来源里**不一致**。
# 固件 ``register_interface.h`` 写的是 ``TYPE_DATA_16BIT``，而 Motor Pilot 的
# ``RegListProfiler_MCPV2.json`` 把它标成 ``S32``。以固件为准（16 位）：
# 实测按 16 位读回 ``6352`` 得到正常值，按 32 位读回 ``6360`` 只会拿到一个响应码。
# NOTE: ``SC_NOMINAL_SPEED`` disagrees between the two official sources. The
# firmware says 16-bit; Motor Pilot's register list says S32. The firmware wins:
# reading it as 16-bit returns a value, reading it as 32-bit returns only a status
# byte.
MC_REG_SC_NOMINAL_SPEED = reg_id(99, TYPE_DATA_16BIT)
MC_REG_SC_J = reg_id(101, TYPE_DATA_32BIT)
MC_REG_SC_F = reg_id(102, TYPE_DATA_32BIT)
MC_REG_SC_MAX_CURRENT = reg_id(103, TYPE_DATA_32BIT)

MC_REG_OVERVOLTAGETHRESHOLD = reg_id(112, TYPE_DATA_16BIT)
MC_REG_UNDERVOLTAGETHRESHOLD = reg_id(113, TYPE_DATA_16BIT)

# Profiler 状态与进度，来源 RegListProfiler_MCPV2.json（官方 Motor Pilot 清单）。
# 注意：这三个是 U8，不是 F32；MCPV2 profiler 固件没有 PROFILER_STATE 寄存器，
# 那一族属于 RegistersCatalog.json 的另一套寄存器集，不能混用。
# Profiler state/progress registers, from the official Motor Pilot register list.
# These are U8, not F32, and MCPV2 profiler firmware has no PROFILER_STATE register.
MC_REG_SC_STATE = reg_id(16, TYPE_DATA_8BIT)
MC_REG_SC_STEPS = reg_id(17, TYPE_DATA_8BIT)
MC_REG_SC_PP = reg_id(18, TYPE_DATA_8BIT)
MC_REG_SC_COMPLETED = reg_id(20, TYPE_DATA_8BIT)
MC_REG_STATUS = reg_id(1, TYPE_DATA_8BIT)
MC_REG_RESISTOR_OFFSET = reg_id(116, TYPE_DATA_32BIT)

#: ``RESISTOR_OFFSET`` 的"尚未标定"哨兵值，取自官方 ``GUI/profiler.qml``。
#: The "offset unavailable" sentinel for ``RESISTOR_OFFSET``, from profiler.qml.
RESISTOR_OFFSET_UNAVAILABLE = -32768.0

MC_REG_BUS_VOLTAGE = reg_id(22, TYPE_DATA_16BIT)
MC_REG_FAULTS_FLAGS = reg_id(0, TYPE_DATA_32BIT)

# 母线电压换算 / Bus-voltage conversion —— 来源 Inc/parameters_conversion.h
ADC_REFERENCE_VOLTAGE = 3.3
VBUS_PARTITIONING_FACTOR = 0.0625
#: ADC 满量程母线电压 [V]；阈值类寄存器的数字量就是这个量程下的原始值。
#: ADC full-scale bus voltage [V]; threshold registers are raw counts on it.
VBUS_FULL_SCALE_V = ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR

#: 控制板/功率板/电机名称，用来确认连的是对的板子。
#: Controller / power-board / motor name registers, used to confirm identity.
MC_REG_FW_NAME = reg_id(0, TYPE_DATA_STRING)
MC_REG_CTRL_STAGE_NAME = reg_id(1, TYPE_DATA_STRING)
MC_REG_PWR_STAGE_NAME = reg_id(2, TYPE_DATA_STRING)
MC_REG_MOTOR_NAME = reg_id(3, TYPE_DATA_STRING)


# ---------------------------------------------------------------------------
# CRC-4 / CCITT-G704 x^4+x+1 —— 查表逐值取自 Src/aspep.c
# ---------------------------------------------------------------------------

CRC4_LOOKUP8 = bytes((
    0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x07, 0x05, 0x03, 0x01, 0x0F, 0x0D, 0x0B, 0x09,
    0x07, 0x05, 0x03, 0x01, 0x0F, 0x0D, 0x0B, 0x09, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E,
    0x0E, 0x0C, 0x0A, 0x08, 0x06, 0x04, 0x02, 0x00, 0x09, 0x0B, 0x0D, 0x0F, 0x01, 0x03, 0x05, 0x07,
    0x09, 0x0B, 0x0D, 0x0F, 0x01, 0x03, 0x05, 0x07, 0x0E, 0x0C, 0x0A, 0x08, 0x06, 0x04, 0x02, 0x00,
    0x0B, 0x09, 0x0F, 0x0D, 0x03, 0x01, 0x07, 0x05, 0x0C, 0x0E, 0x08, 0x0A, 0x04, 0x06, 0x00, 0x02,
    0x0C, 0x0E, 0x08, 0x0A, 0x04, 0x06, 0x00, 0x02, 0x0B, 0x09, 0x0F, 0x0D, 0x03, 0x01, 0x07, 0x05,
    0x05, 0x07, 0x01, 0x03, 0x0D, 0x0F, 0x09, 0x0B, 0x02, 0x00, 0x06, 0x04, 0x0A, 0x08, 0x0E, 0x0C,
    0x02, 0x00, 0x06, 0x04, 0x0A, 0x08, 0x0E, 0x0C, 0x05, 0x07, 0x01, 0x03, 0x0D, 0x0F, 0x09, 0x0B,
    0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x06, 0x04, 0x02, 0x00, 0x0E, 0x0C, 0x0A, 0x08,
    0x06, 0x04, 0x02, 0x00, 0x0E, 0x0C, 0x0A, 0x08, 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F,
    0x0F, 0x0D, 0x0B, 0x09, 0x07, 0x05, 0x03, 0x01, 0x08, 0x0A, 0x0C, 0x0E, 0x00, 0x02, 0x04, 0x06,
    0x08, 0x0A, 0x0C, 0x0E, 0x00, 0x02, 0x04, 0x06, 0x0F, 0x0D, 0x0B, 0x09, 0x07, 0x05, 0x03, 0x01,
    0x0A, 0x08, 0x0E, 0x0C, 0x02, 0x00, 0x06, 0x04, 0x0D, 0x0F, 0x09, 0x0B, 0x05, 0x07, 0x01, 0x03,
    0x0D, 0x0F, 0x09, 0x0B, 0x05, 0x07, 0x01, 0x03, 0x0A, 0x08, 0x0E, 0x0C, 0x02, 0x00, 0x06, 0x04,
    0x04, 0x06, 0x00, 0x02, 0x0C, 0x0E, 0x08, 0x0A, 0x03, 0x01, 0x07, 0x05, 0x0B, 0x09, 0x0F, 0x0D,
    0x03, 0x01, 0x07, 0x05, 0x0B, 0x09, 0x0F, 0x0D, 0x04, 0x06, 0x00, 0x02, 0x0C, 0x0E, 0x08, 0x0A,
))

CRC4_LOOKUP4 = bytes((
    0x00, 0x07, 0x0E, 0x09, 0x0B, 0x0C, 0x05, 0x02,
    0x01, 0x06, 0x0F, 0x08, 0x0A, 0x0D, 0x04, 0x03,
))


def crc4_header(header: int) -> int:
    """对头部低 28 位算 CRC-4，返回 4 位值。
    Compute the CRC-4 over the low 28 bits of a header; return 4 bits.

    算法与 ``aspep.c`` 的 ``ASPEP_ComputeHeaderCRC`` 逐字节一致：先按字节喂
    3 个字节，最后一个 nibble 用 4 位表。
    Byte-wise identical to the firmware: three bytes then one nibble.
    """
    header &= 0x0FFFFFFF
    crc = 0
    crc = CRC4_LOOKUP8[crc ^ (header & 0xFF)]
    crc = CRC4_LOOKUP8[crc ^ ((header >> 8) & 0xFF)]
    crc = CRC4_LOOKUP8[crc ^ ((header >> 16) & 0xFF)]
    crc = CRC4_LOOKUP4[crc ^ ((header >> 24) & 0x0F)]
    return crc & 0x0F


def crc4_check(header: int) -> bool:
    """校验含 CRC 的完整头部；算法与固件一致（结果为 0 才有效）。
    Verify a header that already carries its CRC; zero means valid, as in the
    firmware."""
    crc = 0
    crc = CRC4_LOOKUP8[crc ^ (header & 0xFF)]
    crc = CRC4_LOOKUP8[crc ^ ((header >> 8) & 0xFF)]
    crc = CRC4_LOOKUP8[crc ^ ((header >> 16) & 0xFF)]
    crc = CRC4_LOOKUP8[crc ^ ((header >> 24) & 0xFF)]
    return crc == 0


def build_header(packet_type: int, payload_len: int, packet_number: int = 0) -> int:
    """构造 32 位 ASPEP 头部并写入 CRC。
    Build a 32-bit ASPEP header with its CRC filled in."""
    if not 0 <= payload_len <= HDR_LEN_MASK:
        raise ValueError(f"payload length {payload_len} 超出 13 位范围 / out of 13-bit range")
    header = (packet_type & ID_MASK) | ((payload_len & HDR_LEN_MASK) << HDR_LEN_SHIFT)
    header |= (packet_number & 0xFFFF) << 12
    header |= crc4_header(header) << HDR_CRC_SHIFT
    return header & 0xFFFFFFFF


def parse_header(header: int) -> dict:
    """解析 ASPEP 头部；校验失败抛 ``ValueError``。
    Parse an ASPEP header; raise ``ValueError`` when the CRC does not check."""
    if not crc4_check(header):
        raise ValueError(f"ASPEP 头部 CRC 校验失败 / header CRC failed: 0x{header:08X}")
    return {
        "packet_type": header & ID_MASK,
        "payload_len": (header & 0x1FFF0) >> HDR_LEN_SHIFT,
        "packet_number": (header & HDR_PKTNUM_MASK) >> HDR_PKTNUM_SHIFT,
    }


def encode_frame(packet_type: int, payload: bytes, packet_number: int = 0) -> bytes:
    """把负载封成一个完整 ASPEP 帧（头部 + 负载）。
    Wrap a payload into a complete ASPEP frame (header + payload).

    ``packet_number`` 只对 PING 有意义；数据帧和 BEACON 保留字段填 0，与固件
    ``ASPEP_sendPacket`` 的做法一致。漏掉这个参数会让 PING 的包序号永远是 0。
    ``packet_number`` matters for PING only; data and BEACON frames leave the field
    zero exactly as the firmware's ``ASPEP_sendPacket`` does.
    """
    header = build_header(packet_type, len(payload), packet_number)
    return struct.pack("<I", header) + payload


# ---------------------------------------------------------------------------
# 能力协商 / Capability negotiation
# ---------------------------------------------------------------------------

@dataclass
class Capabilities:
    """ASPEP 双方协商出的能力集。
    The capability set negotiated between controller and performer.

    ``DATA_CRC`` 必须协商成 0：官方固件的数据段 CRC 是固定常量的桩实现。
    ``DATA_CRC`` must negotiate to 0 because the official data CRC is a stub.
    """

    version: int = 0
    data_crc: int = 0
    rx_max_size: int = 0
    txs_max_size: int = 0
    txa_max_size: int = 0

    def encode(self) -> bytes:
        """编成 4 字节 BEACON 负载。 / Encode as a 4-byte BEACON payload."""
        value = (
            (BEACON & ID_MASK)
            | ((self.version & 0x7) << BEACON_VER_SHIFT)
            | ((self.data_crc & 0x1) << BEACON_DATACRC_SHIFT)
            | ((self.rx_max_size & 0x3F) << BEACON_RXMAX_SHIFT)
            | ((self.txs_max_size & 0x7F) << BEACON_TXSMAX_SHIFT)
            | ((self.txa_max_size & 0x7F) << BEACON_TXAMAX_SHIFT)
        )
        return struct.pack("<I", value)

    @classmethod
    def decode(cls, payload: bytes) -> "Capabilities":
        """从 BEACON 负载解出能力集。 / Decode capabilities from a BEACON."""
        if len(payload) < ASPEP_CTRL_SIZE:
            raise ValueError(f"BEACON 负载短于 4 字节 / short BEACON: {len(payload)}")
        value = struct.unpack_from("<I", payload, 0)[0]
        return cls(
            version=(value >> BEACON_VER_SHIFT) & 0x7,
            data_crc=(value >> BEACON_DATACRC_SHIFT) & 0x1,
            rx_max_size=(value >> BEACON_RXMAX_SHIFT) & 0x3F,
            txs_max_size=(value >> BEACON_TXSMAX_SHIFT) & 0x7F,
            txa_max_size=(value >> BEACON_TXAMAX_SHIFT) & 0x7F,
        )


@dataclass
class Ping:
    """PING 控制帧 / A PING control frame."""

    c_bit: int = 0
    n_bit: int = 0
    ip_id: int = 0
    packet_number: int = 0

    def encode(self) -> bytes:
        value = (
            (PING & ID_MASK)
            | ((self.c_bit & 1) << 4)
            | ((self.c_bit & 1) << 5)
            | ((self.n_bit & 1) << 6)
            | ((self.n_bit & 1) << 7)
            | ((self.ip_id & 0xF) << 8)
            | ((self.packet_number & 0xFFFF) << 12)
        )
        return struct.pack("<I", value)

    @classmethod
    def decode(cls, payload: bytes) -> "Ping":
        if len(payload) < ASPEP_CTRL_SIZE:
            raise ValueError(f"PING 负载短于 4 字节 / short PING: {len(payload)}")
        value = struct.unpack_from("<I", payload, 0)[0]
        return cls(
            c_bit=(value >> 4) & 1,
            n_bit=(value >> 6) & 1,
            ip_id=(value >> 8) & 0xF,
            packet_number=(value >> 12) & 0xFFFF,
        )


@dataclass
class Nack:
    """NACK 控制帧 / A NACK control frame."""

    error_info: int = 0

    def encode(self) -> bytes:
        return struct.pack(
            "<I",
            (NACK & ID_MASK) | ((self.error_info & 0xFF) << 8) | ((self.error_info & 0xFF) << 16),
        )

    @classmethod
    def decode(cls, payload: bytes) -> "Nack":
        if len(payload) < ASPEP_CTRL_SIZE:
            raise ValueError(f"NACK 负载短于 4 字节 / short NACK: {len(payload)}")
        value = struct.unpack_from("<I", payload, 0)[0]
        return cls(error_info=(value >> 8) & 0xFF)


# ASPEP 错误码 ↔ 名称，便于日志可读。 / Error code names for readable logs.
ASPEP_ERROR_NAMES = {
    1: "BAD_PACKET_TYPE",
    2: "BAD_PACKET_SIZE",
    4: "BAD_CRC_HEADER",
    5: "BAD_CRC_DATA",
}


# ---------------------------------------------------------------------------
# MCP 负载构造 / MCP payload construction
# ---------------------------------------------------------------------------

def mcp_read_request(element_id: int, motor_id: int = 1) -> bytes:
    """构造"读寄存器"的同步负载：命令头 + element ID。
    Build a synchronous "read register" payload: command header + element ID.

    ``element_id`` 必须已经是"寄存器号 + 类型编码"的完整 16 位值，**不能**再套
    ``REG_MASK``：``REG_MASK`` 是给固件从完整 element ID 里抽寄存器号用的
    （``regID = *dataElementID & REG_MASK``），在组包方向套它会连类型位一起抹掉，
    固件随后会回 ``MCP_ERROR_BAD_DATA_TYPE``。
    ``element_id`` must already be the composed 16-bit value. Do NOT apply
    ``REG_MASK`` here: the firmware uses that mask to extract the register number
    from a complete element ID, so applying it while composing erases the type
    bits and the firmware answers ``MCP_ERROR_BAD_DATA_TYPE``.

    固件解析时做 ``(*packetHeader - 1) & MOTOR_MASK``，所以 ``motor_id`` 从 1 开始。
    The firmware evaluates ``(*packetHeader - 1) & MOTOR_MASK``, so motor ids are
    1-based.
    """
    return struct.pack("<HH", GET_DATA_ELEMENT, element_id | (motor_id & MOTOR_MASK))


def mcp_write_request(element_id: int, payload: bytes, motor_id: int = 1) -> bytes:
    """构造"写寄存器"的同步负载：命令头 + element ID + 数据。
    Build a synchronous "write register" payload. See ``mcp_read_request`` for why
    ``element_id`` must not be re-masked."""
    return (
        struct.pack("<HH", SET_DATA_ELEMENT, element_id | (motor_id & MOTOR_MASK))
        + payload
    )


def mcp_command_request(command: int) -> bytes:
    """构造无参命令负载（START_MOTOR / STOP_MOTOR / SW_RESET 等）。
    Build a parameterless command payload."""
    return struct.pack("<H", command)


def decode_float(raw: bytes) -> float:
    """把 4 字节小端解码成 float32。 / Decode 4 little-endian bytes as float32."""
    if len(raw) < 4:
        raise ValueError(f"float 响应短于 4 字节 / short float response: {len(raw)}")
    return struct.unpack_from("<f", raw, 0)[0]


def decode_u16(raw: bytes) -> int:
    """把 2 字节小端解码成 uint16。 / Decode 2 little-endian bytes as uint16."""
    if len(raw) < 2:
        raise ValueError(f"u16 响应短于 2 字节 / short u16 response: {len(raw)}")
    return struct.unpack_from("<H", raw, 0)[0]


def vbus_counts_to_volts(counts: int) -> float:
    """母线电压数字量 → 伏特。 / Bus-voltage counts to volts.

    ``VOLTS = counts * VBUS_FULL_SCALE_V / 65535``，与固件
    ``SCC_Get*Threshold`` 的回读公式同源。
    """
    return counts * VBUS_FULL_SCALE_V / 65535.0


def volts_to_vbus_counts(volts: float) -> int:
    """伏特 → 母线电压数字量。 / Volts to bus-voltage counts.

    这是阈值对话框真正需要的换算：界面标着 ``%``，实际是 0–52.8 V 的伏特值。
    This is the conversion the threshold dialog actually needs: the label says
    percent, the value is volts on a 0-52.8 V scale.
    """
    return int(round(volts * 65535.0 / VBUS_FULL_SCALE_V))


# ---------------------------------------------------------------------------
# 串口客户端 / Serial client
# ---------------------------------------------------------------------------

class ProfilerError(RuntimeError):
    """协议或固件层面的错误。 / A protocol- or firmware-level error."""


def default_host_capabilities() -> Capabilities:
    """控制端默认能力，逐值取自官方 ``Src/mcp_config.c`` 的 ``aspepOverUartA``。
    Default controller capabilities, taken value by value from the official
    ``aspepOverUartA`` in ``Src/mcp_config.c``.

    ``RX_maxSize  = (MCP_RX_SYNC_PAYLOAD_MAX  >> 5) - 1 = (256>>5)-1 = 7``
    ``TXS_maxSize = (MCP_TX_SYNC_PAYLOAD_MAX  >> 5) - 1 = (256>>5)-1 = 7``
    ``TXA_maxSize = (MCP_TX_ASYNC_PAYLOAD_MAX_A >> 6)   = 2048>>6     = 32``
    ``DATA_CRC = 0``（数据段 CRC 是桩实现，必须为 0）
    ``version = 0``

    ``ASPEP_CheckBeacon`` 对 ``version``/``DATA_CRC``/``TXS``/``TXA`` 用严格相等，
    只有控制端的 ``RX`` 允许更大，所以这些值不能随手改。
    ``ASPEP_CheckBeacon`` compares with strict equality except the controller's
    ``RX``, so these values must not be changed casually.
    """
    return Capabilities(version=0, data_crc=0, rx_max_size=7, txs_max_size=7,
                        txa_max_size=32)


class MotorProfilerClient:
    """ST Motor Profiler 固件的同步串口客户端。
    Synchronous serial client for the ST Motor Profiler firmware.

    只实现握手 + 同步寄存器读写。异步（异步流）通道不实现，因为读取识别结果用
    同步读就够了，少一条并发路径就少一类不确定性。
    Only the handshake and synchronous register access are implemented. The
    asynchronous streaming channel is deliberately omitted: reading profiling
    results needs only synchronous reads.

    ``serial_factory`` 允许注入假串口做离线测试。
    ``serial_factory`` allows injecting a fake port for offline tests.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 1843200,
        timeout_s: float = 1.0,
        serial_factory: Optional[Callable[..., object]] = None,
        log: Optional[Callable[[str], None]] = None,
        host_capabilities: Optional[Capabilities] = None,
        inter_byte_delay_s: float = 0.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._serial_factory = serial_factory
        self._log = log or (lambda _msg: None)
        self._ser = None
        self.peer: Optional[Capabilities] = None
        self.negotiated: Optional[Capabilities] = None
        self.connected = False
        self.sync_packet_count = 0
        self.last_packet_numbers: list = []
        # 字节间延时：**实机需要，默认关闭**。在 1843200 baud 下一次写完一帧时，
        # 目标侧偶发收不全（实测同一帧有时被处理、有时完全没有反应）。CLI 在真实
        # 串口路径上默认给 1 ms/字节。默认 0 是为了让离线测试不必等真实时间。
        # Inter-byte pacing: needed on real hardware, off by default. Writing a whole
        # frame in one go at 1843200 baud is occasionally not fully received by the
        # target (measured: the same frame is sometimes processed, sometimes ignored).
        # The CLI defaults to 1 ms per byte on a real port; the library default is 0
        # so offline tests do not wait on real time.
        self.inter_byte_delay_s = inter_byte_delay_s
        # 控制端能力：默认取官方 ``Src/mcp_config.c`` 里 ``aspepOverUartA`` 的值，
        # 因为 ``ASPEP_CheckBeacon`` 要求双方严格相等，猜错就永远连不上。
        # Controller capabilities, defaulted to the official ``aspepOverUartA``
        # values because ``ASPEP_CheckBeacon`` demands strict equality.
        self.host_capabilities = host_capabilities or default_host_capabilities()

    # -- 生命周期 / lifecycle ------------------------------------------------

    def open(self) -> None:
        """打开串口。 / Open the serial port."""
        if self._ser is not None:
            return
        factory = self._serial_factory
        if factory is None:
            import serial  # 延迟导入，便于无 pyserial 环境下跑纯帧测试

            factory = serial.Serial
        self._ser = factory(self.port, self.baudrate, timeout=self.timeout_s)
        self._log(f"串口已打开 / port open: {self.port} @ {self.baudrate}")

    def close(self) -> None:
        """关闭串口。 / Close the serial port."""
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None
                self.connected = False
                self._log("串口已关闭 / port closed")

    def __enter__(self) -> "MotorProfilerClient":
        self.open()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # -- 低层收发 / low-level I/O -------------------------------------------

    def _write(self, data: bytes) -> None:
        """按 ``inter_byte_delay_s`` 逐字节写，避免目标收不全。
        Write byte by byte honouring ``inter_byte_delay_s`` so the target receives
        every byte.

        用 ``getattr`` 取 ``flush``：假串口与某些串口实现没有这个方法，直接调用会
        让离线测试全部报错。
        ``flush`` is looked up with ``getattr`` because fake ports and some real ones
        do not provide it.
        """
        self._log(f"TX {len(data):3d}B  {data.hex(' ')}")
        if self.inter_byte_delay_s <= 0.0:
            self._ser.write(data)
            return
        flush = getattr(self._ser, "flush", None)
        for index, byte in enumerate(data):
            self._ser.write(bytes((byte,)))
            if flush is not None:
                flush()
            if index + 1 < len(data):
                time.sleep(self.inter_byte_delay_s)

    def _read_exact(self, count: int, deadline_s: float) -> bytes:
        """在截止时间内读满 ``count`` 字节。
        Read exactly ``count`` bytes before a deadline."""
        buf = bytearray()
        while len(buf) < count:
            remaining = deadline_s - time.monotonic()
            if remaining <= 0:
                raise ProfilerError(
                    f"读超时 / read timeout: 期望 {count} 字节, 只收到 {len(buf)}"
                )
            chunk = self._ser.read(count - len(buf))
            if not chunk:
                continue
            buf.extend(chunk)
        return bytes(buf)

    def _read_frame(self, deadline_s: float) -> tuple:
        """读一个完整 ASPEP 帧，返回 ``(packet_type, payload)``。
        Read one complete ASPEP frame and return ``(packet_type, payload)``."""
        raw_header = self._read_exact(ASPEP_HEADER_SIZE, deadline_s)
        header = struct.unpack("<I", raw_header)[0]
        self._log(f"RX 头部 / header 0x{header:08X}")
        info = parse_header(header)
        payload = b""
        if info["payload_len"]:
            payload = self._read_exact(info["payload_len"], deadline_s)
            self._log(f"RX {info['payload_len']:3d}B  {payload.hex(' ')}")
        if info["packet_type"] == DATA_PACKET:
            self.sync_packet_count += 1
        elif info["packet_type"] == PING:
            self.last_packet_numbers.append(Ping.decode(payload).packet_number)
        return info["packet_type"], payload

    def _send_beacon(self, caps: Capabilities) -> None:
        self._write(encode_frame(BEACON, caps.encode()))

    def _send_ping(self, c_bit: int, packet_number: int) -> None:
        ping = Ping(c_bit=c_bit, n_bit=self.sync_packet_count & 1, packet_number=packet_number)
        self._write(encode_frame(PING, ping.encode(), packet_number=packet_number))

    def _send_data(self, payload: bytes) -> None:
        self._write(encode_frame(DATA_PACKET, payload))

    # -- 握手 / handshake ---------------------------------------------------

    def connect(self, timeout_s: float = 5.0) -> Capabilities:
        """走完 ASPEP 状态机直到 CONNECTED。
        Drive the ASPEP state machine up to CONNECTED.

        **控制端必须先发 BEACON。** 这一点从固件源码确认，不是惯例：
        ``ASPEP_start`` 只做 ``fASPEP_cfg_recept(rxHeader, 4)``（只配置接收），
        整个固件没有任何主动发送 BEACON 的路径；``mc_tasks.c`` 调用
        ``ASPEP_start(&aspepOverUartA)`` 之后就一直在等。所以"等对方先发包"会
        永远超时。
        **The controller must send the first BEACON.** Confirmed from the firmware:
        ``ASPEP_start`` only arms reception, nothing ever sends an unsolicited
        BEACON, and ``mc_tasks.c`` calls ``ASPEP_start`` and then waits. Waiting for
        the peer to speak first times out forever.

        状态机（``aspep.c`` 的 ``ASPEP_RXframeProcess``）：
        performer 在 IDLE 收到 BEACON → 校验能力、回 BEACON 并进入 CONFIGURED；
        在 CONFIGURED 收到 PING → 回 PING 并进入 CONNECTED。

        能力必须**完全相等**才谈得成（``ASPEP_CheckBeacon`` 对
        ``version``/``DATA_CRC``/``TXS``/``TXA`` 用严格相等，只允许控制端的
        ``RX`` 更大），所以首包直接按目标固件的已知能力发。
        Capabilities must match exactly — strict equality on ``version`` /
        ``DATA_CRC`` / ``TXS`` / ``TXA``, with only the controller's ``RX`` allowed to
        be larger — so the first beacon carries the target firmware's known values.
        """
        deadline = time.monotonic() + timeout_s
        host = self.host_capabilities
        self._send_beacon(host)
        self._log(f"已发送首个 BEACON / initial beacon sent: {host}")

        # 阶段 1：等 performer 回 BEACON，确认能力谈成。
        self.peer = None
        while time.monotonic() < deadline:
            ptype, payload = self._read_frame(deadline)
            if ptype == BEACON:
                self.peer = Capabilities.decode(payload)
                break
        if self.peer is None:
            raise ProfilerError(
                "未收到 BEACON 应答；请确认烧录的是 Profiler 固件、串口与波特率正确、"
                "且 Motor Pilot 等上位机没有占用串口 / "
                "no BEACON reply; check the profiler firmware, port, baud rate and that "
                "no other host tool holds the port"
            )
        self._log(f"performer 能力 / peer capabilities: {self.peer}")

        if self.peer.data_crc != 0:
            raise ProfilerError(
                f"performer 要求 DATA_CRC={self.peer.data_crc}，但官方固件的数据段 CRC "
                "是固定常量的桩实现（0xCA 0xFE），无法使用 / "
                "the official data CRC is a constant stub and cannot be used"
            )

        # 阶段 2：发 PING(CFG) 完成连接。
        self._send_ping(ASPEP_PING_CFG, packet_number=0)
        while time.monotonic() < deadline:
            try:
                ptype, payload = self._read_frame(deadline)
            except ProfilerError:
                break
            if ptype == PING:
                ping = Ping.decode(payload)
                if ping.c_bit != 1:
                    # c_bit=0 表示 performer 中途复位过，必须重新握手。
                    raise ProfilerError(
                        "performer 报告自己已复位（PING c_bit=0），需要重新握手 / "
                        "performer reports a reset; re-handshake required"
                    )
                self.connected = True
                self.negotiated = host
                self._log("已连接 / connected")
                return host
            if ptype == NACK:
                raise ProfilerError(f"performer 回报 NACK / NACK: {Nack.decode(payload)}")
        raise ProfilerError("握手未完成 / handshake did not complete")

    # -- 寄存器读写 / register access ---------------------------------------

    def transact(self, payload: bytes, timeout_s: float = 2.0) -> bytes:
        """发一个同步负载并取回同步响应。
        Send one synchronous payload and return the synchronous response."""
        if not self.connected:
            raise ProfilerError("尚未连接 / not connected")
        self._send_data(payload)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ptype, response = self._read_frame(deadline)
            if ptype == DATA_PACKET:
                return response
            if ptype == NACK:
                nack = Nack.decode(response)
                name = ASPEP_ERROR_NAMES.get(nack.error_info, "?")
                raise ProfilerError(f"performer NACK {nack.error_info} ({name})")
            # BEACON / PING 是心跳，忽略后继续等同步响应。
        raise ProfilerError("未收到同步响应 / no synchronous response")

    def read_float(self, element_id: int, motor_id: int = 1) -> float:
        """读一个 F32 寄存器。 / Read one F32 register."""
        response = self.transact(mcp_read_request(element_id, motor_id))
        # 末尾 1 字节是 MCP 响应码：0 = OK。
        if len(response) >= 5:
            status = response[-1]
            if status != MCP_CMD_OK:
                raise ProfilerError(f"读寄存器 0x{element_id:04X} 失败 / read failed: {status}")
            return decode_float(response[:4])
        if len(response) == 4:
            return decode_float(response)
        raise ProfilerError(f"读响应长度异常 / odd read response: {len(response)} 字节")

    def read_u16(self, element_id: int, motor_id: int = 1) -> int:
        """读一个 U16 寄存器。 / Read one U16 register."""
        response = self.transact(mcp_read_request(element_id, motor_id))
        if len(response) >= 3:
            status = response[-1]
            if status != MCP_CMD_OK:
                raise ProfilerError(f"读寄存器 0x{element_id:04X} 失败 / read failed: {status}")
            return decode_u16(response[:2])
        if len(response) == 2:
            return decode_u16(response)
        raise ProfilerError(f"读响应长度异常 / odd read response: {len(response)} 字节")

    def read_u8(self, element_id: int, motor_id: int = 1) -> int:
        """读一个 U8 寄存器（``SC_STATE``/``SC_STEPS``/``SC_COMPLETED`` 等）。
        Read one U8 register (``SC_STATE`` / ``SC_STEPS`` / ``SC_COMPLETED``)."""
        response = self.transact(mcp_read_request(element_id, motor_id))
        if len(response) >= 2:
            status = response[-1]
            if status != MCP_CMD_OK:
                raise ProfilerError(f"读寄存器 0x{element_id:04X} 失败 / read failed: {status}")
            return response[0]
        if len(response) == 1:
            return response[0]
        raise ProfilerError(f"读响应长度异常 / odd read response: {len(response)} 字节")

    def read_s16(self, element_id: int, motor_id: int = 1) -> int:
        """读一个 S16 寄存器（``OVERVOLTAGETHRESHOLD`` / ``UNDERVOLTAGETHRESHOLD``）。
        Read one S16 register (the bus-voltage thresholds).

        这两个阈值官方清单标的是 ``S16``，与固件的 ``TYPE_DATA_16BIT`` 只差符号
        解释，元素编码相同。
        The thresholds are declared S16 by the official list; the element encoding is
        the same as the firmware's ``TYPE_DATA_16BIT``, only the sign differs.
        """
        response = self.transact(mcp_read_request(element_id, motor_id))
        if len(response) >= 3:
            status = response[-1]
            if status != MCP_CMD_OK:
                raise ProfilerError(f"读寄存器 0x{element_id:04X} 失败 / read failed: {status}")
            return struct.unpack_from("<h", response, 0)[0]
        if len(response) == 2:
            return struct.unpack_from("<h", response, 0)[0]
        raise ProfilerError(f"读响应长度异常 / odd read response: {len(response)} 字节")

    def read_string(self, element_id: int, motor_id: int = 1) -> str:
        """读一个 STRING 寄存器。 / Read one STRING register."""
        response = self.transact(mcp_read_request(element_id, motor_id))
        if response and response[-1] == MCP_CMD_OK and len(response) > 1:
            response = response[:-1]
        return response.split(b"\x00", 1)[0].decode("ascii", errors="replace")

    def write_raw(self, element_id: int, payload: bytes, motor_id: int = 1) -> None:
        """写一个 RAW 寄存器（整块字节）。 / Write one RAW register (raw bytes)."""
        response = self.transact(mcp_write_request(element_id, payload, motor_id))
        if response and response[-1] != MCP_CMD_OK:
            raise ProfilerError(f"写寄存器 0x{element_id:04X} 失败 / write failed")

    def send_command(self, command: int, timeout_s: float = 2.0) -> None:
        """下发无参命令。 / Issue a parameterless command."""
        response = self.transact(mcp_command_request(command), timeout_s=timeout_s)
        if response and response[-1] != MCP_CMD_OK:
            raise ProfilerError(f"命令 0x{command:02X} 失败 / command failed: {response[-1]}")

    # -- 便捷读取 / convenience readers -------------------------------------

    def identify(self) -> dict:
        """读回板卡与电机身份，用来确认连的是对的硬件。
        Read back board and motor identity to confirm the hardware matches."""
        return {
            "firmware": self.read_string(MC_REG_FW_NAME),
            "controller": self.read_string(MC_REG_CTRL_STAGE_NAME),
            "power_board": self.read_string(MC_REG_PWR_STAGE_NAME),
            "motor": self.read_string(MC_REG_MOTOR_NAME),
        }

    def read_results(self) -> dict:
        """读回 profiler 的识别结果。 / Read back the profiler identification results.

        这一组全部是 ``F32``，与固件 ``register_interface.h`` 逐条一致。
        ``SC_NOMINAL_SPEED`` 不在这里：它是 16 位输入寄存器（且两处官方定义冲突），
        用 :meth:`read_s16` 单独读。
        All of these are F32 and match the firmware definitions. ``SC_NOMINAL_SPEED``
        is deliberately excluded: it is a 16-bit input register whose official
        definitions conflict; read it with :meth:`read_s16`.
        """
        return {
            "rs_ohm": self.read_float(MC_REG_SC_RS),
            "ls_h": self.read_float(MC_REG_SC_LS),
            "ke": self.read_float(MC_REG_SC_KE),
            "vbus_v": self.read_float(MC_REG_SC_VBUS),
            "meas_nominal_speed": self.read_float(MC_REG_SC_MEAS_NOMINALSPEED),
            "ld_lq_ratio": self.read_float(MC_REG_SC_LDLQRATIO),
            "inertia_j": self.read_float(MC_REG_SC_J),
            "friction_f": self.read_float(MC_REG_SC_F),
            "max_current": self.read_float(MC_REG_SC_MAX_CURRENT),
            "current": self.read_float(MC_REG_SC_CURRENT),
        }

    def read_thresholds(self) -> dict:
        """读回母线电压阈值，并换算成伏特。
        Read back the bus-voltage thresholds and convert to volts."""
        over_counts = self.read_s16(MC_REG_OVERVOLTAGETHRESHOLD)
        under_counts = self.read_s16(MC_REG_UNDERVOLTAGETHRESHOLD)
        return {
            "over_threshold_counts": over_counts,
            "over_threshold_v": vbus_counts_to_volts(over_counts),
            "under_threshold_counts": under_counts,
            "under_threshold_v": vbus_counts_to_volts(under_counts),
        }


# ---------------------------------------------------------------------------
# 离线自测用：假串口 / Fake port for offline tests
# ---------------------------------------------------------------------------

class FakePerformer:
    """一个最小的 ASPEP performer 模拟器，用来在无硬件时验证客户端状态机。
    A minimal ASPEP performer emulator, used to validate the client without
    hardware.

    它按 ``aspep.c`` 的状态机回帧，并支持通过 ``register_map`` 注入寄存器值。
    It answers frames exactly as ``aspep.c`` does and serves register reads from
    an injectable ``register_map``.
    """

    def __init__(self, capabilities: Optional[Capabilities] = None,
                 register_map: Optional[dict] = None):
        # 默认能力取自官方 ``Src/mcp_config.c`` 的 ``aspepOverUartA``：
        #   RX_maxSize  = (256 >> 5) - 1 = 7
        #   TXS_maxSize = (256 >> 5) - 1 = 7
        #   TXA_maxSize = (2048 >> 6)    = 32
        #   DATA_CRC    = 0
        # Defaults mirror the official ``aspepOverUartA`` in ``Src/mcp_config.c``.
        self.caps = capabilities or Capabilities(
            version=0, data_crc=0, rx_max_size=7, txs_max_size=7, txa_max_size=32
        )
        self.register_map = dict(register_map or {})
        self.state = "IDLE"
        self.tx = bytearray()
        # 真实固件从不主动发 BEACON（``ASPEP_start`` 只配置接收），所以默认是
        # False；置位只用于单独测试客户端的"等到应答"路径。
        # The real firmware never sends an unsolicited BEACON, so this defaults to
        # False; enabling it only exercises the client's wait-for-reply path.
        self.pending_beacon_on_open = False
        # 记录每次能力校验结果，便于测试断言"为什么被拒"。
        # Record each capability check so tests can assert why a beacon was rejected.
        self.beacon_checks: list = []
        # 置位后永远判定能力不符，用来测试客户端的失败路径。
        # When set, every capability check fails; used to exercise the client's
        # failure path.
        self.reject_all_beacons = False
        # 置位后不响应完成连接的 PING，模拟能力始终谈不拢的 performer。
        # When set, the completing PING is never answered, emulating a performer
        # whose capabilities never line up.
        self.refuse_completing_ping = False

    def check_beacon(self, peer: "Capabilities") -> list:
        """复刻 ``ASPEP_CheckBeacon`` 的判定，返回不匹配原因列表（空=通过）。
        Mirror ``ASPEP_CheckBeacon``; return the list of mismatched fields (empty
        means the beacon is accepted)."""
        if self.reject_all_beacons:
            return ["forced_rejection"]
        reasons = []
        if peer.version != self.caps.version:
            reasons.append("version")
        if peer.data_crc != self.caps.data_crc:
            reasons.append("data_crc")
        if peer.rx_max_size > self.caps.rx_max_size:
            reasons.append("rx_max_size_too_big")
        if peer.txs_max_size != self.caps.txs_max_size:
            reasons.append("txs_max_size")
        if peer.txa_max_size != self.caps.txa_max_size:
            reasons.append("txa_max_size")
        return reasons

    def _frame(self, packet_type: int, payload: bytes) -> bytes:
        return encode_frame(packet_type, payload)

    def open(self) -> None:
        if self.pending_beacon_on_open:
            self.tx.extend(self._frame(BEACON, self.caps.encode()))
            self.pending_beacon_on_open = False

    def close(self) -> None:
        pass

    @property
    def in_waiting(self) -> int:
        return len(self.tx)

    def write(self, data: bytes) -> int:
        if len(data) < ASPEP_HEADER_SIZE:
            return len(data)
        header = struct.unpack_from("<I", data, 0)[0]
        info = parse_header(header)
        payload = data[ASPEP_HEADER_SIZE:ASPEP_HEADER_SIZE + info["payload_len"]]
        ptype = info["packet_type"]

        if ptype == BEACON:
            peer = Capabilities.decode(payload)
            reasons = self.check_beacon(peer)
            self.beacon_checks.append({"peer": peer, "reasons": reasons})
            self.state = "CONFIGURED" if not reasons else "IDLE"
            self.tx.extend(self._frame(BEACON, self.caps.encode()))
        elif ptype == PING:
            ping = Ping.decode(payload)
            if self.refuse_completing_ping:
                return len(data)
            if self.state == "CONFIGURED":
                self.state = "CONNECTED"
            self.tx.extend(
                self._frame(PING, Ping(c_bit=1, n_bit=0, ip_id=0,
                                       packet_number=ping.packet_number).encode())
            )
        elif ptype == DATA_PACKET:
            self.tx.extend(self._handle_mcp(payload))
        return len(data)

    def _handle_mcp(self, payload: bytes) -> bytes:
        if len(payload) < MCP_HEADER_SIZE:
            return self._frame(DATA_PACKET, bytes([MCP_CMD_NOK]))
        command = struct.unpack_from("<H", payload, 0)[0] & CMD_MASK
        body = payload[MCP_HEADER_SIZE:]
        if command == GET_MCP_VERSION:
            return self._frame(DATA_PACKET, bytes([MCP_VERSION, MCP_CMD_OK]))
        if command == GET_DATA_ELEMENT:
            if len(body) < MCP_ID_SIZE:
                return self._frame(DATA_PACKET, bytes([MCP_CMD_NOK]))
            element_id = struct.unpack_from("<H", body, 0)[0]
            # 复刻固件：寄存器号从完整 element ID 里用 REG_MASK 抽出，类型位单独校验。
            # Mirrors the firmware: the register number comes out of the composed
            # element ID, and the type bits are validated separately.
            register_number = element_id & REG_MASK
            type_code = element_id & TYPE_MASK
            value = self.register_map.get(register_number)
            if value is not None:
                expected_type = (
                    TYPE_DATA_32BIT if isinstance(value, float)
                    else TYPE_DATA_8BIT if isinstance(value, int) and 0 <= value <= 0xFF
                    else TYPE_DATA_16BIT if isinstance(value, int)
                    else TYPE_DATA_STRING if isinstance(value, str)
                    else None
                )
                if expected_type is not None and type_code != expected_type:
                    return self._frame(DATA_PACKET, bytes([MCP_ERROR_BAD_DATA_TYPE]))
            if value is None:
                return self._frame(DATA_PACKET, bytes([MCP_CMD_NOK]))
            if isinstance(value, float):
                return self._frame(DATA_PACKET, struct.pack("<f", value) + bytes([MCP_CMD_OK]))
            if isinstance(value, int):
                # U8 与 U16 共用 int 存储，按类型位决定宽度。
                # U8 and U16 both come from ints; the type bits pick the width.
                fmt = "<B" if type_code == TYPE_DATA_8BIT else "<H"
                return self._frame(DATA_PACKET, struct.pack(fmt, value) + bytes([MCP_CMD_OK]))
            if isinstance(value, str):
                return self._frame(
                    DATA_PACKET, value.encode("ascii") + b"\x00" + bytes([MCP_CMD_OK])
                )
            return self._frame(DATA_PACKET, bytes(value) + bytes([MCP_CMD_OK]))
        if command in (START_MOTOR, STOP_MOTOR, STOP_RAMP, START_STOP, SW_RESET):
            return self._frame(DATA_PACKET, bytes([MCP_CMD_OK]))
        return self._frame(DATA_PACKET, bytes([MCP_CMD_NOK]))

    def read(self, count: int = 1) -> bytes:
        chunk = bytes(self.tx[:count])
        del self.tx[:count]
        return chunk

    def flush(self) -> None:
        """``serial.Serial.flush`` 的空实现；假串口没有发送缓冲。
        No-op stand-in for ``serial.Serial.flush``; the fake port has no TX buffer."""
        return None

    def reset_input_buffer(self) -> None:
        pass


class FakeSerialFactory:
    """把 ``FakePerformer`` 包装成 ``serial.Serial`` 那样的可调用工厂。
    Wrap a ``FakePerformer`` as a ``serial.Serial``-like callable factory."""

    def __init__(self, performer: Optional[FakePerformer] = None):
        self.performer = performer or FakePerformer()

    def __call__(self, *_args, **_kwargs) -> FakePerformer:
        self.performer.open()
        return self.performer


def simulated_performer() -> FakePerformer:
    """构造一个带**真实量级**寄存器值的假 performer，用于离线自测整条链路。
    Build a fake performer carrying realistic register values so the whole path can
    be exercised offline.

    这些数值来自本工程的实机基线，不是随手编的：

    - 板卡身份 / board identity：``NUCLEO-G431RB`` + ``X-NUCLEO-IHM16M1``
      （与 ``docs/`` 记录的实机条件一致）
    - 母线电压阈值 / bus thresholds：固件默认 7 V / 15 V，对应数字量
      ``8688`` / ``18618``（``volts_to_vbus_counts``）
    - 母线电压 ``BUS_VOLTAGE``：实测约 12.3 V，按 ``52.8 V`` 满量程换算成数字量
    - 识别结果 / identified results：先用 ST Workbench 数据库参考值
      （Rs 5.292 Ω、Ls 1.058 mH、Ke 4.964），这样量纲和量级都对得上
    - ``RESISTOR_OFFSET`` 用 QML 里的哨兵值 ``-32768``，表示"尚未标定"
    """
    bus_volts = 12.3
    register_map = {
        MC_REG_FW_NAME: "Motor Profiler",
        MC_REG_CTRL_STAGE_NAME: "NUCLEO-G431RB",
        MC_REG_PWR_STAGE_NAME: "X-NUCLEO-IHM16M1",
        MC_REG_MOTOR_NAME: "GBM2804H-100T",
        MC_REG_OVERVOLTAGETHRESHOLD: volts_to_vbus_counts(15.0),
        MC_REG_UNDERVOLTAGETHRESHOLD: volts_to_vbus_counts(7.0),
        MC_REG_BUS_VOLTAGE: volts_to_vbus_counts(bus_volts),
        MC_REG_SC_RS: 5.292,
        MC_REG_SC_LS: 1.058e-3,
        MC_REG_SC_KE: 4.964,
        MC_REG_SC_VBUS: bus_volts,
        MC_REG_SC_MEAS_NOMINALSPEED: 1572.0,
        MC_REG_SC_NOMINAL_SPEED: 1572,
        MC_REG_SC_LDLQRATIO: 1.0,
        MC_REG_SC_J: 2.91e-5,
        MC_REG_SC_F: 9.37e-6,
        MC_REG_SC_MAX_CURRENT: 0.8,
        MC_REG_SC_CURRENT: 0.8,
        MC_REG_SC_STATE: 0,
        MC_REG_SC_STEPS: 0,
        MC_REG_SC_COMPLETED: 0,
        MC_REG_SC_PP: 7,
        MC_REG_STATUS: 0,
    }
    return FakePerformer(register_map=register_map)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _cmd_probe(args: argparse.Namespace) -> int:
    """只读探针：握手 + 读身份 + 读阈值 + 读当前识别结果（不启动电机）。
    Read-only probe: handshake, identity, thresholds and any existing results.

    **不会下发任何启动命令**，所以在接线首次验证时是安全的：即使读数是垃圾，
    也不会让功率级动起来。
    Issues no start command, so it is safe for the first contact with hardware.
    """
    # 日志走 stderr，stdout 只留 JSON，便于脚本直接管 pipe 解析。
    # Logs go to stderr so stdout carries only JSON and can be piped directly.
    factory = FakeSerialFactory(simulated_performer()) if args.simulate else None
    with MotorProfilerClient(
        port=args.port,
        baudrate=args.baud,
        serial_factory=factory,
        log=lambda msg: print(msg, file=sys.stderr),
        # 真实串口按 1 ms/字节发送；模拟模式不需要节奏。
        # Real ports pace at 1 ms per byte; simulation needs no pacing.
        inter_byte_delay_s=0.0 if args.simulate else args.byte_delay_ms / 1000.0,
    ) as client:
        client.connect()
        report: dict = {"identity": {}, "thresholds": {}, "results": {}, "raw": {}}
        try:
            report["identity"] = client.identify()
        except ProfilerError as exc:
            report["identity"] = {"error": str(exc)}
        try:
            report["raw"]["over_threshold"] = client.read_s16(MC_REG_OVERVOLTAGETHRESHOLD)
            report["raw"]["under_threshold"] = client.read_s16(MC_REG_UNDERVOLTAGETHRESHOLD)
            report["thresholds"] = client.read_thresholds()
        except ProfilerError as exc:
            report["thresholds"] = {"error": str(exc)}
        try:
            report["raw"]["bus_voltage_counts"] = client.read_u16(MC_REG_BUS_VOLTAGE)
            report["raw"]["bus_voltage_v"] = vbus_counts_to_volts(
                report["raw"]["bus_voltage_counts"]
            )
            report["raw"]["profiler_state"] = client.read_u8(MC_REG_SC_STATE)
            report["raw"]["profiler_steps"] = client.read_u8(MC_REG_SC_STEPS)
            report["raw"]["profiler_completed"] = client.read_u8(MC_REG_SC_COMPLETED)
            report["raw"]["pole_pairs"] = client.read_u8(MC_REG_SC_PP)
        except ProfilerError as exc:
            report["raw"]["error"] = str(exc)
        try:
            report["results"] = client.read_results()
        except ProfilerError as exc:
            report["results"] = {"error": str(exc)}
        print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。 / Build the command-line parser."""
    parser = argparse.ArgumentParser(
        description="ST Motor Profiler 固件的 ASPEP/MCP 串口客户端 / "
                    "ASPEP/MCP serial client for the ST Motor Profiler firmware"
    )
    parser.add_argument("--port", default="COM6", help="串口，默认 COM6 / serial port")
    parser.add_argument("--baud", type=int, default=1843200, help="波特率，固件固定 1843200")
    sub = parser.add_subparsers(dest="command", required=True)

    probe = sub.add_parser("probe", help="只读探针，不启动电机 / read-only probe")
    probe.add_argument("--simulate", action="store_true", help="用假串口自测 / use the fake port")
    probe.add_argument("--byte-delay-ms", type=float, default=1.0,
                       help="字节间延时，实机默认 1 ms；收到不到应答时调大 / "
                            "inter-byte delay in ms, 1 by default on a real port")
    probe.set_defaults(func=_cmd_probe)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """入口。 / Entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
