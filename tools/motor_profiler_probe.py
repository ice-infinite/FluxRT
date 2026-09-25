#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""针对 ST Motor Profiler 固件的原始串口对话诊断。
Raw serial-dialogue diagnostics for the ST Motor Profiler firmware.

用途 / Purpose
--------------
``motor_profiler_client.py`` 走的是**严格成帧**的路径：按头部长度字段读负载、
按状态机推进。当真实固件的行为与官方源码不一致时，严格路径会卡住而看不出原因。
本工具反过来：**贪婪收集原始字节**，按已知的固定字节模式统计响应，从而回答
"设备到底接受了什么、拒绝了什么"。

``motor_profiler_client.py`` takes a strictly framed path: it reads payloads by the
header's length field and advances a state machine. When the real firmware differs
from the official source, that path stalls without explaining why. This tool does the
opposite: it collects raw bytes greedily and counts known fixed byte patterns, which
answers "what does the device accept and what does it reject".

本工具**不启动电机**，只发协议控制帧。
This tool never starts the motor; it only sends protocol control frames.

实测结论（2026-09-25，板上为 Motor Profiler 固件）
--------------------------------------------------
从官方源码（``Src/aspep.c``、``Inc/aspep.h``）逆向出的帧格式与**实测固件**存在
三处差异，均由本工具发现：

1. **控制帧没有负载。** 官方 ``ASPEP_sendBeacon`` / ``ASPEP_sendPing`` /
   ``ASPEP_sendNack`` 都构造 4 字节负载并传 ``ASPEP_CTRL_SIZE`` 给发送函数，
   但实测设备发的是**只有 4 字节头部**的帧，负载长度为 0。响应里出现的
   ``05 00 00 50``（BEACON）与 ``0f 04 04 c0``（NACK）都是 4 字节。
2. **NACK 不带负载。** 官方把错误码放在负载的 bit 8-15 与 bit 16-23；
   实测错误码直接编码在**头部**的对应位（``0f 04 04 c0`` 的 bit 8-15 为 0x04，
   即 ``ASPEP_BAD_CRC_HEADER``）。
3. **发送速率影响可靠性。** 1843200 baud 下把整帧一次写完时目标偶发收不全；
   逐字节加约 1 ms 间隔后 BEACON 交换稳定（多次运行每次都能拿到 BEACON 回应）。
   Windows 侧一次写入 8 字节的实测耗时约 170–210 us，远高于 8 字节 @1843200 的
   理论值 43 us，说明写入经过 OS/VCP 缓冲。

握手进展 / Handshake progress
-----------------------------
- **BEACON 交换：成功。** 设备接受控制端发来的 BEACON 并回一个 BEACON。
  实测约 10 组不同的能力值（含官方 ``aspepOverUartA`` 的 7/7/32）都能拿到
  每次一个 BEACON 回应，说明能力协商**不是**当前的阻塞点。
- **PING(CFG)：被拒。** 设备对 PING 回 NACK，因此状态机不进入 CONNECTED，
  后续 DATA_PACKET（寄存器读写）同样被拒。

已知未解 / Open question
------------------------
为什么 PING 被拒尚未定位。官方 ``aspep.c`` 的状态机在 IDLE / CONFIGURED /
CONNECTED 三个状态都处理 PING，所以按源码不应该被拒。
**下一步建议**：用 ``--dump`` 抓一次真实上位机（Motor Pilot）与设备的完整对话，
与 ``--beacon``/``--ping`` 的输出逐字节对比——这是唯一能不靠猜测定位差异的方法。
Why PING is rejected is not yet located. The next step is to capture the real host
tool's dialogue with ``--dump`` and diff it byte for byte against this tool's output.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

import motor_profiler_client as mpc  # noqa: E402

#: 实测固件应答里的固定字节模式。用于在不依赖成帧的前提下统计"接受/拒绝"。
#: Fixed byte patterns seen from the measured firmware; counting them avoids relying
#: on framing, which the real firmware does not follow.
KNOWN_PATTERNS = {
    "beacon_reply": bytes((0x05, 0x00, 0x00, 0x50)),
    "nack_bad_crc_header": bytes((0x0F, 0x04, 0x04, 0xC0)),
    "nack_bad_packet_size": bytes((0x0F, 0x02, 0x02, 0x50)),
    "ping_reply": bytes((0x06, 0x00, 0x00, 0x60)),
}

#: ASPEP NACK 错误码名称，取自 ``Src/aspep.h``。
NACK_REASONS = {1: "BAD_PACKET_TYPE", 2: "BAD_PACKET_SIZE",
                4: "BAD_CRC_HEADER", 5: "BAD_CRC_DATA"}


def count_patterns(blob: bytes) -> dict:
    """统计每种已知模式在原始字节里出现的次数。
    Count each known pattern's occurrences in a raw byte blob."""
    return {name: blob.count(pat) for name, pat in KNOWN_PATTERNS.items()}


def decode_nack_header(blob: bytes) -> list:
    """从原始字节里抽出所有 4 字节 NACK 头部并解出错误码。
    Extract every 4-byte NACK header from a raw blob and decode its error code.

    实测固件的 NACK 只有头部、没有负载，所以这里按 4 字节切分而不是按帧长度切分。
    The measured firmware's NACK is header-only, so this slices by 4 bytes rather than
    by a frame length.
    """
    found = []
    for offset in range(0, len(blob) - 3, 4):
        chunk = blob[offset:offset + 4]
        header = struct.unpack("<I", chunk)[0]
        if (header & 0xF) != mpc.NACK:
            continue
        if not mpc.crc4_check(header):
            continue
        error_info = (header >> 8) & 0xFF
        found.append({
            "offset": offset,
            "header": f"0x{header:08X}",
            "error_info": error_info,
            "reason": NACK_REASONS.get(error_info, "UNKNOWN"),
        })
    return found


class PacedPort:
    """带字节间延时的串口包装：实机需要节奏，否则整帧一次写完偶发收不全。
    A serial wrapper that paces bytes; real hardware needs pacing or a whole-frame
    write is occasionally not fully received."""

    def __init__(self, port: str, baudrate: int, delay_s: float, timeout_s: float = 0.05):
        import serial

        self.ser = serial.Serial(port, baudrate, timeout=timeout_s)
        self.delay_s = delay_s

    def send(self, frame: bytes) -> None:
        for byte in frame:
            self.ser.write(bytes((byte,)))
            self.ser.flush()
            if self.delay_s > 0:
                time.sleep(self.delay_s)

    def drain(self, seconds: float) -> bytes:
        end = time.monotonic() + seconds
        blob = bytearray()
        while time.monotonic() < end:
            chunk = self.ser.read(4096)
            if chunk:
                blob.extend(chunk)
        return bytes(blob)

    def close(self) -> None:
        self.ser.close()


def build_steps(args: argparse.Namespace) -> list:
    """按参数组装要发的帧序列。 / Assemble the frame sequence from the options."""
    caps = mpc.default_host_capabilities()
    if args.capabilities:
        rx, txs, txa = (int(v) for v in args.capabilities.split(","))
        caps = mpc.Capabilities(version=0, data_crc=0, rx_max_size=rx,
                                txs_max_size=txs, txa_max_size=txa)
    beacon = mpc.encode_frame(mpc.BEACON, caps.encode())
    ping = mpc.encode_frame(
        mpc.PING,
        mpc.Ping(c_bit=1, n_bit=0, ip_id=0, packet_number=0).encode(),
        packet_number=0,
    )
    read_vbus = mpc.encode_frame(mpc.DATA_PACKET, mpc.mcp_read_request(mpc.MC_REG_SC_VBUS))

    steps = []
    if args.beacon or args.all:
        steps.append(("BEACON", beacon, args.gap))
    if args.ping or args.all:
        steps.append(("PING(CFG)", ping, args.gap))
    if args.read or args.all:
        steps.append(("READ SC_VBUS", read_vbus, args.gap))
    if args.dump:
        steps = []  # dump 模式不发任何东西，只监听
    return steps


def _cmd_run(args: argparse.Namespace) -> int:
    """执行诊断并输出报告。 / Run the diagnostics and print a report."""
    steps = build_steps(args)
    if not steps and not args.dump:
        print("没有要执行的步骤；用 --beacon/--ping/--read/--all/--dump 指定 / "
              "no steps selected", file=sys.stderr)
        return 2

    port = PacedPort(args.port, args.baud, args.byte_delay_ms / 1000.0)
    report = {
        "port": args.port,
        "baud": args.baud,
        "byte_delay_ms": args.byte_delay_ms,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "steps": [],
    }
    try:
        time.sleep(args.settle_s)
        port.ser.reset_input_buffer()

        if args.dump:
            print(f"纯监听 {args.listen_s}s（不发任何帧）...")
            blob = port.drain(args.listen_s)
            report["steps"].append({"name": "DUMP", "tx": "", "rx": blob.hex(" ")})
            print(f"  收到 {len(blob)}B: {blob.hex(' ') if blob else '(空)'}")
        else:
            for name, frame, gap in steps:
                port.ser.reset_input_buffer()
                print(f"--- {name} ---")
                print(f"  TX: {frame.hex(' ')}")
                port.send(frame)
                blob = port.drain(gap)
                counts = count_patterns(blob)
                nacks = decode_nack_header(blob)
                print(f"  RX: {blob.hex(' ') if blob else '(空)'}")
                for key, value in counts.items():
                    if value:
                        print(f"    {key}: {value}")
                for nack in nacks:
                    print(f"    NACK @{nack['offset']}: {nack['header']} "
                          f"err={nack['error_info']} ({nack['reason']})")
                report["steps"].append({
                    "name": name,
                    "tx": frame.hex(" "),
                    "rx": blob.hex(" "),
                    "patterns": {k: v for k, v in counts.items() if v},
                    "nacks": nacks,
                })
                time.sleep(args.inter_step_s)

        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        report["accepted_beacon"] = any(
            s.get("patterns", {}).get("beacon_reply") for s in report["steps"]
        )
    finally:
        port.close()

    print()
    print("=== 汇总 / summary ===")
    print(f"  设备接受 BEACON: {report['accepted_beacon']}")
    all_nacks = [n for s in report["steps"] for n in s.get("nacks", [])]
    if all_nacks:
        reasons = sorted({n["reason"] for n in all_nacks})
        print(f"  收到的 NACK 原因: {', '.join(reasons)}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(f"  报告已写出 / report written: {args.output}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。 / Build the command-line parser."""
    parser = argparse.ArgumentParser(
        description="ST Motor Profiler 固件的原始串口对话诊断 / raw dialogue diagnostics"
    )
    parser.add_argument("--port", default="COM6", help="串口，默认 COM6")
    parser.add_argument("--baud", type=int, default=1843200, help="波特率")
    parser.add_argument("--byte-delay-ms", type=float, default=1.0,
                       help="字节间延时，默认 1 ms；实机必需")
    parser.add_argument("--settle-s", type=float, default=0.3,
                       help="打开串口后的等待时间")
    parser.add_argument("--gap", type=float, default=1.0,
                       help="每个步骤后收集响应的时间")
    parser.add_argument("--inter-step-s", type=float, default=0.4,
                       help="步骤之间的间隔")
    parser.add_argument("--capabilities", default=None,
                       help="覆盖能力协商值，格式 RX,TXS,TXA")
    parser.add_argument("--beacon", action="store_true", help="只发 BEACON")
    parser.add_argument("--ping", action="store_true", help="只发 PING")
    parser.add_argument("--read", action="store_true", help="只读 SC_VBUS")
    parser.add_argument("--all", action="store_true", help="依次发 BEACON/PING/READ")
    parser.add_argument("--dump", action="store_true",
                       help="纯监听，不发帧（用于抓真实上位机的对话）")
    parser.add_argument("--listen-s", type=float, default=5.0, help="--dump 的监听时长")
    parser.add_argument("--output", type=Path, default=None, help="把 JSON 报告写到该路径")
    parser.set_defaults(func=_cmd_run)
    return parser


def main(argv: Optional[list] = None) -> int:
    """入口。 / Entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not any((args.beacon, args.ping, args.read, args.all, args.dump)):
        args.all = True
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
