#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Motor Profiler 参数识别运行器：连接、启动、轮询、取结果。
Motor Profiler identification runner: connect, start, poll, collect.

用途 / Purpose
--------------
把"启动识别并等它跑完"这件事做成可重复、可留证的一步。所有协议细节都已在
``motor_profiler_client.py`` 与
``docs/MotorProfiler串口协议逆向与实测差异.md`` 里固化；本工具只负责编排
与记录。

Drive "start the profiling and wait for it to finish" as one repeatable step that
leaves evidence behind. The protocol details are documented in the client module and
in ``docs/MotorProfiler串口协议逆向与实测差异.md``; this tool only orchestrates and
records.

前置条件（缺一不可）/ Preconditions
-----------------------------------
1. 板上为 **Motor_Profiler** 固件，且已应用 ``docs/patches/`` 的两处修复；
2. 电机已接 IHM16M1，母线约 12.3 V、限流 2 A，**空载可自由旋转**；
3. **没有其它程序占用串口**（Motor Pilot 必须完全退出，否则读不到响应）。

The board must run the patched Motor_Profiler firmware, the motor must be wired to
the IHM16M1 with ~12.3 V / 2 A and free to spin, and no other program may hold the
port (Motor Pilot must be fully closed).

安全 / Safety
-------------
识别会让电机转动。``--allow-motor-run`` 是强制开关，不给就不启动。
Profiling spins the motor; ``--allow-motor-run`` is mandatory to start it.
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

#: ``SCC_State_t`` 名称，取自固件 ``mp_self_com_ctrl.h``。
#: Names from the firmware's ``SCC_State_t``.
SCC_STATE_NAMES = {
    0: "SCC_IDLE",
    1: "SCC_DUTY_DETECTING_PHASE",
    2: "SCC_ALIGN_PHASE",
    3: "SCC_RS_DETECTING_PHASE_RAMP",
    4: "SCC_RS_DETECTING_PHASE",
    5: "SCC_LS_DETECTING_PHASE",
    6: "SCC_WAIT_RESTART",
    7: "SCC_RESTART_SCC",
    8: "SCC_KE_DETECTING_PHASE",
    9: "SCC_PHASE_STOP",
    10: "SCC_CALIBRATION_END",
}

#: 要采集的识别结果寄存器 ``(名称, 寄存器号, 类型)``。
#: Result registers to snapshot as ``(name, index, type)``.
RESULT_REGISTERS = (
    ("rs_ohm", 91, mpc.TYPE_DATA_32BIT),
    ("ls_h", 92, mpc.TYPE_DATA_32BIT),
    ("ke", 93, mpc.TYPE_DATA_32BIT),
    ("vbus_v", 94, mpc.TYPE_DATA_32BIT),
    ("meas_nominal_speed", 95, mpc.TYPE_DATA_32BIT),
    ("inertia_j", 101, mpc.TYPE_DATA_32BIT),
    ("friction_f", 102, mpc.TYPE_DATA_32BIT),
)

#: Profiler 控制寄存器 ``(名称, 寄存器号, 类型)``。
#: Profiler control registers.
CONTROL_REGISTERS = (
    ("state", 16, mpc.TYPE_DATA_8BIT),
    ("completed", 20, mpc.TYPE_DATA_8BIT),
    ("check", 15, mpc.TYPE_DATA_8BIT),
    ("pole_pairs", 18, mpc.TYPE_DATA_8BIT),
    ("steps", 17, mpc.TYPE_DATA_8BIT),
    ("current_a", 96, mpc.TYPE_DATA_32BIT),
    ("nominal_speed", 99, mpc.TYPE_DATA_16BIT),
)


class IdentificationError(RuntimeError):
    """识别过程中的错误。 / An error during identification."""


class Device:
    """带节拍的设备会话：握手一次，然后反复读写寄存器。
    A paced device session: handshake once, then read and write registers.

    用**原始串口**而不是 ``MotorProfilerClient`` 的高层入口，因为实测这台设备的
    应答格式与库里的严格成帧假设不同（控制帧无负载、应答长度字段不可信）。
    这里沿用已在实机上验证过的做法。

    Uses the raw port rather than the client's high-level entry points, because the
    measured device does not match the library's strict framing assumptions. The
    proven sequence is reproduced here.
    """

    def __init__(self, port: str, baud: int = 1843200, byte_delay_s: float = 0.003,
                 log=None) -> None:
        import serial

        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.byte_delay_s = byte_delay_s
        self.log = log or (lambda _m: None)

    def close(self) -> None:
        """关闭串口。 / Close the port."""
        self.ser.close()

    def _send(self, frame: bytes) -> None:
        """按字节节拍发送。 / Send with per-byte pacing."""
        for byte in frame:
            self.ser.write(bytes((byte,)))
            self.ser.flush()
            time.sleep(self.byte_delay_s)

    def _read(self, seconds: float, limit: int = 128) -> bytes:
        """在 ``seconds`` 内收集字节。 / Collect bytes for ``seconds``."""
        end = time.monotonic() + seconds
        buf = bytearray()
        while time.monotonic() < end:
            chunk = self.ser.read(limit)
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    @staticmethod
    def _has_beacon_reply(blob: bytes) -> bool:
        """响应里是否含 BEACON 应答。 / Whether the blob carries a BEACON reply.

        实测形态是 ``05 c7 01 14``（回它自己的能力值），但只认前两字节，避免把
        能力值的其它可能取值写死。
        The measured form is ``05 c7 01 14`` (its own capability payload); only the
        first two bytes are pinned so other capability values still match.
        """
        return bytes((0x05, 0xC7)) in blob

    @staticmethod
    def _has_ping_reply(blob: bytes) -> bool:
        """响应里是否含 PING 应答。 / Whether the blob carries a PING reply.

        实测同时出现 ``06 00 00 60`` 与 ``f6 00 00 c0`` 两种形态，差别在 n_bit
        （``syncPacketCount`` 的奇偶）。两者都是有效应答，所以按**类型 nibble** 判定：
        低 4 位为 6（PING）且高 4 位为 CRC，头部 CRC 自洽即可。
        Both ``06 00 00 60`` and ``f6 00 00 c0`` occur, differing only in n_bit (the
        parity of ``syncPacketCount``). Both are valid, so the check keys on the type
        nibble: the low 4 bits must be 6 (PING) and the header CRC must verify.
        """
        for offset in range(0, max(0, len(blob) - 3)):
            header = struct.unpack_from("<I", blob, offset)[0]
            if (header & 0x0F) == mpc.PING and mpc.crc4_check(header):
                return True
        return False

    def handshake(self, attempts: int = 6) -> bool:
        """反复发 BEACON+PING，直到在一次交换里同时看到两者的应答。
        Repeat BEACON+PING until one exchange shows both replies.

        判定在**整段响应里查找**而不是只看首字节：实测设备经常先回一条 NACK
        （此前积累的错误状态），真正的应答跟在后面。
        The check searches the whole response rather than only its first byte: the
        measured device often emits a NACK first, with the real reply behind it.
        """
        for attempt in range(1, attempts + 1):
            self.ser.reset_input_buffer()
            self._send(mpc.encode_frame(mpc.BEACON,
                                       mpc.default_host_capabilities().encode()))
            first = self._read(0.9)
            self.ser.reset_input_buffer()
            self._send(mpc.encode_frame(
                mpc.PING, mpc.Ping(c_bit=1, n_bit=0, ip_id=0, packet_number=0).encode(),
                packet_number=0))
            second = self._read(0.9)
            combined = first + second
            beacon_ok = self._has_beacon_reply(combined)
            ping_ok = self._has_ping_reply(combined)
            self.log(f"  握手尝试 {attempt}: BEACON={first.hex(' ') or '无'} | "
                     f"PING={second.hex(' ') or '无'}"
                     f"  -> beacon={beacon_ok} ping={ping_ok}")
            if beacon_ok and ping_ok:
                return True
        return False

    def read(self, index: int, type_code: int):
        """读一个寄存器。 / Read one register."""
        self.ser.reset_input_buffer()
        self._send(mpc.encode_frame(
            mpc.DATA_PACKET, mpc.mcp_read_request(mpc.reg_id(index, type_code))))
        raw = self._read(0.8, 64)
        if len(raw) < 4:
            return None
        header = struct.unpack_from("<I", raw, 0)[0]
        if not mpc.crc4_check(header):
            return None
        length = (header & 0x1FFF0) >> 4
        payload = raw[4:4 + length]
        if type_code == mpc.TYPE_DATA_8BIT:
            return payload[0] if payload else None
        if type_code == mpc.TYPE_DATA_16BIT:
            return struct.unpack_from("<H", payload, 0)[0] if len(payload) >= 2 else None
        if type_code == mpc.TYPE_DATA_32BIT:
            return struct.unpack_from("<f", payload, 0)[0] if len(payload) >= 4 else None
        return None

    def write_float(self, index: int, value: float) -> bytes:
        """写一个 F32 寄存器。 / Write one F32 register."""
        self.ser.reset_input_buffer()
        self._send(mpc.encode_frame(
            mpc.DATA_PACKET,
            mpc.mcp_write_request(mpc.reg_id(index, mpc.TYPE_DATA_32BIT),
                                  struct.pack("<f", value))))
        return self._read(0.9, 32)

    def profiler_command(self, command: int) -> bytes:
        """下发 Profiler 命令（负载首字节为命令字，实测合法值 0/1/6）。
        Issue a profiler command; the payload's first byte is the selector, and the
        measured valid values are 0, 1 and 6."""
        self.ser.reset_input_buffer()
        self._send(mpc.encode_frame(
            mpc.DATA_PACKET, struct.pack("<H", 0x68) + bytes((command, 0))))
        return self._read(0.9, 32)

    def snapshot(self) -> dict:
        """读一份控制与结果快照。 / Read one control-and-result snapshot."""
        snap: dict = {}
        for name, index, type_code in CONTROL_REGISTERS:
            snap[name] = self.read(index, type_code)
        for name, index, type_code in RESULT_REGISTERS:
            snap[name] = self.read(index, type_code)
        return snap


def run_identification(device: Device, current_a: float, max_seconds: float,
                       poll_s: float, log) -> dict:
    """对一档电流跑一次识别并轮询到完成或超时。
    Run one identification at one current level, polling until done or timeout."""
    record = {
        "current_requested_a": current_a,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "polls": [],
        "final": None,
        "completed": False,
    }

    log(f"  设定识别电流为 {current_a} A")
    response = device.write_float(96, current_a)
    log(f"    写 SC_CURRENT 响应: {response.hex(' ') if response else '无'}")

    log("  发 STOP(cmd=0)")
    log(f"    响应: {device.profiler_command(0).hex(' ') or '无'}")
    time.sleep(0.5)
    log("  发 START(cmd=1)")
    log(f"    响应: {device.profiler_command(1).hex(' ') or '无'}")

    deadline = time.monotonic() + max_seconds
    while time.monotonic() < deadline:
        snap = device.snapshot()
        state = snap.get("state")
        record["polls"].append({
            "t_s": round(max_seconds - (deadline - time.monotonic()), 1),
            "state": state,
            "state_name": SCC_STATE_NAMES.get(state, "?") if state is not None else None,
            "completed": snap.get("completed"),
            "current_a": snap.get("current_a"),
            "rs_ohm": snap.get("rs_ohm"),
            "ls_h": snap.get("ls_h"),
            "ke": snap.get("ke"),
        })
        log(f"    t={record['polls'][-1]['t_s']:6.1f}s  "
            f"state={state}({record['polls'][-1]['state_name']})  "
            f"completed={snap.get('completed')}  "
            f"Rs={snap.get('rs_ohm')}  Ls={snap.get('ls_h')}  Ke={snap.get('ke')}  "
            f"I={snap.get('current_a')}")
        if snap.get("completed") == 1:
            record["completed"] = True
            break
        time.sleep(poll_s)

    record["final"] = device.snapshot()
    record["finished_utc"] = datetime.now(timezone.utc).isoformat()
    return record


def _cmd_run(args: argparse.Namespace) -> int:
    """执行识别流程。 / Run the identification flow."""
    if not args.allow_motor_run:
        print("拒绝执行：识别会让电机转动，必须显式给出 --allow-motor-run / "
              "refusing to spin the motor without --allow-motor-run", file=sys.stderr)
        return 2

    log = (lambda m: print(m, flush=True)) if not args.quiet else (lambda _m: None)
    report = {
        "port": args.port,
        "baud": args.baud,
        "currents_a": args.current,
        "max_seconds_per_current": args.max_seconds,
        "runs": [],
    }

    device = Device(args.port, args.baud, args.byte_delay_ms / 1000.0, log=log)
    try:
        log(f"打开 {args.port} @ {args.baud}")
        time.sleep(args.settle_s)
        log("握手 BEACON+PING ...")
        if not device.handshake():
            print("握手失败：确认板上有修复后的 Profiler 固件、且没有其它程序占用串口 / "
                  "handshake failed", file=sys.stderr)
            return 3
        log("  握手成功")

        before = device.snapshot()
        report["before"] = before
        log(f"起始状态: state={before.get('state')} "
            f"({SCC_STATE_NAMES.get(before.get('state'), '?')})  "
            f"completed={before.get('completed')}  current={before.get('current_a')}")

        for index, current in enumerate(args.current, start=1):
            log(f"\n=== 第 {index}/{len(args.current)} 档: {current} A ===")
            record = run_identification(device, current, args.max_seconds,
                                        args.poll_s, log)
            report["runs"].append(record)
            log(f"  本档完成: {record['completed']}  "
                f"Rs={record['final'].get('rs_ohm')}  "
                f"Ls={record['final'].get('ls_h')}  "
                f"Ke={record['final'].get('ke')}")
            if index < len(args.current):
                log("  停 5 s 让电机与绕组回到起点")
                device.profiler_command(0)
                time.sleep(5.0)

        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
    finally:
        try:
            device.profiler_command(0)
            log("\n已发 STOP，电机应停止")
        except Exception:
            pass
        device.close()

    print("\n=== 汇总 / summary ===")
    for index, record in enumerate(report["runs"], start=1):
        final = record["final"] or {}
        print(f"  第 {index} 档 {record['current_requested_a']} A: "
              f"completed={record['completed']}  "
              f"Rs={final.get('rs_ohm')}  Ls={final.get('ls_h')}  "
              f"Ke={final.get('ke')}  "
              f"J={final.get('inertia_j')}  F={final.get('friction_f')}  "
              f"maxspeed={final.get('meas_nominal_speed')}")

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"\n报告已写出 / report written: {args.output}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。 / Build the command-line parser."""
    parser = argparse.ArgumentParser(
        description="Motor Profiler 参数识别运行器 / identification runner")
    parser.add_argument("--port", default="COM6", help="串口，默认 COM6")
    parser.add_argument("--baud", type=int, default=1843200, help="波特率")
    parser.add_argument("--byte-delay-ms", type=float, default=3.0,
                        help="字节间延时，默认 3 ms；收不到响应时调大")
    parser.add_argument("--settle-s", type=float, default=0.5,
                        help="打开串口后的等待时间")
    parser.add_argument("--current", type=float, nargs="+", default=[0.8],
                        help="识别电流档位，可给多个，例如 0.4 0.6 0.8")
    parser.add_argument("--max-seconds", type=float, default=180.0,
                        help="每档最长轮询时间，默认 180 s")
    parser.add_argument("--poll-s", type=float, default=3.0,
                        help="轮询间隔，默认 3 s")
    parser.add_argument("--output", type=Path, default=None, help="JSON 报告输出路径")
    parser.add_argument("--quiet", action="store_true", help="只输出汇总")
    parser.add_argument("--allow-motor-run", action="store_true",
                        help="必须显式给出，否则拒绝启动识别")
    parser.set_defaults(func=_cmd_run)
    return parser


def main(argv: Optional[list] = None) -> int:
    """入口。 / Entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
