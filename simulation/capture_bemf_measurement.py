#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BEMF 实测采集器：开环强拖 -> 停机 -> 抓停机后三相端电压固定窗。

BEMF measurement capture: spin the motor open-loop, stop, then grab the
post-stop phase-voltage fixed window.

用途 / Purpose
--------------
用 FluxRT Diagnostic 固件的开环强拖把电机拖到已知转速，然后 ``foc_stop``
关断栅极，让电机靠惯性滑行/减速。停机瞬间马上触发 256 拍 @ 12 kHz 的
相电压固定窗（``foc_phase_capture``，分压 ON）。分析器会先构造 U-V/V-W/W-U
线间波形，消除浮动中性点和公共模电压，再换算相反电动势。

为什么这样能判定 Ke 的单位口径 / Why this settles the Ke convention
-------------------------------------------------------------------
Profiler 读回的 ``SC_KE`` 存在线电压 RMS 与相电压 RMS 两种口径的歧义，两者
相差 ``sqrt(3)``，换算成磁链后相差 3.83 倍（见
``profiles/identification/evidence/motor-profiler-20260925/README.md``）。
一次"已知转速下的开路相电压幅值"实测可以同时裁定 Ke 的数值与口径：

    相电压峰值   V_ph_pk  = BEMF 波形幅值（单相，对地）
    相电压 RMS   V_ph_rms = V_ph_pk / sqrt(2)
    线电压 RMS   V_ll_rms = V_ph_rms * sqrt(3)

两种口径的 Ke：
    Ke(线RMS) = V_ll_rms / (n_rpm / 1000)
    Ke(相RMS) = V_ph_rms / (n_rpm / 1000)

与 Profiler 的 ``SC_KE`` 对比，哪个口径吻合，磁链就按哪个口径换算。

重要边界 / Important limits
---------------------------
- 分压网络是 ST 官方**名义**参数（10k/2.2k，4469 uV/count），不是逐板标定值
  （A19 静态标定因缺可信万用表参考未完成）。对"裁定 3.83 倍口径差异"来说，
  名义模型精度绰绰有余（需要区分的比值是 1.73 或 3.83，不是 1.05）。
- 磁链主结果由采集窗内的线间电压幅值和电频率共同求出，不依赖指令转速。
  ``foc_start`` 目标只用于开环跟踪/滑行衰减诊断。
- 256 拍 @ 12 kHz = 21.3 ms。1000 rpm / 7 对极 = 116.7 Hz 电频率，
  21.3 ms 只有约 2.5 个电周期，测频精度约 +/-2%。判口径足够。

前置条件 / Preconditions
------------------------
1. 板上是 FluxRT **Diagnostic** 固件（带 foc_phase_capture + foc_start），
   不是 Motor_Profiler 固件；
2. 电机空载、可自由旋转、无桨叶无负载，母线 12.0~12.6 V 限流 2 A；
3. 没有其它程序占用串口。

安全 / Safety
-------------
本脚本会让电机转动，``--allow-motor-run`` 是强制开关。
The motor spins; ``--allow-motor-run`` is mandatory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import sys
import time

SAMPLE_RATE_HZ = 12000
WINDOW_SAMPLES = 256
DEFAULT_ALIGNMENT_S = 2.0
DEFAULT_RAMP_S = 5.0
#: IHM16M1 名义分压模型：3.3 V / 12 bit / 10k-2.2k，官方名义 4469 uV/count。
NOMINAL_UV_PER_COUNT = 4469


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rpm", type=float, default=582.0,
                        help="open-loop target speed in rpm (default 582, hardware-proven)")
    parser.add_argument("--hold-s", type=float, default=3.0,
                        help="seconds to hold the forced speed before stop (default 3.0)")
    parser.add_argument("--alignment-s", type=float, default=DEFAULT_ALIGNMENT_S,
                        help="CM4.4 alignment duration (default 2.0 s)")
    parser.add_argument("--ramp-s", type=float, default=DEFAULT_RAMP_S,
                        help="CM4.4 open-loop ramp duration (default 5.0 s)")
    parser.add_argument("--output", type=pathlib.Path, required=True,
                        help="output CSV path; .log/.json siblings are written too")
    parser.add_argument("--allow-motor-run", action="store_true",
                        help="mandatory safety switch: the motor will spin")
    return parser.parse_args()


def read_for(port: object, seconds: float, raw: list[str]) -> list[str]:
    deadline = time.monotonic() + seconds
    result: list[str] = []
    while time.monotonic() < deadline:
        if port.in_waiting:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if line:
                raw.append(line)
                result.append(line)
        else:
            time.sleep(0.01)
    return result


def command(port: object, text: str, wait_s: float, raw: list[str]) -> list[str]:
    port.write((text + "\r\n").encode("ascii"))
    port.flush()
    return read_for(port, wait_s, raw)


def stop_and_arm_capture(port: object, raw: list[str]) -> tuple[list[str], float]:
    """把停机与采集 arm 连续排入 Shell，返回主机侧两次写入间隔。"""
    stop_sent_at = time.monotonic()
    port.write(b"foc_stop\r\n")
    port.flush()
    arm_sent_at = time.monotonic()
    port.write(b"foc_phase_capture start on\r\n")
    port.flush()
    return read_for(port, 0.35, raw), (arm_sent_at - stop_sent_at) * 1000.0


def best_effort_command(port: object, text: str, wait_s: float,
                        raw: list[str]) -> None:
    """失败路径尽力停机，不用二次串口异常覆盖原始错误。"""
    try:
        command(port, text, wait_s, raw)
    except Exception as error:  # noqa: BLE001 - shutdown path must not mask root cause
        raw.append(f"HOST_SHUTDOWN_WARNING,{text},{type(error).__name__},{error}")


def parse_status_speed(lines: list[str]) -> float | None:
    """从 FSTAT 行解析观测器估计转速（仅用于一致性交叉检查，不是真值）。"""
    for line in lines:
        marker = line.find("FSTAT,")
        if marker < 0:
            continue
        fields = line[marker:].split(",")
        # FSTAT,state,observer,reliable,closed,target_rpm,measured_rpm
        if len(fields) >= 7:
            try:
                return float(fields[6])
            except ValueError:
                continue
    return None


def parse_runtime_timing(lines: list[str]) -> tuple[float, int, int] | None:
    """解析 C0：返回开环终速 [rpm]、对齐 [ms]、升速 [ms]。"""
    for line in lines:
        marker = line.find("C0,")
        if marker < 0:
            continue
        fields = line[marker:].split(",")
        if len(fields) < 6:
            continue
        startup = fields[4].split("/")
        timing = fields[5].split("/")
        if len(startup) < 1 or len(timing) < 2:
            continue
        try:
            return float(startup[0]), int(timing[0]), int(timing[1])
        except ValueError:
            continue
    return None


def main() -> int:
    args = parse_args()
    if not args.allow_motor_run:
        print("Refusing to spin the motor without --allow-motor-run.")
        return 2
    if args.rpm < 500.0 or args.rpm > 1200.0:
        # 低于 500 rpm BEMF 太小信噪比差；高于 1200 rpm 超出数据库最大转速。
        print(f"Refusing rpm={args.rpm}: use 500..1200 (BEMF SNR vs motor limit).")
        return 2
    if args.hold_s < 0.5 or args.alignment_s <= 0.0 or args.ramp_s <= 0.0:
        print("Refusing timing: hold-s >= 0.5 and alignment/ramp > 0 are required.")
        return 2

    try:
        import serial as serial_module
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    raw: list[str] = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix(".log")
    start_ack_wait_s = 1.0
    total_spin_s = args.alignment_s + args.ramp_s + args.hold_s
    stop_to_arm_host_ms: float | None = None
    original_timing: tuple[float, int, int] | None = None

    with serial_module.Serial(args.port, args.baud, timeout=0.1) as port:
        try:
            read_for(port, 0.4, raw)
            # 确保从停机、采集器空闲的状态开始。
            command(port, "foc_stop", 0.25, raw)
            command(port, "foc_phase_capture stop", 0.15, raw)
            # BEMF 实验必须保持纯开环强拖；停机状态显式关闭闭环，最终也保留安全默认。
            original_timing = parse_runtime_timing(command(port, "foc_cfg show", 0.2, raw))
            if original_timing is None:
                raise RuntimeError("cannot parse C0 runtime timing before capture")
            command(port, "foc_cfg closedloop 0", 0.2, raw)
            # foc_start 的参数是闭环目标；开环强拖实际使用 startup_final_speed_rpm。
            # 因此本实验必须把 startup/align/ramp 三项一起设置成请求值，否则命令写
            # 1000 rpm 而转子仍只会被默认 582 rpm 的强制角拖动。
            command(port, f"foc_cfg startup {int(args.rpm)}", 0.2, raw)
            command(port, f"foc_cfg align {round(args.alignment_s * 1000.0)}", 0.2, raw)
            command(port, f"foc_cfg ramp {round(args.ramp_s * 1000.0)}", 0.2, raw)
            # armed 采集器会拒绝 foc_start（互斥保护），因此采用顺序法：
            # 先强拖，再停机，停机后立刻 arm+完成（自动 256 拍）。
            # 采集窗口从 arm 时刻开始，因此"停机 -> arm"的间隔要尽量短。
            start_lines = command(
                port, f"foc_start {int(args.rpm)}", start_ack_wait_s, raw)
            if not any("FSTART,armed" in line for line in start_lines):
                raise RuntimeError("foc_start was refused; check bus window and diagnostics")
            # CM4.4 默认是 2 s 对齐 + 5 s 升速。command() 已经等待了一部分，
            # 这里只补足剩余时间，保证从 foc_start 到 foc_stop 总计恰好
            # alignment + ramp + hold，而不是在升速未结束时提前停机。
            remaining_spin_s = max(0.0, total_spin_s - start_ack_wait_s)
            read_for(port, remaining_spin_s, raw)
            pre_stop_status = command(port, "foc_status", 0.25, raw)
            speed_estimate = parse_status_speed(pre_stop_status)
            # 连续排队停机与 arm，避免旧实现等待 FSTOP 回显造成约 124 ms 的转速衰减。
            # Shell 仍按顺序执行：先关栅极，再接通分压并 arm 固定窗。
            cap_lines, stop_to_arm_host_ms = stop_and_arm_capture(port, raw)
            if not any("FSTOP" in line for line in cap_lines):
                raise RuntimeError("foc_stop acknowledgement missing before capture")
            if not any("FPV armed,on" in line for line in cap_lines):
                raise RuntimeError("phase capture was refused after foc_stop")
            # 256 拍 @ 12 kHz = 21.3 ms 自动完成。
            time.sleep(0.1)
            status_lines = command(port, "foc_phase_capture status", 0.25, raw)
            if not any("complete" in line and "256/256" in line.replace(" ", "")
                       for line in status_lines):
                raise RuntimeError("capture did not complete 256 samples")
            dump_lines = command(port, "foc_phase_capture dump", 3.0, raw)
            if not any("FPV_META,divider=on" in line for line in dump_lines):
                raise RuntimeError("capture dump mode does not match")
            command(port, "foc_status", 0.5, raw)
        finally:
            # 失败路径也保证停机 + 采集器复位。
            best_effort_command(port, "foc_phase_capture stop", 0.1, raw)
            best_effort_command(port, "foc_stop", 0.2, raw)
            best_effort_command(port, "foc_cfg closedloop 0", 0.1, raw)
            if original_timing is not None:
                old_rpm, old_align_ms, old_ramp_ms = original_timing
                best_effort_command(port, f"foc_cfg startup {int(old_rpm)}", 0.1, raw)
                best_effort_command(port, f"foc_cfg align {old_align_ms}", 0.1, raw)
                best_effort_command(port, f"foc_cfg ramp {old_ramp_ms}", 0.1, raw)

    # 解析整个会话缓冲里的 FPV 样本（与 capture_phase_voltage_window.py 相同）。
    raw_fields = ["sequence", "phase_u_raw", "phase_v_raw", "phase_w_raw",
                  "current_u_raw", "current_v_raw", "bus_last_raw"]
    samples: list[dict[str, int]] = []
    for line in raw:
        marker = line.find("FPV,")
        if marker < 0:
            continue
        values = line[marker:].split(",")[1:]
        if len(values) != len(raw_fields):
            continue
        try:
            samples.append(dict(zip(raw_fields, (int(v, 0) for v in values))))
        except ValueError:
            continue

    log_path.write_text("\n".join(raw) + "\n", encoding="utf-8")
    if len(samples) != WINDOW_SAMPLES:
        raise RuntimeError(f"expected {WINDOW_SAMPLES} samples, got {len(samples)}")
    if [s["sequence"] for s in samples] != list(range(WINDOW_SAMPLES)):
        raise RuntimeError("sample sequence is not contiguous 0..255")

    csv_fields = ["divider_mode", *raw_fields]
    for sample in samples:
        sample["divider_mode"] = "on"
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        import csv
        writer = csv.DictWriter(stream, fieldnames=csv_fields)
        writer.writeheader()
        writer.writerows(samples)

    summary = {
        "format_version": 2,
        "capture_kind": "bemf-post-stop-window",
        "purpose": "settle the SC_KE unit convention with one open-circuit BEMF measurement",
        "requested_rpm": args.rpm,
        "alignment_s": args.alignment_s,
        "open_loop_ramp_s": args.ramp_s,
        "hold_s": args.hold_s,
        "command_to_stop_s": total_spin_s + 0.25,
        "stop_to_arm_host_ms": stop_to_arm_host_ms,
        "runtime_timing_before_capture": {
            "startup_rpm": original_timing[0],
            "alignment_ms": original_timing[1],
            "ramp_ms": original_timing[2],
        },
        "observer_speed_estimate_rpm_at_hold": speed_estimate,
        "sample_rate_hz": SAMPLE_RATE_HZ,
        "samples": len(samples),
        "nominal_uv_per_count": NOMINAL_UV_PER_COUNT,
        "model_caveat": "nominal ST divider model, not board-calibrated (A19 pending)",
        "output": str(args.output),
        "output_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "log": str(log_path),
        "log_sha256": hashlib.sha256(log_path.read_bytes()).hexdigest(),
    }
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary["metadata"] = str(metadata_path)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("\nNext: run analyze_bemf_measurement.py on this CSV to settle the Ke convention.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
