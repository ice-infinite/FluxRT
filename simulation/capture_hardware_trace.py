#!/usr/bin/env python3
"""Capture a bounded FluxRT run and always stop the power stage.

FluxRT 实机遥测采集入口（host 侧）：通过串口驱动固件，抓取 `FTR,` 遥测行，并写出与
Rust `foc-bringup-sim` 同构的归一化 CSV，供 `compare_traces.py` 与 MATLAB 叠加使用。
FluxRT hardware-telemetry capture entry point (host side): drives the firmware over
a serial port, collects `FTR,` lines, and writes a normalized CSV with the same
schema as the Rust `foc-bringup-sim` trace.

职责 / Responsibility:
  - 用 `foc_cfg` 下发 SMO / EMF 滤波 / PLL 调参，再启动 trace 与电机，采集固定时长；
  - 把固件的整数定标字段（mA / mV / per-mille / mrad）换算为 SI 单位（A / V / 1 / rad）。
  Pushes SMO / EMF-filter / PLL tuning, starts trace and motor for a bounded
  duration, then converts the integer-scaled fields to SI units.

安全 / Safety:
  - 没有 `--allow-motor-run` 时在打开串口前拒绝运行；
  - `--closed-loop` 只临时开启闭环，退出时恢复为 0；
  - 所有退出路径都必须发送 `foc_stop`，包括解析失败、文件写入失败和异常；
    功率级绝不允许留在使能状态。实现见 `main()` 的 `finally`。
  - `foc_stop` is sent on every exit path, so the power stage is never left enabled
    even when parsing or file output fails. See the `finally` block in `main()`.

输出 / Outputs:
  - `--output`：归一化 CSV，例如 `simulation/results/hardware_582rpm_12v3_30s.csv`；
  - 同名 `.log`：原始串口行（含非 FTR 行），用于事后定位解析失败；
  - 两者都在 `.gitignore` 的 `/simulation/results/` 下，属运行产物，不提交 git。
  Both are runtime artifacts under the git-ignored `/simulation/results/`.

参考 / Reference: docs/仿真实机相关性验证.md §4.2
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
import sys
import time

import serial


# 固件 `FTR,` 数据行的字段顺序与量纲（wire format，逐字节载荷，改动即失配）。
# Field order and units of one firmware `FTR,` data line (load-bearing wire format;
# any reordering or renaming silently shifts or drops columns).
#
# 顺序是 FTR V2 的规范定义；固件用紧凑 `FTR_HEADER,2,31` 标记版本和列数，旧日志
# 可能仍带完整列名。两种 header 都对应下面同一个 31 列顺序。
# Canonical FTR V2 order. Firmware now emits compact `FTR_HEADER,2,31`; archived
# logs may contain the expanded names, and both map to these same 31 fields.
#
# 单位 / Units: *_ma = mA, *_mv = mV, *_pm = per-mille (0..1000 满占空比),
#   *_mrad = mrad, observer_speed_rpm = rpm, step = 12 kHz 控制步数 [cycles],
#   state/reliable/flags = 枚举或位域整数 (enum / bit field)。
# 同一字段在 trace 里是整数定标、在仿真 CSV 里是 SI 浮点，见 normalized_row()。
FIELD_NAMES = [
    "step",
    "state",
    "ia_ma",
    "ib_ma",
    "ic_ma",
    "id_ref_ma",
    "iq_ref_ma",
    "id_ma",
    "iq_ma",
    "vd_mv",
    "vq_mv",
    "duty_a_pm",
    "duty_b_pm",
    "duty_c_pm",
    "vbus_mv",
    "control_angle_mrad",
    "forced_angle_mrad",
    "observer_angle_mrad",
    "observer_speed_rpm",
    "reliable",
    "flags",
    "bemf_alpha_mv",
    "bemf_beta_mv",
    "pll_phase_error_mrad",
    "speed_mean_rpm",
    "speed_variance_rpm2",
    "gate_flags",
    "reliable_samples",
    "observer_wait_ms",
    "observer_loss_ms",
    "voltage_limited",
]

OBSERVER_GATE_WINDOW_READY = 1 << 0
OBSERVER_GATE_SPEED_FINITE = 1 << 1
OBSERVER_GATE_SPEED_ABOVE_MINIMUM = 1 << 2
OBSERVER_GATE_SPEED_BELOW_MAXIMUM = 1 << 3
OBSERVER_GATE_BEMF_ABOVE_MINIMUM = 1 << 4
OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM = 1 << 5
OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM = 1 << 6
OBSERVER_GATE_ALL = (1 << 7) - 1

# 归一化 CSV 的列顺序，是 Python / MATLAB 对比链路的公共契约。
# Column order of the normalized CSV; the shared contract of the Python and MATLAB
# comparison paths, and the reason the two traces can be joined by name alone.
#
# 与仿真侧 `foc-bringup-sim` 导出的 CSV 同列同名，才能按列名对齐。
# Same names and order as the `foc-bringup-sim` CSV so alignment is by column name.
#
# 实机没有编码器/测速仪真值，`true_speed_rpm` / `true_angle_rad` 恒为空字符串；
# 只有仿真侧能用 plant 真值填充这两列（见 compare_traces.py 的 NaN 处理）。
# The board has no independent speed/angle truth, so the `true_*` columns are always
# empty here; only the simulation can fill them from the plant truth.
CSV_NAMES = [
    "time_s",
    "step",
    "state",
    "phase_current_a",
    "phase_current_b",
    "phase_current_c",
    "target_speed_rpm",
    "observer_speed_rpm",
    "true_speed_rpm",
    "id_ref_a",
    "iq_ref_a",
    "id_a",
    "iq_a",
    "vd_v",
    "vq_v",
    "duty_a",
    "duty_b",
    "duty_c",
    "dc_bus_voltage_v",
    "control_angle_rad",
    "forced_angle_rad",
    "observer_angle_rad",
    "true_angle_rad",
    "observer_reliable",
    "flags",
    "observer_bemf_alpha_v",
    "observer_bemf_beta_v",
    "observer_bemf_magnitude_v",
    "observer_pll_phase_error_rad",
    "observer_speed_mean_rpm",
    "observer_speed_variance_rpm2",
    "observer_speed_variance_ratio",
    "observer_reliability_flags",
    "observer_reliable_samples",
    "observer_wait_elapsed_s",
    "observer_loss_elapsed_s",
    "voltage_limited",
]


# 命令行契约 / CLI contract:
#   --port / --baud    串口与波特率，默认 COM6 / 115200（固件 shell 在 LPUART1）
#   --duration         采集时长 [s]，必须 > 0；默认 5.0（文档另有 30 s 长测）
#   --target-rpm       速度目标 [rpm]，默认 582.0，原样透传给 `foc_start`
#   --rate-hz          trace 抽稀率 [Hz]，只允许 10..50；不改变 12 kHz 控制频率
#   --control-hz       完整控制频率 [Hz]，默认 12000；只用于 step -> 秒换算；
#                      兼容旧命令行别名 --pwm-hz
#   --closed-loop      本次临时 foc_cfg closedloop 1，退出时恢复为 0
#   --allow-motor-run  显式上电安全闸门；缺少时拒绝打开串口
#   --prestart-settle-s 先停机后等待转子静止的时间 [s]，默认 8.0；本机空载低摩擦
#                       转子在 2 s 连续重启测试中仍可能旋转，而当前固件不含 flying-start
#   --observer-backend 选择 smo 或 bemf 后端；未给出时保留固件当前值
#   --smo-slide-v      SMO 滑模增益 [V]，下发时 ×1000 转成固件要求的 mV
#   --smo-boundary-a   SMO 边界层 [A]，下发时 ×1000 转成固件要求的 mA
#   --emf-filter       EMF 滤波系数 [无量纲 0..1]，下发时 ×1000 转成 per-mille
#   --pll-kp/--pll-ki  PLL 增益，无量纲，原值下发（不缩放）
#   --pll-acq-ratio    捕获期 Kp/Ki 校正比例，0..1；终速前馈不缩放
#   --acquire-phase-rad 启动获取窗平均绝对包角误差上限 [rad]，下发时转成 mrad
#   --run-phase-rad    接管后保持最大原始包角误差 [rad]，下发时转成 mrad
#   --confirm-ms       接管前所有门连续成立时间 [ms]；观测门按 1 kHz 评估
#   --acquire-ms       开环保持等待观测器收敛的超时 [ms]
#   --loss-ms          接管后观测器连续失锁超时 [ms]
#   --alignment-ms     Rev-Up 转子对齐时间 [ms]；必须先停机
#   --ramp-ms          Rev-Up 从 0 到终速的开环升速时间 [ms]；必须先停机
#   --startup-speed-rpm Rev-Up 无感获取终速幅值 [rpm]；可高于最终目标
#   --alignment-current-a 对齐结束 q 轴电流 [A]；升速时递减到 startup current
#   --startup-current-a 强拖 q 轴电流 [A]，下发时转成 mA；必须先停机
#   --handoff-support-ratio 接管转矩支撑比例，0..1，下发时转成 per-mille
#   --speed-preload-ratio 速度 PI 积分预装比例，0..1，下发时转成 per-mille
#   --iq-slew-a-per-s  闭环 Iq 给定限速 [A/s]，下发时转成 mA/s
#   --output           归一化 CSV 路径，必填
# 只有 --output 必填，其余都有默认值；未给出（None）的调参项不会下发。
# Only --output is mandatory; tuning options left at None are not sent at all.
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--target-rpm", type=float, default=582.0)
    parser.add_argument("--rate-hz", type=int, default=50)
    parser.add_argument("--control-hz", "--pwm-hz", dest="control_hz", type=int, default=12000)
    parser.add_argument("--closed-loop", action="store_true")
    parser.add_argument("--allow-motor-run", action="store_true")
    parser.add_argument("--prestart-settle-s", type=float, default=8.0)
    parser.add_argument("--observer-backend", choices=("smo", "bemf"))
    parser.add_argument("--smo-slide-v", type=float)
    parser.add_argument("--smo-boundary-a", type=float)
    parser.add_argument("--emf-filter", type=float)
    parser.add_argument("--pll-kp", type=float)
    parser.add_argument("--pll-acq-ratio", type=float)
    parser.add_argument("--pll-ki", type=float)
    parser.add_argument("--acquire-phase-rad", type=float)
    parser.add_argument("--run-phase-rad", type=float)
    parser.add_argument("--confirm-ms", type=int)
    parser.add_argument("--acquire-ms", type=int)
    parser.add_argument("--loss-ms", type=int)
    parser.add_argument("--transition-ms", type=int)
    parser.add_argument("--alignment-ms", type=int)
    parser.add_argument("--ramp-ms", type=int)
    parser.add_argument("--startup-speed-rpm", type=float)
    parser.add_argument("--alignment-current-a", type=float)
    parser.add_argument("--startup-current-a", type=float)
    parser.add_argument("--handoff-support-ratio", type=float)
    parser.add_argument("--speed-preload-ratio", type=float)
    parser.add_argument("--iq-slew-a-per-s", type=float)
    parser.add_argument(
        "--inverter-stage",
        type=int,
        choices=range(0, 5),
        help="target inverter model stage: 0=off, 2=observer correction, 4=observer+PWM",
    )
    parser.add_argument("--output", type=pathlib.Path, required=True)
    return parser.parse_args()


# 向 shell 写一条命令并立刻 flush。必须以 CRLF 结尾：固件 shell 以 `\r\n` 断行。
# Writes one shell command and flushes immediately. The CRLF terminator is required;
# the firmware shell only recognises `\r\n`.
def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


# 非阻塞地取出当前已到达的整行：既返回给调用方解析，也追加进 raw_lines。
# raw_lines 会原样落盘成 `.log`，用于采集失败后复盘（FTR 之外的 shell 提示、
# REFUSED、dropped 统计都只存在于这里）。
# Non-blocking drain of complete lines. Lines go both to the caller and to
# `raw_lines`, which is written verbatim as the `.log`; shell banners, REFUSED
# notices and dropped counters exist only there.
#
# 行解码用 errors="replace"，串口噪声或半行截断不会让采集整体失败。
# Decoding uses errors="replace" so line noise or a truncated read cannot abort a run.
def read_available(port: serial.Serial, raw_lines: list[str]) -> list[str]:
    lines: list[str] = []
    while port.in_waiting:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if line:
            raw_lines.append(line)
            lines.append(line)
    return lines


# 解析一行 `FTR,` 遥测，返回 31 个整数定标字段；否则返回 None。
# Parses one `FTR,` telemetry line into the 31 integer fields, or None.
#
# 非 FTR 行、字段数不符、任一字段非法都"静默跳过"而不抛异常，这样 shell 的
# 横幅/错误行不会打断整段采集；代价是同行若混入其他输出会被整行丢弃。
# Non-FTR lines, wrong field counts and unparsable fields are skipped instead of
# raising, so banners and error lines cannot abort the capture; the cost is that
# a line polluted by other output is dropped whole.
#
# int(value, 0) 同时接受十进制和 0x 前缀十六进制（固件把 flags 以 0x%08x 打印）。
# int(value, 0) accepts decimal and 0x-prefixed hex alike, as flags are documented
# in hexadecimal.
def parse_trace(line: str) -> dict[str, int] | None:
    marker = line.find("FTR,")
    if marker < 0:
        return None
    values = line[marker:].split(",")[1:]
    if len(values) != len(FIELD_NAMES):
        return None
    try:
        return dict(zip(FIELD_NAMES, (int(value, 0) for value in values)))
    except ValueError:
        return None


# 把一条固件整数样本映射为归一化 CSV 行：列名与顺序 = CSV_NAMES，所有单位换算集中在此。
# Maps one firmware integer sample onto a normalized CSV row. Column names and order
# follow CSV_NAMES, and every unit conversion happens here and nowhere else.
def normalized_row(sample: dict[str, int], control_hz: int, target: float) -> dict[str, object]:
    bemf_alpha_v = sample["bemf_alpha_mv"] / 1000.0
    bemf_beta_v = sample["bemf_beta_mv"] / 1000.0
    speed_mean_rpm = sample["speed_mean_rpm"]
    variance_ratio: float | str = ""
    if abs(speed_mean_rpm) > 1.0e-6:
        variance_ratio = sample["speed_variance_rpm2"] / (speed_mean_rpm * speed_mean_rpm)
    return {
        # 实时步数在 PWM 输出使能前一刻清零，因此 step 是仿真与实机共用的时间原点；
        # 换算用控制频率（默认 12000 Hz）而不是 trace 抽稀率 [s = step / f_pwm]。
        # Realtime step count is reset immediately before the PWM outputs are armed,
        # so it is the common simulation/hardware time origin.
        "time_s": sample["step"] / control_hz,
        "step": sample["step"],
        "state": sample["state"],
        # 相电流 mA -> A；定标系数与固件一致，改动会破坏与仿真 CSV 的量纲一致性。
        # Phase currents mA -> A; the scale must stay consistent with the sim CSV.
        "phase_current_a": sample["ia_ma"] / 1000.0,
        "phase_current_b": sample["ib_ma"] / 1000.0,
        "phase_current_c": sample["ic_ma"] / 1000.0,
        # 目标是"下发的给定值"而不是测量值；实机没有真值转速可用。
        # The target is the commanded setpoint, not a measurement; the board has no
        # independent true-speed channel.
        "target_speed_rpm": target,
        "observer_speed_rpm": sample["observer_speed_rpm"],
        # 留空表示"本平台测不到"；读取侧转成 NaN，且该列不在 COMPARE_FIELDS 内，
        # 因此不会污染差值指标（只有仿真侧有 plant 真值可画）。
        # Empty means "not measurable on this board"; the reader maps it to NaN and the
        # column is not in COMPARE_FIELDS, so it never enters the diff metrics.
        "true_speed_rpm": "",
        "id_ref_a": sample["id_ref_ma"] / 1000.0,
        "iq_ref_a": sample["iq_ref_ma"] / 1000.0,
        "id_a": sample["id_ma"] / 1000.0,
        "iq_a": sample["iq_ma"] / 1000.0,
        # dq 电压 mV -> V。
        # dq voltages mV -> V.
        "vd_v": sample["vd_mv"] / 1000.0,
        "vq_v": sample["vq_mv"] / 1000.0,
        # 占空比 per-mille(0..1000) -> 标幺(0..1)，与 MATLAB 的 ylim([0 1]) 对齐。
        # Duty per-mille(0..1000) -> per-unit(0..1), matching MATLAB's ylim([0 1]).
        "duty_a": sample["duty_a_pm"] / 1000.0,
        "duty_b": sample["duty_b_pm"] / 1000.0,
        "duty_c": sample["duty_c_pm"] / 1000.0,
        # 母线电压 mV -> V（文档测试条件约 12.3 V）。
        # Bus voltage mV -> V (the documented test condition is about 12.3 V).
        "dc_bus_voltage_v": sample["vbus_mv"] / 1000.0,
        # 三个角度必须分开：control=本次 Park 实际使用的角度，forced=Rev-Up 强制角，
        # observer=SMO/PLL 估算角。混用会把"控制角"和"观察角"当成同一个量。
        # Three distinct angles: control = the angle actually used by Park, forced =
        # the Rev-Up ramp angle, observer = the SMO/PLL estimate. Conflating them
        # hides exactly the error this toolchain is meant to measure.
        "control_angle_rad": sample["control_angle_mrad"] / 1000.0,
        "forced_angle_rad": sample["forced_angle_mrad"] / 1000.0,
        "observer_angle_rad": sample["observer_angle_mrad"] / 1000.0,
        # 实机无独立角度真值，同 true_speed_rpm。
        # No independent angle truth on the board, same as true_speed_rpm.
        "true_angle_rad": "",
        "observer_reliable": sample["reliable"],
        "flags": sample["flags"],
        "observer_bemf_alpha_v": bemf_alpha_v,
        "observer_bemf_beta_v": bemf_beta_v,
        "observer_bemf_magnitude_v": math.hypot(bemf_alpha_v, bemf_beta_v),
        "observer_pll_phase_error_rad": sample["pll_phase_error_mrad"] / 1000.0,
        "observer_speed_mean_rpm": speed_mean_rpm,
        "observer_speed_variance_rpm2": sample["speed_variance_rpm2"],
        "observer_speed_variance_ratio": variance_ratio,
        "observer_reliability_flags": sample["gate_flags"],
        "observer_reliable_samples": sample["reliable_samples"],
        "observer_wait_elapsed_s": sample["observer_wait_ms"] / 1000.0,
        "observer_loss_elapsed_s": sample["observer_loss_ms"] / 1000.0,
        "voltage_limited": sample["voltage_limited"],
    }


def observer_summary(samples: list[dict[str, int]]) -> str:
    """Summarize gate failures while the observer owns or is acquiring the angle."""
    active = [sample for sample in samples if sample["state"] in (5, 6, 7)]
    if not active:
        return "FOC_HW_OBSERVER_UNVERIFIED reason=no_handoff_samples"
    gate_bits = (
        ("window", OBSERVER_GATE_WINDOW_READY),
        ("finite", OBSERVER_GATE_SPEED_FINITE),
        ("minspeed", OBSERVER_GATE_SPEED_ABOVE_MINIMUM),
        ("maxspeed", OBSERVER_GATE_SPEED_BELOW_MAXIMUM),
        ("bemf", OBSERVER_GATE_BEMF_ABOVE_MINIMUM),
        ("variance", OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM),
        ("phase", OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM),
    )
    missing = {
        name: sum((sample["gate_flags"] & bit) == 0 for sample in active)
        for name, bit in gate_bits
    }
    last = active[-1]
    bemf_v = math.hypot(last["bemf_alpha_mv"], last["bemf_beta_mv"]) / 1000.0
    mean = last["speed_mean_rpm"]
    ratio = (
        last["speed_variance_rpm2"] / (mean * mean)
        if abs(mean) > 1.0e-6
        else math.inf
    )
    missing_text = "/".join(f"{name}:{count}" for name, count in missing.items())
    return (
        "FOC_HW_OBSERVER_SUMMARY "
        f"samples={len(active)} last_gate=0x{last['gate_flags']:02x}/0x{OBSERVER_GATE_ALL:02x} "
        f"missing={missing_text} bemf={bemf_v:.3f}V pllerr={last['pll_phase_error_mrad'] / 1000.0:.3f}rad "
        f"speed_mean={mean}rpm variance_ratio={ratio:.6f} streak={last['reliable_samples']} "
        f"wait={last['observer_wait_ms']}ms loss={last['observer_loss_ms']}ms "
        f"vlim={sum(sample['voltage_limited'] != 0 for sample in active)}/{len(active)}"
    )


def parse_final_run_status(raw_lines: list[str]) -> tuple[int, int, int, int] | None:
    """Return final errors, deadline misses, control status and Rust fault flags.

    `foc_status` is requested only after `foc_stop`, so the last matching lines are
    the authoritative post-run result.  Keeping this separate from FTR parsing
    prevents a healthy serial stream from being mistaken for a healthy controller.
    """
    errors: int | None = None
    deadline_misses: int | None = None
    control_status: int | None = None
    rust_fault: int | None = None
    for line in raw_lines:
        diagnostics_match = re.search(r"\berrors=(\d+)\b", line)
        if diagnostics_match is not None:
            errors = int(diagnostics_match.group(1), 10)
        misses_match = re.search(r"\bmisses=(\d+)\b", line)
        if misses_match is not None:
            deadline_misses = int(misses_match.group(1), 10)
        compact_diagnostics_match = re.search(
            r"\bFOC f=[0-9a-fA-F]+ s=\d+ e=(\d+) ISR=\d+/\d+ miss=(\d+)\b",
            line,
        )
        if compact_diagnostics_match is not None:
            errors = int(compact_diagnostics_match.group(1), 10)
            deadline_misses = int(compact_diagnostics_match.group(2), 10)
        status_match = re.search(
            r"\bFOC last status=(\d+) rust_fault=0x([0-9a-fA-F]+)\b", line
        )
        if status_match is None:
            status_match = re.search(
                r"\bFOC st=(\d+) rf=([0-9a-fA-F]+)\b", line
            )
        if status_match is not None:
            control_status = int(status_match.group(1), 10)
            rust_fault = int(status_match.group(2), 16)
    if (
        errors is None
        or deadline_misses is None
        or control_status is None
        or rust_fault is None
    ):
        return None
    return errors, deadline_misses, control_status, rust_fault


# 采集主流程。顺序与安全和数据都相关：先停机 -> 下发调参 -> 起 trace -> 起电机 ->
# 采集 -> 在 finally 里停机。trace 必须先于电机启动，才能拿到版本 header 和 alignment 段。
# Capture flow. The ordering is safety- and data-relevant: stop, tune, start trace,
# start motor, capture, then stop inside `finally`. Trace starts before the motor so
# the versioned `FTR_HEADER,` marker and the alignment phase are never missed.
def main() -> int:
    args = parse_args()
    # 先做参数自检再碰硬件：非法时长/抽稀率不应改变串口与电源状态。
    # Validate arguments before touching hardware, so a bad duration or rate cannot
    # change the state of the port or the power stage.
    if not args.allow_motor_run:
        raise SystemExit("refusing to energize the motor without --allow-motor-run")
    if args.duration <= 0 or not 10 <= args.rate_hz <= 50 or args.control_hz <= 0:
        raise SystemExit("duration/control-hz must be positive and rate must be 10..50 Hz")
    if not 0.0 <= args.prestart_settle_s <= 30.0:
        raise SystemExit("prestart-settle-s must be in [0, 30]")
    if args.pll_acq_ratio is not None and not 0.0 < args.pll_acq_ratio <= 1.0:
        raise SystemExit("pll-acq-ratio must be in (0, 1]")
    if args.acquire_phase_rad is not None and not 0.0 < args.acquire_phase_rad <= math.pi / 2.0:
        raise SystemExit("acquire-phase-rad must be in (0, pi/2]")
    if args.run_phase_rad is not None and not 0.0 < args.run_phase_rad <= math.pi / 2.0:
        raise SystemExit("run-phase-rad must be in (0, pi/2]")
    if args.confirm_ms is not None and not 1 <= args.confirm_ms <= 1000:
        raise SystemExit("confirm-ms must be in [1, 1000]")
    if args.acquire_ms is not None and not 1 <= args.acquire_ms <= 10_000:
        raise SystemExit("acquire-ms must be in [1, 10000]")
    if args.loss_ms is not None and not 1 <= args.loss_ms <= 5_000:
        raise SystemExit("loss-ms must be in [1, 5000]")
    if args.transition_ms is not None and not 1 <= args.transition_ms <= 5_000:
        raise SystemExit("transition-ms must be in [1, 5000]")
    if args.alignment_ms is not None and not 100 <= args.alignment_ms <= 10_000:
        raise SystemExit("alignment-ms must be in [100, 10000]")
    if args.ramp_ms is not None and not 100 <= args.ramp_ms <= 10_000:
        raise SystemExit("ramp-ms must be in [100, 10000]")
    if args.startup_speed_rpm is not None and not 1.0 <= args.startup_speed_rpm <= 10_000.0:
        raise SystemExit("startup-speed-rpm must be in [1, 10000]")
    if args.alignment_current_a is not None and not 0.01 <= args.alignment_current_a <= 10.0:
        raise SystemExit("alignment-current-a must be in [0.01, 10]")
    if args.speed_preload_ratio is not None and not 0.0 <= args.speed_preload_ratio <= 1.0:
        raise SystemExit("speed-preload-ratio must be in [0, 1]")
    if args.iq_slew_a_per_s is not None and not 0.01 <= args.iq_slew_a_per_s <= 1_000.0:
        raise SystemExit("iq-slew-a-per-s must be in [0.01, 1000]")

    samples: list[dict[str, int]] = []
    raw_lines: list[str] = []
    args.output.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0) as port:
        # 适配器/USB-CDC 上电后需要稳定时间；清空输入缓冲丢掉上电横幅，
        # 然后立刻 foc_stop，把"未知的初始状态"变成确定的已停机状态。
        # USB-CDC needs settling time; drop the boot banner and then force a known-safe
        # state with foc_stop before configuring anything.
        time.sleep(0.25)
        port.reset_input_buffer()
        send(port, "foc_stop")
        # Resetting the forced angle while the unloaded rotor is still coasting is
        # a flying-start test, not a repeatable cold start. Wait after the hard stop
        # so batch captures begin from a stationary rotor.
        time.sleep(args.prestart_settle_s)
        read_available(port, raw_lines)
        if args.closed_loop:
            send(port, "foc_cfg closedloop 1")
            time.sleep(0.08)
            read_available(port, raw_lines)
        # 固件要求"必须先停机才能改配置"（否则 foc_cfg 直接 REFUSED），
        # 所以这个顺序不可调换；每项之间留 0.08 s 让 shell 处理命令。
        # Firmware refuses foc_cfg unless the stage is stopped ("run foc_stop first"),
        # so this order is mandatory and the spacing lets the shell run each command.
        if args.observer_backend is not None:
            send(port, f"foc_cfg observer {args.observer_backend}")
            time.sleep(0.08)
            read_available(port, raw_lines)
        #
        # scale 把 CLI 的物理量换成固件要求的整数定标：V -> mV、A -> mA、
        # 无量纲系数 -> per-mille；PLL 增益不缩放（scale = 1.0）。
        # scale converts the CLI quantity into the firmware's integer scaling:
        # V -> mV, A -> mA, dimensionless -> per-mille; PLL gains are unscaled.
        tuning_commands = (
            ("slide", args.smo_slide_v, 1000.0),
            ("boundary", args.smo_boundary_a, 1000.0),
            ("filter", args.emf_filter, 1000.0),
            ("pll_kp", args.pll_kp, 1.0),
            ("pll_acq", args.pll_acq_ratio, 1000.0),
            ("pll_ki", args.pll_ki, 1.0),
            ("acqphase", args.acquire_phase_rad, 1000.0),
            ("runphase", args.run_phase_rad, 1000.0),
            ("confirm", args.confirm_ms, 1.0),
            ("acquire", args.acquire_ms, 1.0),
            ("loss", args.loss_ms, 1.0),
            ("transition", args.transition_ms, 1.0),
            ("align", args.alignment_ms, 1.0),
            ("ramp", args.ramp_ms, 1.0),
            ("startup", args.startup_speed_rpm, 1.0),
            ("aligncurrent", args.alignment_current_a, 1000.0),
            ("current", args.startup_current_a, 1000.0),
            ("support", args.handoff_support_ratio, 1000.0),
            ("preload", args.speed_preload_ratio, 1000.0),
            ("islew", args.iq_slew_a_per_s, 1000.0),
        )
        for name, value, scale in tuning_commands:
            if value is not None:
                send(port, f"foc_cfg {name} {value * scale:g}")
                time.sleep(0.08)
                read_available(port, raw_lines)
        if args.inverter_stage is not None:
            send(port, f"foc_cfg invstage {args.inverter_stage}")
            time.sleep(0.08)
            read_available(port, raw_lines)
        # trace 可在停机状态启动（PWM 频率在 TIM1 初始化时就已固定），
        # 这样 FTR_HEADER 与上电瞬间的首批样本都不会丢。
        # Trace can start while stopped because the PWM frequency is fixed during TIM1
        # init; the header and the first samples right after arming are therefore kept.
        send(port, f"foc_trace start {args.rate_hz}")
        time.sleep(0.15)
        send(port, f"foc_start {args.target_rpm:g}")
        deadline = time.monotonic() + args.duration
        try:
            while time.monotonic() < deadline:
                for line in read_available(port, raw_lines):
                    sample = parse_trace(line)
                    if sample is not None:
                        samples.append(sample)
                # 5 ms 轮询远快于 50 Hz 遥测流，OS 调度抖动不会造成丢样本。
                # 5 ms polling is far faster than the 50 Hz stream, so OS scheduling
                # jitter cannot drop samples.
                time.sleep(0.005)
        finally:
            # 安全路径：先停机、再关 trace。即使采集循环里抛异常（串口错误、解码异常、
            # 键盘中断），这里也必须执行，保证功率级被关断；
            # CSV 写入发生在 with 之外，那时电机已经停了。
            # Safety path: stop the motor first, disable trace second. It runs even when
            # the capture loop raises (serial error, decode failure, keyboard interrupt),
            # so the power stage is always left disabled; the CSV write happens outside the
            # `with` block, by which time the motor is already stopped.
            send(port, "foc_stop")
            time.sleep(0.15)
            # 停机后再排空一次串口缓冲，收全停机前已产生的样本，
            # 但不因此延长电机通电时间。
            # Drain once more after the stop to collect already-queued samples without
            # extending the time the motor is energized.
            for line in read_available(port, raw_lines):
                sample = parse_trace(line)
                if sample is not None:
                    samples.append(sample)
            send(port, "foc_trace stop")
            if args.closed_loop:
                send(port, "foc_cfg closedloop 0")
            send(port, "foc_status")
            time.sleep(0.25)
            read_available(port, raw_lines)

    if not samples:
        # 解析不出样本时仍先写出原始日志再失败：没有串口记录就无法区分是接线、
        # 波特率、foc_start REFUSED，还是 Production 档把 trace 裁掉了。
        # Still write the raw log before failing: without it there is no way to tell a
        # wiring/baud problem from a REFUSED start or a build without trace.
        args.output.with_suffix(".log").write_text("\n".join(raw_lines), encoding="utf-8")
        raise SystemExit("no FTR samples received; raw log was saved")

    first_step = samples[0]["step"]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_NAMES)
        writer.writeheader()
        for sample in samples:
            writer.writerow(normalized_row(sample, args.control_hz, args.target_rpm))
    log_path = args.output.with_suffix(".log")
    log_path.write_text("\n".join(raw_lines) + "\n", encoding="utf-8")

    # FOC_HW_* 行供 CI 与上位脚本解析，是稳定接口，不要改动既有标记名。
    # CAPTURE_PASS 只代表采集链路拿到了样本；RUN_PASS 还要求停机后诊断无错误，
    # 并且请求闭环时至少实际出现一次状态 7。只看 0 fault 会把“仍在开环保持等待”
    # 误报为闭环通过。
    # The FOC_HW_* markers are a stable interface. CAPTURE_PASS only proves that
    # samples arrived; RUN_PASS additionally requires clean post-stop diagnostics.
    print(f"FOC_HW_TRACE={args.output}")
    print(f"FOC_HW_RAW_LOG={log_path}")
    # elapsed 用首末 step 差而不是墙钟：墙钟会被串口阻塞与调度误差污染。
    # elapsed uses the step delta, not wall-clock time, which serial blocking and
    # scheduling jitter would corrupt.
    print(
        "FOC_HW_CAPTURE_PASS "
        f"samples={len(samples)} elapsed={(samples[-1]['step'] - first_step) / args.control_hz:.3f}s "
        f"reliable={sum(sample['reliable'] != 0 for sample in samples)}"
    )
    print(observer_summary(samples))
    run_status = parse_final_run_status(raw_lines)
    if run_status is None:
        print("FOC_HW_RUN_UNVERIFIED reason=missing_final_status")
        return 3
    errors, deadline_misses, control_status, rust_fault = run_status
    if errors != 0 or deadline_misses != 0 or control_status == 4 or rust_fault != 0:
        print(
            "FOC_HW_RUN_FAIL "
            f"errors={errors} deadline_misses={deadline_misses} "
            f"control_status={control_status} rust_fault=0x{rust_fault:08x}"
        )
        return 2
    if args.closed_loop and not any(sample["state"] == 7 for sample in samples):
        print(
            "FOC_HW_RUN_FAIL reason=closed_loop_not_reached "
            f"final_state={samples[-1]['state']} errors={errors} "
            f"deadline_misses={deadline_misses} control_status={control_status} "
            f"rust_fault=0x{rust_fault:08x}"
        )
        return 2
    print(
        "FOC_HW_RUN_PASS "
        f"errors={errors} deadline_misses={deadline_misses} "
        f"control_status={control_status} rust_fault=0x{rust_fault:08x}"
    )
    return 0


# 串口异常统一转成 SystemExit 文本，让调用方靠退出码判断，而不是读 Python 回溯。
# Serial failures become a plain SystemExit message so callers can judge by exit code.
if __name__ == "__main__":
    try:
        sys.exit(main())
    except serial.SerialException as error:
        raise SystemExit(f"serial error: {error}") from error
