#!/usr/bin/env python3
"""Compare same-scenario firmware and host-simulation FOC CSV traces.

FluxRT 同工况数值对比入口：读入份符合归一化 schema 的 CSV（实机与 PC 仿真），
按时间做最近邻配对，输出误差指标、状态切换时间与保持段统计。
FluxRT numeric correlation entry point: reads two CSVs that follow the normalized
schema (hardware and PC simulation), pairs samples by nearest time, and reports error
metrics, state-transition times and hold-segment statistics.

职责 / Responsibility:
  - 计算"实机减仿真"同刻误差（MAE / RMSE / 最大绝对误差）；
  - 计算各自相对参考值的电流跟踪 RMSE 与观察器保持段统计；
  - 生成 JSON（机器可读）与中文 Markdown 表格（人读、纳入 docs/）。
  Computes per-field hardware-minus-simulation errors, per-trace current-tracking RMSE,
  observer hold-segment statistics, and emits JSON plus a Chinese Markdown table.

边界 / Boundary:
  - 只做统计与呈现，不下"通过/不通过"结论；数值是校准依据（见 markdown() 末尾说明）；
  - 不重新仿真、不拟合参数，也不读取串口——采集由 capture_hardware_trace.py 完成。
  Statistics and presentation only; it never declares pass/fail, never re-simulates and
  never touches the serial port.

输入 / Inputs: 两侧 CSV 都由 capture_hardware_trace.py（实机）或 `foc-bringup-sim`
（仿真）产生，列名相同、单位为 SI（A / V / per-mille 已转标幺 / rpm / rad）。
Both inputs use the same column names and SI units, which is what makes name-based
alignment valid.

参考 / Reference: docs/仿真实机相关性验证.md
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics


# 逐个做同刻差值的字段（都是 SI 单位）：电流 [A]、电压 [V]、占空比 [标幺]、估速 [rpm]。
# Fields that get a point-by-point hardware-minus-simulation error, all in SI units:
# currents [A], voltages [V], duty [per-unit], observer speed [rpm].
# 只放"两侧都由同一条控制代码产生"的量；true_* 列实机没有，故不参与。
# Only quantities produced by the same control code on both sides; the true_* columns
# exist only in the simulation and are therefore excluded.
COMPARE_FIELDS = ("id_a", "iq_a", "vd_v", "vq_v", "duty_a", "duty_b", "duty_c", "observer_speed_rpm")
# 状态编号到名字的映射，必须与固件 `foc_state_name()` 的枚举保持一致。
# State id to name; must stay in sync with the firmware `foc_state_name()` enum.
# 3..8 = alignment / open-loop-ramp / open-loop-hold / observer-transition /
#        closed-loop / fault，报告里的状态切换时间依赖这些名字。
STATE_NAMES = {3: "alignment", 4: "open-loop-ramp", 5: "open-loop-hold", 6: "observer-transition", 7: "closed-loop", 8: "fault"}


# 命令行契约 / CLI contract:
#   --simulation  PC 仿真 CSV（必填），通常 simulation/results/bringup_sim_*.csv
#   --hardware    实机 CSV（必填），通常 simulation/results/hardware_*.csv
#   --json        可选；机器可读结果，惯例写到 simulation/results/（已被 git 忽略）
#   --markdown    可选；中文表格，惯例写到 docs/simulation/（会提交 git）
# 两个输入都必填；输出都是可选的，但正文永远打印到 stdout。
# Both inputs are mandatory; both outputs are optional, and the JSON is always printed
# to stdout regardless.
def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation", type=pathlib.Path, required=True)
    parser.add_argument("--hardware", type=pathlib.Path, required=True)
    parser.add_argument("--json", type=pathlib.Path)
    parser.add_argument("--markdown", type=pathlib.Path)
    return parser.parse_args()


# 读取一份 CSV 并把所有列转成 float。
# Loads a CSV and converts every column to float.
#
# 空字段（""）转成 NaN，而不是报错：实机 CSV 的 true_speed_rpm / true_angle_rad
# 本来就是空的，NaN 让"测不到"与"数值为 0"在使用时能被区分。
# Empty fields become NaN instead of raising: the hardware CSV leaves the true_* columns
# empty, and NaN keeps "not measured" distinguishable from a real zero.
#
# 空文件直接抛错——两份空轨迹做差值只会得到一堆无意义的 NaN。
# An empty trace raises, because diffing two empty traces would only produce noise.
def load(path: pathlib.Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for row in csv.DictReader(stream):
            converted: dict[str, float] = {}
            for key, value in row.items():
                converted[key] = float(value) if value not in (None, "") else math.nan
            rows.append(converted)
    if not rows:
        raise ValueError(f"empty trace: {path}")
    return rows


# 时间对齐：为每个实机样本找时间上最近的仿真样本，返回成对列表。
# Time alignment: for every hardware sample, pick the nearest-in-time simulation sample.
#
# 对齐假设 / Assumptions:
#   - 两条 time_s 都单调不减，且以"控制使能那一刻"为零点（固件在使能 PWM 前清零 step，
#     仿真从 tick 0 起算，均按控制周期换算成秒）。原点若不一致，所有指标会被系统性
#     偏移污染，而本函数不做时移或互相关；
#   - 仿真采样通常比实机密（sample-every 320 -> 约 37.5 Hz），因此这里用"单向前进的
#     游标"，复杂度 O(n+m)，而不做插值。
#   - Both time axes are monotonic and start when control is enabled; the cursor only
#     moves forward, giving O(n+m) without interpolation. Any real origin mismatch would
#     bias every metric, and this function applies no shift or cross-correlation.
#
# 失败模式 / Failure mode: 实机时间超出仿真末尾时游标停在最后一个仿真样本，
# 后续配对全部复用该样本 —— 因此两侧时长差距过大时，末尾的 RMSE 会被拉高，
# 应把它读作"仿真没覆盖到"，而不是物理误差。
# Once the hardware time passes the end of the simulation, the cursor pins on the last
# simulation sample, so a duration mismatch inflates the tail metrics; read that as
# "the simulation did not cover this", not as a physical error.
def nearest_pairs(sim: list[dict[str, float]], hardware: list[dict[str, float]]) -> list[tuple[dict[str, float], dict[str, float]]]:
    pairs = []
    index = 0
    for measured in hardware:
        while index + 1 < len(sim) and abs(sim[index + 1]["time_s"] - measured["time_s"]) < abs(sim[index]["time_s"] - measured["time_s"]):
            index += 1
        pairs.append((sim[index], measured))
    return pairs


# 每个状态第一次出现的时刻 [s]，是"启动时序是否一致"的判据。
# First time [s] each state appears, the evidence for start-sequence agreement.
#
# 只记首次出现：状态若在运行中再次进入（例如跌回 open-loop），这里看不出来，
# 需要看原始曲线；未知编号退化为数字字符串，保证不丢样本。
# Only the first occurrence is recorded, so a later re-entry is invisible here and must
# be read from the curves; unknown ids fall back to their numeric string.
def state_transitions(rows: list[dict[str, float]]) -> dict[str, float]:
    transitions: dict[str, float] = {}
    for row in rows:
        state = int(row["state"])
        name = STATE_NAMES.get(state, str(state))
        transitions.setdefault(name, row["time_s"])
    return transitions


# 误差序列的三个标量摘要。量纲随输入（A / V / rpm），单位必须由调用方记住。
# Three scalar summaries of an error sequence; the unit follows the input (A / V / rpm).
# RMSE 对大幅偏差更敏感，MAE 反映典型偏差，max_abs 用于发现单点尖峰。
# RMSE weights large deviations, MAE shows the typical error, max_abs catches spikes.
def metric(errors: list[float]) -> dict[str, float]:
    return {
        "mae": statistics.fmean(abs(value) for value in errors),
        "rmse": math.sqrt(statistics.fmean(value * value for value in errors)),
        "max_abs": max(abs(value) for value in errors),
    }


# 三相电流的最大绝对值 [A]，用于和固件的高速峰值锁存做量级对照。
# Max absolute phase current [A], used as an order-of-magnitude cross-check against the
# firmware peak latch.
# 这是逐样本取 max，因此是下界：12 kHz 的瞬时峰值不会被 50 Hz CSV 采到，
# 文档中实机 CSV 峰值 0.822 A 低于固件锁存约 0.904 A 正是这个原因。
# Sampling can only under-report: a 12 kHz instantaneous peak is invisible to a 50 Hz
# CSV, which is why the documented CSV peak is below the firmware latch.
def peak_current(rows: list[dict[str, float]]) -> float:
    return max(abs(row[field]) for row in rows for field in ("phase_current_a", "phase_current_b", "phase_current_c"))


# 保持段（rev-up 结束后）的观察器统计，默认从 2.5 s 起算。
# Observer statistics over the hold segment (after rev-up), from 2.5 s by default.
#
# start_s=2.5 对应文档里 alignment/ramp/hold 在约 2.18 s 结束的启动时序，
# 取在此之后的数据才能避开加速过程本身（这是硬编码的经验值，不是识别结果）。
# start_s=2.5 is chosen so the acceleration transient is excluded given the documented
# start sequence (hold reached near 2.18 s); it is an empirical constant, not identified.
#
# 观察角减强制角：先求圆均值（对 sin/cos 取平均再 atan2），避免 ±π 跳变把均值算错；
# 再以该常值偏移为基准算残差抖动。观测角与强制角之间存在恒定偏移是正常现象，
# 因此报告的是"偏移量"和"抖动"，而不是把偏移本身当作误差。
# The observer-minus-forced angle uses a circular mean (mean of sin/cos, then atan2) so a
# ±π wrap cannot corrupt it; jitter is then the RMSE about that constant offset, because
# a fixed offset between observer and forced angle is expected rather than an error.
#
# 单位 / Units: 速度 [rpm]，角度 [rad]，reliable_samples 为 observer_reliable != 0 的计数。
def hold_observer(rows: list[dict[str, float]], start_s: float = 2.5) -> dict[str, float]:
    hold = [row for row in rows if row["time_s"] >= start_s]
    speeds = [row["observer_speed_rpm"] for row in hold]
    angle_errors = [
        (row["observer_angle_rad"] - row["forced_angle_rad"] + math.pi) % (2.0 * math.pi) - math.pi
        for row in hold
    ]
    cosine = statistics.fmean(math.cos(value) for value in angle_errors)
    sine = statistics.fmean(math.sin(value) for value in angle_errors)
    offset = math.atan2(sine, cosine)
    residuals = [(value - offset + math.pi) % (2.0 * math.pi) - math.pi for value in angle_errors]
    return {
        "samples": len(hold),
        "reliable_samples": sum(row["observer_reliable"] != 0 for row in hold),
        "speed_mean_rpm": statistics.fmean(speeds),
        "speed_stdev_rpm": statistics.pstdev(speeds),
        "speed_min_rpm": min(speeds),
        "speed_max_rpm": max(speeds),
        "observer_minus_forced_angle_rad": offset,
        "angle_jitter_rmse_rad": math.sqrt(statistics.fmean(value * value for value in residuals)),
    }


# 汇总成最终结果字典：样本数、时长、状态切换、峰值、可靠样本、保持段统计、
# 各自的跟踪 RMSE，以及"实机减仿真"的逐字段误差。
# Assembles the final result dict: sample counts, durations, state transitions, peaks,
# reliable-sample counts, hold-segment statistics, per-trace tracking RMSE, and the
# per-field hardware-minus-simulation errors.
def analyze(sim: list[dict[str, float]], hardware: list[dict[str, float]]) -> dict[str, object]:
    pairs = nearest_pairs(sim, hardware)
    # 逐字段算"实机 - 仿真"同刻误差 [A]/[V]/[rpm]；符号约定固定为实机减仿真，
    # 便于报告里直接说"实机偏大/偏小"。
    # Per-field hardware-minus-simulation error; the sign convention is fixed so the
    # report can read as "hardware is high/low" without re-deriving it.
    fields = {
        field: metric([measured[field] - predicted[field] for predicted, measured in pairs])
        for field in COMPARE_FIELDS
    }
    # 跟踪误差是各轨迹相对自己的指令值（不是实机对仿真），
    # 因此可以在仿真与实机时长不同的情况下分别评估电流环质量。
    # Tracking error is relative to each trace's own command, so the current loop can be
    # judged on each side independently even when the two durations differ.
    tracking = {
        "simulation_iq_rmse_a": metric([row["iq_a"] - row["iq_ref_a"] for row in sim])["rmse"],
        "hardware_iq_rmse_a": metric([row["iq_a"] - row["iq_ref_a"] for row in hardware])["rmse"],
        "simulation_id_rmse_a": metric([row["id_a"] - row["id_ref_a"] for row in sim])["rmse"],
        "hardware_id_rmse_a": metric([row["id_a"] - row["id_ref_a"] for row in hardware])["rmse"],
    }
    return {
        "simulation_samples": len(sim),
        "hardware_samples": len(hardware),
        "compared_samples": len(pairs),
        "simulation_duration_s": sim[-1]["time_s"],
        "hardware_duration_s": hardware[-1]["time_s"],
        "state_transitions_s": {
            "simulation": state_transitions(sim),
            "hardware": state_transitions(hardware),
        },
        "peak_phase_current_a": {
            "simulation": peak_current(sim),
            "hardware": peak_current(hardware),
        },
        "observer_reliable_samples": {
            "simulation": sum(row["observer_reliable"] != 0 for row in sim),
            "hardware": sum(row["observer_reliable"] != 0 for row in hardware),
        },
        "hold_observer": {
            "simulation": hold_observer(sim),
            "hardware": hold_observer(hardware),
        },
        "tracking": tracking,
        "hardware_minus_simulation": fields,
    }


# 生成中文 Markdown 报告。表格与结尾说明是同一批数字的"给人读"形式。
# Renders the Chinese Markdown report; the tables and closing caveat are the
# human-readable form of the same numbers.
#
# 注意：这里只呈现数值并明确写出"不是自动通过判据"，
# 因为零偏、母线电压、初始转子位置、摩擦/惯量与死区尚未辨识。
# It only presents numbers and states explicitly that they are not automatic pass
# criteria, because offsets, bus voltage, initial rotor position, friction/inertia and
# dead time are not yet identified.
def markdown(result: dict[str, object]) -> str:
    transitions = result["state_transitions_s"]
    peaks = result["peak_phase_current_a"]
    reliable = result["observer_reliable_samples"]
    hold = result["hold_observer"]
    tracking = result["tracking"]
    metrics = result["hardware_minus_simulation"]
    lines = [
        "# FOC 同工况仿真—实机对比结果",
        "",
        f"- 样本：仿真 {result['simulation_samples']}，实机 {result['hardware_samples']}，对齐比较 {result['compared_samples']}",
        f"- 时长：仿真 {result['simulation_duration_s']:.3f} s，实机 {result['hardware_duration_s']:.3f} s",
        f"- 相电流峰值：仿真 {peaks['simulation']:.3f} A，实机 {peaks['hardware']:.3f} A",
        f"- 观察器可靠样本：仿真 {reliable['simulation']}，实机 {reliable['hardware']}",
        "",
        "## 状态切换时间",
        "",
        "| 状态 | 仿真/s | 实机/s |",
        "|---|---:|---:|",
    ]
    for state in ("alignment", "open-loop-ramp", "open-loop-hold", "observer-transition", "closed-loop", "fault"):
        sim_value = transitions["simulation"].get(state)
        hw_value = transitions["hardware"].get(state)
        if sim_value is not None or hw_value is not None:
            lines.append(f"| {state} | {sim_value if sim_value is not None else '-'} | {hw_value if hw_value is not None else '-'} |")
    lines.extend(
        [
            "",
            "## 电流跟踪",
            "",
            "| 指标 | 仿真/A | 实机/A |",
            "|---|---:|---:|",
            f"| Iq 跟踪 RMSE | {tracking['simulation_iq_rmse_a']:.4f} | {tracking['hardware_iq_rmse_a']:.4f} |",
            f"| Id 跟踪 RMSE | {tracking['simulation_id_rmse_a']:.4f} | {tracking['hardware_id_rmse_a']:.4f} |",
            "",
            "## 2.5 s 后观察器保持段",
            "",
            "| 指标 | 仿真 | 实机 |",
            "|---|---:|---:|",
            f"| 可靠样本 | {hold['simulation']['reliable_samples']} / {hold['simulation']['samples']} | {hold['hardware']['reliable_samples']} / {hold['hardware']['samples']} |",
            f"| 估算速度均值/rpm | {hold['simulation']['speed_mean_rpm']:.2f} | {hold['hardware']['speed_mean_rpm']:.2f} |",
            f"| 估算速度标准差/rpm | {hold['simulation']['speed_stdev_rpm']:.2f} | {hold['hardware']['speed_stdev_rpm']:.2f} |",
            f"| 观察角减强制角/rad | {hold['simulation']['observer_minus_forced_angle_rad']:.4f} | {hold['hardware']['observer_minus_forced_angle_rad']:.4f} |",
            f"| 角度抖动 RMSE/rad | {hold['simulation']['angle_jitter_rmse_rad']:.4f} | {hold['hardware']['angle_jitter_rmse_rad']:.4f} |",
            "",
            "## 同时刻实机减仿真误差",
            "",
            "| 信号 | MAE | RMSE | 最大绝对误差 |",
            "|---|---:|---:|---:|",
        ]
    )
    for field, values in metrics.items():
        lines.append(f"| {field} | {values['mae']:.4f} | {values['rmse']:.4f} | {values['max_abs']:.4f} |")
    lines.extend(
        [
            "",
            "> 这些数值是校准依据，不是自动的“通过”判据。相电流零偏、母线电压、初始转子位置、摩擦/惯量和死区尚未辨识时，不能要求逐点重合。",
            "",
        ]
    )
    return "\n".join(lines)


# 入口：无论是否落盘，都把 JSON 打到 stdout，方便管道/CI 直接消费。
# Entry point: the JSON always goes to stdout so pipes and CI can consume it directly.
# --markdown 是唯一面向人的产物，惯例写入 docs/simulation/（会提交 git）；
# --json 惯例写入 simulation/results/（已被 .gitignore 忽略）。
# --markdown is the human artifact and is normally committed under docs/simulation/,
# while --json is normally written to the git-ignored simulation/results/.
def main() -> int:
    args = arguments()
    result = analyze(load(args.simulation), load(args.hardware))
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    print(encoded)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(encoded + "\n", encoding="utf-8")
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(markdown(result), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
