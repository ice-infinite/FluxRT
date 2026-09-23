#!/usr/bin/env python3
"""Compare same-scenario firmware and host-simulation FOC CSV traces."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics


COMPARE_FIELDS = ("id_a", "iq_a", "vd_v", "vq_v", "duty_a", "duty_b", "duty_c", "observer_speed_rpm")
STATE_NAMES = {3: "alignment", 4: "open-loop-ramp", 5: "open-loop-hold", 6: "observer-transition", 7: "closed-loop", 8: "fault"}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation", type=pathlib.Path, required=True)
    parser.add_argument("--hardware", type=pathlib.Path, required=True)
    parser.add_argument("--json", type=pathlib.Path)
    parser.add_argument("--markdown", type=pathlib.Path)
    return parser.parse_args()


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


def nearest_pairs(sim: list[dict[str, float]], hardware: list[dict[str, float]]) -> list[tuple[dict[str, float], dict[str, float]]]:
    pairs = []
    index = 0
    for measured in hardware:
        while index + 1 < len(sim) and abs(sim[index + 1]["time_s"] - measured["time_s"]) < abs(sim[index]["time_s"] - measured["time_s"]):
            index += 1
        pairs.append((sim[index], measured))
    return pairs


def state_transitions(rows: list[dict[str, float]]) -> dict[str, float]:
    transitions: dict[str, float] = {}
    for row in rows:
        state = int(row["state"])
        name = STATE_NAMES.get(state, str(state))
        transitions.setdefault(name, row["time_s"])
    return transitions


def metric(errors: list[float]) -> dict[str, float]:
    return {
        "mae": statistics.fmean(abs(value) for value in errors),
        "rmse": math.sqrt(statistics.fmean(value * value for value in errors)),
        "max_abs": max(abs(value) for value in errors),
    }


def peak_current(rows: list[dict[str, float]]) -> float:
    return max(abs(row[field]) for row in rows for field in ("phase_current_a", "phase_current_b", "phase_current_c"))


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


def analyze(sim: list[dict[str, float]], hardware: list[dict[str, float]]) -> dict[str, object]:
    pairs = nearest_pairs(sim, hardware)
    fields = {
        field: metric([measured[field] - predicted[field] for predicted, measured in pairs])
        for field in COMPARE_FIELDS
    }
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
