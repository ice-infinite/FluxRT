#!/usr/bin/env python3
"""Capture a bounded FluxRT run and always stop the power stage."""

from __future__ import annotations

import argparse
import csv
import pathlib
import sys
import time

import serial


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
]

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
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--target-rpm", type=float, default=582.0)
    parser.add_argument("--rate-hz", type=int, default=50)
    parser.add_argument("--pwm-hz", type=int, default=12000)
    parser.add_argument("--smo-slide-v", type=float)
    parser.add_argument("--smo-boundary-a", type=float)
    parser.add_argument("--emf-filter", type=float)
    parser.add_argument("--pll-kp", type=float)
    parser.add_argument("--pll-ki", type=float)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    return parser.parse_args()


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def read_available(port: serial.Serial, raw_lines: list[str]) -> list[str]:
    lines: list[str] = []
    while port.in_waiting:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if line:
            raw_lines.append(line)
            lines.append(line)
    return lines


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


def normalized_row(sample: dict[str, int], pwm_hz: int, target: float) -> dict[str, object]:
    return {
        # Realtime step count is reset immediately before the PWM outputs are
        # armed, so it is the common simulation/hardware time origin.
        "time_s": sample["step"] / pwm_hz,
        "step": sample["step"],
        "state": sample["state"],
        "phase_current_a": sample["ia_ma"] / 1000.0,
        "phase_current_b": sample["ib_ma"] / 1000.0,
        "phase_current_c": sample["ic_ma"] / 1000.0,
        "target_speed_rpm": target,
        "observer_speed_rpm": sample["observer_speed_rpm"],
        "true_speed_rpm": "",
        "id_ref_a": sample["id_ref_ma"] / 1000.0,
        "iq_ref_a": sample["iq_ref_ma"] / 1000.0,
        "id_a": sample["id_ma"] / 1000.0,
        "iq_a": sample["iq_ma"] / 1000.0,
        "vd_v": sample["vd_mv"] / 1000.0,
        "vq_v": sample["vq_mv"] / 1000.0,
        "duty_a": sample["duty_a_pm"] / 1000.0,
        "duty_b": sample["duty_b_pm"] / 1000.0,
        "duty_c": sample["duty_c_pm"] / 1000.0,
        "dc_bus_voltage_v": sample["vbus_mv"] / 1000.0,
        "control_angle_rad": sample["control_angle_mrad"] / 1000.0,
        "forced_angle_rad": sample["forced_angle_mrad"] / 1000.0,
        "observer_angle_rad": sample["observer_angle_mrad"] / 1000.0,
        "true_angle_rad": "",
        "observer_reliable": sample["reliable"],
        "flags": sample["flags"],
    }


def main() -> int:
    args = parse_args()
    if args.duration <= 0 or not 10 <= args.rate_hz <= 100:
        raise SystemExit("duration must be positive and rate must be 10..100 Hz")

    samples: list[dict[str, int]] = []
    raw_lines: list[str] = []
    args.output.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0) as port:
        time.sleep(0.25)
        port.reset_input_buffer()
        send(port, "foc_stop")
        time.sleep(0.15)
        tuning_commands = (
            ("slide", args.smo_slide_v, 1000.0),
            ("boundary", args.smo_boundary_a, 1000.0),
            ("filter", args.emf_filter, 1000.0),
            ("pll_kp", args.pll_kp, 1.0),
            ("pll_ki", args.pll_ki, 1.0),
        )
        for name, value, scale in tuning_commands:
            if value is not None:
                send(port, f"foc_cfg {name} {value * scale:g}")
                time.sleep(0.08)
                read_available(port, raw_lines)
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
                time.sleep(0.005)
        finally:
            # Stop is deliberately sent before disabling trace. This remains
            # the safety path even when parsing or file output fails.
            send(port, "foc_stop")
            time.sleep(0.15)
            for line in read_available(port, raw_lines):
                sample = parse_trace(line)
                if sample is not None:
                    samples.append(sample)
            send(port, "foc_trace stop")
            send(port, "foc_status")
            time.sleep(0.25)
            read_available(port, raw_lines)

    if not samples:
        args.output.with_suffix(".log").write_text("\n".join(raw_lines), encoding="utf-8")
        raise SystemExit("no FTR samples received; raw log was saved")

    first_step = samples[0]["step"]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_NAMES)
        writer.writeheader()
        for sample in samples:
            writer.writerow(normalized_row(sample, args.pwm_hz, args.target_rpm))
    log_path = args.output.with_suffix(".log")
    log_path.write_text("\n".join(raw_lines) + "\n", encoding="utf-8")

    print(f"FOC_HW_TRACE={args.output}")
    print(f"FOC_HW_RAW_LOG={log_path}")
    print(
        "FOC_HW_CAPTURE_PASS "
        f"samples={len(samples)} elapsed={(samples[-1]['step'] - first_step) / args.pwm_hz:.3f}s "
        f"reliable={sum(sample['reliable'] != 0 for sample in samples)}"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except serial.SerialException as error:
        raise SystemExit(f"serial error: {error}") from error
