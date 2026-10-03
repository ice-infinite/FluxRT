#!/usr/bin/env python3
"""Compare independent Rust and MATLAB advanced-FOC policy traces."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

FLOAT_COLUMNS = {
    "id_ref_a", "iq_ref_a", "vd_ff_v", "vq_ff_v", "injection_alpha_v",
    "injection_beta_v", "voltage_limit_v", "flying_angle_rad",
    "flying_speed_rad_s",
}
INTEGER_COLUMNS = {
    "region", "modulation", "active_features", "current_limited",
    "hfi_valid", "flying_state",
}


def metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def trace(path: Path) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    return {row["case_id"]: row for row in rows}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-metadata", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--matlab-metadata", type=Path, required=True)
    parser.add_argument("--matlab-trace", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=2.0e-5)
    args = parser.parse_args()

    rust_meta = metadata(args.rust_metadata)
    matlab_meta = metadata(args.matlab_metadata)
    for key in ("contract", "version", "workspace_revision", "case_count", "result"):
        if rust_meta.get(key) != matlab_meta.get(key):
            raise SystemExit(f"metadata mismatch {key}: {rust_meta.get(key)!r} != {matlab_meta.get(key)!r}")

    rust = trace(args.rust_trace)
    matlab = trace(args.matlab_trace)
    if rust.keys() != matlab.keys():
        raise SystemExit(f"case set mismatch: {sorted(rust)} != {sorted(matlab)}")
    for case_id, rust_row in rust.items():
        matlab_row = matlab[case_id]
        for column in INTEGER_COLUMNS:
            if int(float(rust_row[column])) != int(float(matlab_row[column])):
                raise SystemExit(f"{case_id}.{column} mismatch: {rust_row[column]} != {matlab_row[column]}")
        for column in FLOAT_COLUMNS:
            left, right = float(rust_row[column]), float(matlab_row[column])
            if not (math.isfinite(left) and math.isfinite(right)) or abs(left-right) > args.tolerance:
                raise SystemExit(f"{case_id}.{column} mismatch: {left} != {right}")
    print(f"[PASS] Rust/MATLAB advanced FOC policy matrix: {len(rust)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
