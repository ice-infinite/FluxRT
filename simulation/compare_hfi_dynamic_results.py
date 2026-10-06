#!/usr/bin/env python3
"""Compare independent Rust and MATLAB dynamic active-HFI metrics."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

FLOAT_COLUMNS = {
    "rms_angle_error_rad",
    "maximum_angle_error_rad",
    "mean_response_a",
    "peak_current_a",
}


def metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def metrics(path: Path) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    return {row["case_id"]: row for row in rows}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-metadata", type=Path, required=True)
    parser.add_argument("--rust-metrics", type=Path, required=True)
    parser.add_argument("--matlab-metadata", type=Path, required=True)
    parser.add_argument("--matlab-metrics", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=2.0e-3)
    args = parser.parse_args()

    rust_meta = metadata(args.rust_metadata)
    matlab_meta = metadata(args.matlab_metadata)
    for key in ("contract", "version", "workspace_revision", "case_count", "result"):
        if rust_meta.get(key) != matlab_meta.get(key):
            raise SystemExit(
                f"metadata mismatch {key}: {rust_meta.get(key)!r} != {matlab_meta.get(key)!r}"
            )
    rust = metrics(args.rust_metrics)
    matlab = metrics(args.matlab_metrics)
    if rust.keys() != matlab.keys():
        raise SystemExit(f"case set mismatch: {sorted(rust)} != {sorted(matlab)}")
    for case_id, rust_row in rust.items():
        matlab_row = matlab[case_id]
        if int(float(rust_row["valid"])) != int(float(matlab_row["valid"])):
            raise SystemExit(f"{case_id}.valid mismatch")
        for column in FLOAT_COLUMNS:
            left = float(rust_row[column])
            right = float(matlab_row[column])
            if not (math.isfinite(left) and math.isfinite(right)):
                raise SystemExit(f"{case_id}.{column} is not finite")
            if abs(left-right) > args.tolerance:
                raise SystemExit(f"{case_id}.{column} mismatch: {left} != {right}")
    print(f"[PASS] Rust/MATLAB dynamic active-HFI matrix: {len(rust)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
