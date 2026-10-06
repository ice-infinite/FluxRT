#!/usr/bin/env python3
"""Compare independent Rust and MATLAB active-HFI plant results."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

FLOAT_COLUMNS = {
    "true_theta_mod_pi_rad",
    "estimated_theta_mod_pi_rad",
    "angle_error_rad",
    "response_magnitude_a",
    "relative_saliency",
    "peak_current_a",
}


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def read_trace(path: Path) -> dict[str, dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    return {row["case_id"]: row for row in rows}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust-metadata", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--matlab-metadata", type=Path, required=True)
    parser.add_argument("--matlab-trace", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=5.0e-4)
    args = parser.parse_args()

    rust_meta = read_metadata(args.rust_metadata)
    matlab_meta = read_metadata(args.matlab_metadata)
    for key in ("contract", "version", "workspace_revision", "case_count", "result"):
        if rust_meta.get(key) != matlab_meta.get(key):
            raise SystemExit(
                f"metadata mismatch {key}: {rust_meta.get(key)!r} != {matlab_meta.get(key)!r}"
            )

    rust = read_trace(args.rust_trace)
    matlab = read_trace(args.matlab_trace)
    if rust.keys() != matlab.keys():
        raise SystemExit(f"case set mismatch: {sorted(rust)} != {sorted(matlab)}")
    for case_id, rust_row in rust.items():
        matlab_row = matlab[case_id]
        if int(float(rust_row["valid"])) != int(float(matlab_row["valid"])):
            raise SystemExit(
                f"{case_id}.valid mismatch: {rust_row['valid']} != {matlab_row['valid']}"
            )
        for column in FLOAT_COLUMNS:
            left = float(rust_row[column])
            right = float(matlab_row[column])
            if not (math.isfinite(left) and math.isfinite(right)):
                raise SystemExit(f"{case_id}.{column} is not finite")
            if abs(left - right) > args.tolerance:
                raise SystemExit(f"{case_id}.{column} mismatch: {left} != {right}")
    print(f"[PASS] Rust/MATLAB active-HFI salient-plant matrix: {len(rust)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
