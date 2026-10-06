#!/usr/bin/env python3
"""Compare independent Rust and MATLAB H2 trace-replay error tables."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


EXACT_COLUMNS = {
    "angle_index_valid",
    "angle_sample_count",
    "angle_span_counts",
    "transition_count",
}
FLOAT_COLUMNS = {
    "mechanical_angle_rad",
    "electrical_angle_rad",
    "candidate_inductance_h",
    "rmse_a",
    "maximum_absolute_residual_a",
}


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def read_rows(path: Path) -> dict[tuple[str, str], dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    result: dict[tuple[str, str], dict[str, str]] = {}
    for row in rows:
        key = (row["case_id"], f"{float(row['candidate_inductance_h']):.9g}")
        if key in result:
            raise SystemExit(f"duplicate replay result row: {key}")
        result[key] = row
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust-metadata", type=Path, required=True)
    parser.add_argument("--rust-trace", type=Path, required=True)
    parser.add_argument("--matlab-metadata", type=Path, required=True)
    parser.add_argument("--matlab-trace", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=2.0e-9)
    args = parser.parse_args()

    rust_meta = read_metadata(args.rust_metadata)
    matlab_meta = read_metadata(args.matlab_metadata)
    for key in (
        "contract",
        "version",
        "workspace_revision",
        "case_count",
        "row_count",
        "angle_evidence",
        "result",
    ):
        if rust_meta.get(key) != matlab_meta.get(key):
            raise SystemExit(
                f"metadata mismatch {key}: {rust_meta.get(key)!r} != {matlab_meta.get(key)!r}"
            )
    if rust_meta.get("angle_evidence") != "BLOCKED_UNTIL_VALID_INDEX":
        raise SystemExit("fixture must not claim a valid AS5600 angle index")

    rust = read_rows(args.rust_trace)
    matlab = read_rows(args.matlab_trace)
    if rust.keys() != matlab.keys():
        raise SystemExit(f"row set mismatch: {sorted(rust)} != {sorted(matlab)}")
    for key, rust_row in rust.items():
        matlab_row = matlab[key]
        if rust_row["angle_source"] != matlab_row["angle_source"]:
            raise SystemExit(
                f"{key}.angle_source mismatch: {rust_row['angle_source']} != "
                f"{matlab_row['angle_source']}"
            )
        for column in EXACT_COLUMNS:
            if int(float(rust_row[column])) != int(float(matlab_row[column])):
                raise SystemExit(
                    f"{key}.{column} mismatch: {rust_row[column]} != {matlab_row[column]}"
                )
        for column in FLOAT_COLUMNS:
            left = float(rust_row[column])
            right = float(matlab_row[column])
            if not (math.isfinite(left) and math.isfinite(right)):
                raise SystemExit(f"{key}.{column} is not finite")
            if abs(left - right) > args.tolerance:
                raise SystemExit(f"{key}.{column} mismatch: {left} != {right}")
    print(f"[PASS] Rust/MATLAB H2 replay error table: {len(rust)} rows")
    print("[BLOCKED] H2-5 angle evidence remains unavailable in the historical fixture")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
