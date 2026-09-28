#!/usr/bin/env python3
"""Compare deterministic Rust/MATLAB D0/D1 contract evidence.

This is intentionally a strict textual key/value comparison. D0 identity and
D1 discrete events are never softened by numeric tolerances.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def load_result(path: Path) -> tuple[list[str], dict[str, str]]:
    lines = [line.strip() for line in path.read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    values: dict[str, str] = {}
    for line in lines:
        if "=" not in line:
            raise ValueError(f"{path}: malformed line {line!r}")
        key, value = line.split("=", 1)
        if not key or key in values:
            raise ValueError(f"{path}: empty or duplicate key {key!r}")
        values[key] = value
    for required in ("contract_gate_result_version", "engine_id", "d0", "d1_event_count", "d1"):
        if required not in values:
            raise ValueError(f"{path}: missing {required}")
    if values["contract_gate_result_version"] != "1" or values["d0"] != "PASS" or values["d1"] != "PASS":
        raise ValueError(f"{path}: result is not a V1 D0/D1 PASS")
    count = int(values["d1_event_count"])
    expected_events = {f"d1_event_{index}" for index in range(count)}
    actual_events = {key for key in values if key.startswith("d1_event_") and key != "d1_event_count" and key != "d1_event_tick_tolerance"}
    if actual_events != expected_events:
        raise ValueError(f"{path}: D1 event key set mismatch")
    return lines, values


def compare(rust_path: Path, matlab_path: Path) -> None:
    rust_lines, rust = load_result(rust_path)
    matlab_lines, matlab = load_result(matlab_path)
    if rust["engine_id"] != "fluxrt.rust.lifecycle.v1":
        raise ValueError(f"unexpected Rust engine identity: {rust['engine_id']!r}")
    if matlab["engine_id"] != "fluxrt.matlab.lifecycle.v1":
        raise ValueError(f"unexpected MATLAB engine identity: {matlab['engine_id']!r}")
    rust_shared = {key: value for key, value in rust.items() if key != "engine_id"}
    matlab_shared = {key: value for key, value in matlab.items() if key != "engine_id"}
    rust_shared_lines = [line for line in rust_lines if not line.startswith("engine_id=")]
    matlab_shared_lines = [line for line in matlab_lines if not line.startswith("engine_id=")]
    if rust_shared_lines != matlab_shared_lines or rust_shared != matlab_shared:
        all_keys = sorted(set(rust) | set(matlab))
        differences = [
            f"{key}: rust={rust.get(key)!r}, matlab={matlab.get(key)!r}"
            for key in all_keys
            if key != "engine_id" and rust.get(key) != matlab.get(key)
        ]
        if rust_shared_lines != matlab_shared_lines and not differences:
            differences.append("key order differs")
        raise ValueError("D0/D1 mismatch:\n" + "\n".join(differences))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rust", required=True, type=Path)
    parser.add_argument("--matlab", required=True, type=Path)
    args = parser.parse_args()
    compare(args.rust, args.matlab)
    print("FLUXRT_DUAL_CONTRACT_PASS D0=PASS D1=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
