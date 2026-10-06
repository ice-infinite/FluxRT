#!/usr/bin/env python3
"""Derive a non-approving H3 diagnostic from an incomplete powered capture.

The source ``raw-lines.csv`` and its failed summary are immutable evidence.  This
tool intersects fully identity-matched G431/DengFOC anchors, computes a clock
map for diagnosis, and writes a separate artifact whose status can never be
mistaken for an accepted H3 capture.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MAX_PARTIAL_DIAGNOSTIC_TRUTH_GAP_US = 20_000
MAX_PARTIAL_DIAGNOSTIC_MALFORMED_TRUTH_LINES = 1


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


board = _load("foc_h3_board_capture_partial", ROOT / "tools/foc_h3_board_capture.py")
dynamic = _load("foc_h3_dynamic_capture_partial", ROOT / "tools/foc_h3_dynamic_capture.py")


class PartialDiagnosticError(ValueError):
    """Raised when the failed source cannot support a bounded diagnosis."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def analyse_source(
    raw_path: Path,
    maximum_valid_truth_gap_us: int = dynamic.MAX_VALID_TRUTH_GAP_US,
    maximum_malformed_truth_lines: int = 0,
):
    parent_summary_path = raw_path.parent / "capture.summary.json"
    if not raw_path.is_file() or not parent_summary_path.is_file():
        raise PartialDiagnosticError("raw capture or parent summary is missing")
    parent_summary = json.loads(parent_summary_path.read_text(encoding="utf-8"))
    if parent_summary.get("status") != "capture-failed":
        raise PartialDiagnosticError("source capture is not explicitly failed")

    with raw_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    left = []
    right = []
    truth = []
    queries = []
    reported_count = None
    malformed_truth_line_count = 0
    terminal_partial_truth_line_count = 0
    for row_index, row in enumerate(rows):
        source = row.get("source", "")
        line = row.get("raw_line", "")
        if source == "g431":
            anchor = dynamic.parse_dynamic_anchor(line)
            if anchor is not None:
                left.append(anchor)
            query = dynamic.parse_ftr_query(line)
            if query is not None:
                queries.append(query)
            if line.startswith("FH3R,"):
                fields = line.split(",")
                if len(fields) != 4:
                    raise PartialDiagnosticError("H3 result schema mismatch")
                reported_count = int(fields[1], 10)
        elif source == "dengfoc":
            anchor = board.parse_deng_anchor(line)
            if anchor is not None:
                right.append(anchor)
            sample = board.parse_truth(line)
            if sample is not None:
                truth.append(sample)
            elif line.startswith("SYNC_TRUTH,"):
                if row_index == (len(rows) - 1):
                    terminal_partial_truth_line_count += 1
                else:
                    malformed_truth_line_count += 1

    if not (
        dynamic.MAX_VALID_TRUTH_GAP_US
        <= maximum_valid_truth_gap_us
        <= MAX_PARTIAL_DIAGNOSTIC_TRUTH_GAP_US
    ):
        raise PartialDiagnosticError(
            "partial diagnostic truth gap limit is outside the bounded range"
        )
    if not (
        0
        <= maximum_malformed_truth_lines
        <= MAX_PARTIAL_DIAGNOSTIC_MALFORMED_TRUTH_LINES
    ):
        raise PartialDiagnosticError(
            "partial diagnostic malformed truth limit is outside the bounded range"
        )
    if malformed_truth_line_count > maximum_malformed_truth_lines:
        raise PartialDiagnosticError(
            "malformed AS5600 truth line count "
            f"{malformed_truth_line_count} exceeds "
            f"{maximum_malformed_truth_lines}"
        )

    left_by_sequence = {row.edge_sequence: row for row in left}
    right_by_sequence = {row.edge_sequence: row for row in right}
    if len(left_by_sequence) != len(left) or len(right_by_sequence) != len(right):
        raise PartialDiagnosticError("duplicate anchor sequence")
    common = sorted(set(left_by_sequence) & set(right_by_sequence))
    if len(common) < 3:
        raise PartialDiagnosticError("fewer than three paired anchors")
    if any(b != a + 1 for a, b in zip(common, common[1:])):
        raise PartialDiagnosticError("paired anchor intersection is not contiguous")
    pairs = []
    for sequence in common:
        left_row = left_by_sequence[sequence]
        right_row = right_by_sequence[sequence]
        if (left_row.session_id, left_row.edge_tag) != (
            right_row.session_id,
            right_row.edge_tag,
        ):
            raise PartialDiagnosticError("paired anchor identity mismatch")
        pairs.append((left_row, right_row))

    truth, truth_quality = dynamic.prepare_truth_stream(
        truth,
        maximum_allowed_gap_us=maximum_valid_truth_gap_us,
    )
    truth_quality["malformed_truth_line_count"] = malformed_truth_line_count
    truth_quality["terminal_partial_truth_line_count"] = (
        terminal_partial_truth_line_count
    )
    truth_quality["maximum_malformed_truth_lines"] = (
        maximum_malformed_truth_lines
    )
    clock_summary = board.build_summary(pairs, truth)
    lower = min(left_row.control_tick for left_row, _ in pairs)
    upper = max(left_row.control_tick for left_row, _ in pairs)
    queries = [
        row
        for row in queries
        if lower <= int(row["reference_control_tick"]) <= upper
    ]
    for sequence, row in enumerate(queries):
        row["sequence"] = sequence
    return {
        "pairs": pairs,
        "truth": truth,
        "queries": queries,
        "reported_count": reported_count,
        "g431_count": len(left),
        "dengfoc_count": len(right),
        "g431_only_sequences": sorted(set(left_by_sequence) - set(right_by_sequence)),
        "dengfoc_only_sequences": sorted(set(right_by_sequence) - set(left_by_sequence)),
        "clock_mapping": clock_summary["clock_mapping"],
        "loopback": clock_summary["loopback"],
        "truth_quality": truth_quality,
    }


def write_output(raw_path: Path, output: Path, result) -> dict[str, object]:
    if output.exists():
        raise PartialDiagnosticError("output directory already exists")
    output.mkdir(parents=True)
    anchors_path = output / "paired-anchors.csv"
    with anchors_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["edge_sequence", "reference_control_tick", "truth_tick"])
        for left, right in result["pairs"]:
            writer.writerow([left.edge_sequence, left.control_tick, right.truth_tick_us])
    queries_path = output / "bracketed-queries.csv"
    with queries_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            "sequence",
            "reference_control_tick",
            "controller_state",
            "control_electrical_angle_rad",
            "forced_electrical_angle_rad",
            "estimated_electrical_angle_rad",
            "reliable",
        ])
        writer.writeheader()
        writer.writerows(result["queries"])
    truth_path = output / "valid-truth.csv"
    with truth_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "sequence",
                "truth_tick",
                "mechanical_angle_rad",
                "valid",
                "read_failures",
            ],
        )
        writer.writeheader()
        writer.writerows(result["truth"])
    summary = {
        "contract": "fluxrt-h3-partial-diagnostic",
        "version": 1,
        "status": "partial-diagnostic-not-approved",
        "source_capture_status": "capture-failed",
        "source_raw_sha256": _sha256(raw_path),
        "motor_power_enabled": True,
        "reported_anchor_count": result["reported_count"],
        "g431_anchor_count": result["g431_count"],
        "dengfoc_anchor_count": result["dengfoc_count"],
        "paired_anchor_count": len(result["pairs"]),
        "g431_only_sequences": result["g431_only_sequences"],
        "dengfoc_only_sequences": result["dengfoc_only_sequences"],
        "query_count": len(result["queries"]),
        "query_alignment_status": (
            "bracketed" if result["queries"] else
            "unavailable-source-anchor-domain"
        ),
        "truth_quality": result["truth_quality"],
        "loopback": result["loopback"],
        "clock_mapping": result["clock_mapping"],
        "parameter_approval": "not-granted",
        "target_capability_approval": "not-granted",
        "artifacts": {
            anchors_path.name: _sha256(anchors_path),
            queries_path.name: _sha256(queries_path),
            truth_path.name: _sha256(truth_path),
        },
    }
    (output / "diagnostic.summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--maximum-valid-truth-gap-us",
        type=int,
        default=dynamic.MAX_VALID_TRUTH_GAP_US,
    )
    parser.add_argument(
        "--maximum-malformed-truth-lines",
        type=int,
        default=0,
    )
    args = parser.parse_args()
    try:
        summary = write_output(
            args.raw,
            args.output,
            analyse_source(
                args.raw,
                maximum_valid_truth_gap_us=args.maximum_valid_truth_gap_us,
                maximum_malformed_truth_lines=args.maximum_malformed_truth_lines,
            ),
        )
    except (PartialDiagnosticError, board.CaptureError, dynamic.DynamicCaptureError,
            OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
