#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BEMF 实测分析器（v5，平衡三线联合拟合）：裁定 Ke 的单位口径与磁链。

BEMF measurement analyzer (v5, balanced three-line fit): settle the Ke unit
convention and the flux linkage from the post-stop fixed window.

方法 / Method — 频率法（V/f 法）
--------------------------------
同步电机（PMSM）转子靠惯性自由旋转时，BEMF 电频率与转速满足严格物理关系
``f_e = p * n / 60``，相电压峰值满足 ``V_ph_pk = psi * 2*pi*f_e``。两者由
**同一个采集窗**测出后联立，磁链直接从波形本身解出：

    psi_SI = V_ph_pk / (2*pi*f_e)          [Wb, 每电弧度——固件 params.rs 口径]

**转速不参与计算**：指令转速（foc_start 的目标 rpm）只作为诊断量（开环跟踪
质量 + 滑行衰减检查），不再是 Ke/磁链的基准。这消除了两类固有偏差：
- 开环强拖中转子实际转速 != 指令转速（负载角/猎振，约 1~3%）；
- foc_stop 后滑行衰减导致指令值过期（约 1~2%）。

原始 ADC 测的是三个端子对板地电压，电机中性点浮动时包含未知公共模分量。v5 先构造
``U-V``、``V-W``、``W-U`` 三个线间波形，再按三条线电压固有的 120° 相差联合拟合
共同频率、共同幅值和各线独立 DC 偏置：

    V_ph_pk = median(V_ll_pk) / sqrt(3)

这样公共模电压被差分消除；三个线间结果还能提供幅值/频率一致性检查。

两种磁链口径（重要，勿混用）/ Two flux conventions (do not mix)
---------------------------------------------------------------
- **教科书 SI 口径（固件 params.rs 的口径）**：``psi = V_ph_pk / omega_e``，
  每电弧度的磁链。基线 ``0.005529 Wb`` 与数据库 Ke=4.964 Vll_rms/kRPM
  经标准电机理论精确自洽（GBM2804H, p=7）。
- **MCSDK/convert 工具口径**：除以**机械**角速度，等于 ``p * psi_SI``，
  含极对数折叠。两者相差 **p=7 倍**——这正是交接表"差约 7 倍"警告的来源；
  README 里的"3.83 倍"= 7 × 0.547（0.547 是 run1 未完成 Ke 与数据库之比），
  是两种口径直接相除的假警报。

Ke 的口径判定 / Ke convention decision
--------------------------------------
Profiler 读回的 ``SC_KE`` 存在线电压 RMS 与相电压 RMS 两种口径的歧义（差
sqrt(3)）。本分析器由 f_e 反推实测转速 ``n = f_e*60/p``，再计算：
    Ke(线RMS) = V_ll_rms / (n/1000)
    Ke(相RMS) = V_ph_rms / (n/1000)
哪个与 ``SC_KE`` 的比值更接近 1，SC_KE 就是哪个口径。

判定准则 / Decision rules
--------------------------
- ``flux_vs_baseline_factor``（SI 对 SI 同口径）：≈1 基线正确；显著偏离
  （>1.5 或 <0.67）才说明基线磁链有误；
- ``tracking_deviation_pct``（实测转速 vs 指令转速）：诊断量，|偏差|>10% 时
  validity 降级——此时频率法结果本身仍可信（它不依赖指令值），降级提示的是
  开环跟踪/滑行衰减异常，建议复测；
- 频率测不出（过零点不足、每相 <2 个）时 validity 直接降级，不输出裁定。

用法 / Usage
-----------
    python analyze_bemf_measurement.py <csv> --rpm 582 --sc-ke 3.75248
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics
import sys

SAMPLE_RATE_HZ = 12000
POLE_PAIRS = 7
NOMINAL_UV_PER_COUNT = 4469
#: 工程基线磁链（params.rs: 0.034739897 / 2*pi），**SI 口径**（每电弧度）。
BASELINE_FLUX_WB = 0.005529026
#: Profiler 四次运行的 SC_KE 均值（profiled-20260925 README）。
DEFAULT_SC_KE = 3.75248
SQRT2 = math.sqrt(2.0)
SQRT3 = math.sqrt(3.0)
RAD_PER_RPM = 2.0 * math.pi / 60.0
KRPM_TO_RAD_S = 1000.0 * RAD_PER_RPM


def ke_line_rms_to_flux_mcsdk(ke: float) -> float:
    """线电压 RMS 口径的 Ke -> MCSDK 口径磁链 [Wb]（= p × SI 磁链）。"""
    return (ke / SQRT3 * SQRT2) / KRPM_TO_RAD_S


def ke_phase_rms_to_flux_mcsdk(ke: float) -> float:
    """相电压 RMS 口径的 Ke -> MCSDK 口径磁链 [Wb]（= p × SI 磁链）。"""
    return (ke * SQRT2) / KRPM_TO_RAD_S


def load_samples(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"sequence", "phase_u_raw", "phase_v_raw", "phase_w_raw"}
        if not required.issubset(reader.fieldnames or []):
            raise RuntimeError(f"CSV {path} lacks the fixed-window columns")
        return [row for row in reader]


def remove_offset(values: list[int]) -> list[float]:
    mean = sum(values) / len(values)
    return [float(v) - mean for v in values]


def rising_zero_crossings(values: list[float], sample_rate: int) -> list[float]:
    """上升沿过零时刻（秒），线性插值精化。/ Rising-edge zero crossings in s."""
    crossings: list[float] = []
    for i in range(1, len(values)):
        if values[i - 1] <= 0.0 < values[i]:
            frac = (0.0 - values[i - 1]) / (values[i] - values[i - 1])
            crossings.append((i - 1 + frac) / sample_rate)
    return crossings


def estimate_frequency_hz_from_crossings(crossings: list[float]) -> float | None:
    """由过零时刻序列测频：首末间隔 / 周期数，对窗口截断稳健。"""
    if len(crossings) < 2:
        return None
    n_periods = len(crossings) - 1
    return n_periods / (crossings[-1] - crossings[0])


def solve_linear_system(matrix: list[list[float]], vector: list[float]) -> list[float]:
    """带主元的高斯消元；矩阵很小，避免给分析脚本引入 numpy 依赖。"""
    size = len(vector)
    if size == 0 or len(matrix) != size or any(len(row) != size for row in matrix):
        raise RuntimeError("invalid linear-system dimensions")
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for pivot in range(size):
        best = max(range(pivot, size), key=lambda row: abs(augmented[row][pivot]))
        if abs(augmented[best][pivot]) < 1e-12:
            raise RuntimeError("singular sine-fit matrix")
        augmented[pivot], augmented[best] = augmented[best], augmented[pivot]
        scale = augmented[pivot][pivot]
        augmented[pivot] = [value / scale for value in augmented[pivot]]
        for row in range(size):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            augmented[row] = [
                value - factor * base
                for value, base in zip(augmented[row], augmented[pivot])
            ]
    return [augmented[row][size] for row in range(size)]


def fit_sine_peak(values: list[float], frequency_hz: float,
                  sample_rate_hz: int) -> float:
    """拟合 ``a*cos(wt)+b*sin(wt)+c``，返回峰值 sqrt(a²+b²)。"""
    cosines = [
        math.cos(2.0 * math.pi * frequency_hz * index / sample_rate_hz)
        for index in range(len(values))
    ]
    sines = [
        math.sin(2.0 * math.pi * frequency_hz * index / sample_rate_hz)
        for index in range(len(values))
    ]
    matrix = [
        [sum(c * c for c in cosines), sum(c * s for c, s in zip(cosines, sines)),
         sum(cosines)],
        [sum(c * s for c, s in zip(cosines, sines)), sum(s * s for s in sines),
         sum(sines)],
        [sum(cosines), sum(sines), float(len(values))],
    ]
    vector = [
        sum(value * c for value, c in zip(values, cosines)),
        sum(value * s for value, s in zip(values, sines)),
        sum(values),
    ]
    cosine_gain, sine_gain, _offset = solve_linear_system(matrix, vector)
    return math.hypot(cosine_gain, sine_gain)


def fit_balanced_three_line(
    line_counts: dict[str, list[float]], frequency_hz: float,
    sample_rate_hz: int,
) -> tuple[float, float, dict[str, float]]:
    """联合拟合三条相差 120° 的线电压，返回峰值、归一化残差和各线偏置。

    ``uv/vw/wu`` 的理论相移依次为 ``+30/-90/+150 deg``。共用一组正余弦
    系数，只给每条线保留独立 DC 偏置；短于两个周期时也不会让三条线各自拟合出
    不一致的幅值。
    """
    names = ("uv", "vw", "wu")
    shifts = (math.pi / 6.0, -math.pi / 2.0, 5.0 * math.pi / 6.0)
    size = 5  # common sine/cosine + three line offsets
    normal = [[0.0] * size for _ in range(size)]
    rhs = [0.0] * size
    total_variance = 0.0
    total_square = 0.0

    for line_index, (name, shift) in enumerate(zip(names, shifts)):
        values = line_counts[name]
        mean = sum(values) / len(values)
        total_variance += sum((value - mean) ** 2 for value in values)
        for index, value in enumerate(values):
            angle = 2.0 * math.pi * frequency_hz * index / sample_rate_hz + shift
            row = [math.sin(angle), math.cos(angle), 0.0, 0.0, 0.0]
            row[2 + line_index] = 1.0
            total_square += value * value
            for column in range(size):
                rhs[column] += row[column] * value
                for other in range(size):
                    normal[column][other] += row[column] * row[other]

    coefficients = solve_linear_system(normal, rhs)
    # 最小二乘解处 SSE = y'y - beta'X'y；浮点舍入可产生极小负数。
    residual_square = max(
        0.0, total_square - sum(value * term for value, term in zip(coefficients, rhs)))
    normalized_residual = (
        residual_square / total_variance if total_variance > 1e-12 else math.inf)
    offsets = {name: coefficients[2 + index] for index, name in enumerate(names)}
    return math.hypot(coefficients[0], coefficients[1]), normalized_residual, offsets


def estimate_balanced_frequency(
    line_counts: dict[str, list[float]], sample_rate_hz: int,
) -> tuple[float | None, float | None, float | None, dict[str, float]]:
    """用粗扫 + 局部细扫寻找三线联合拟合的共同电频率。"""
    best: tuple[float, float, float, dict[str, float]] | None = None

    def consider(frequency_hz: float) -> None:
        nonlocal best
        peak, residual, offsets = fit_balanced_three_line(
            line_counts, frequency_hz, sample_rate_hz)
        candidate = (residual, frequency_hz, peak, offsets)
        if best is None or candidate[0] < best[0]:
            best = candidate

    # 本脚本的安全转速门为 500..1200 rpm、极对数为 7；10..200 Hz 同时给失步
    # 工况留出余量。先 0.5 Hz 粗扫，再在最佳点 +/-1 Hz 内以 0.02 Hz 细化。
    for step in range(20, 401):
        consider(step * 0.5)
    assert best is not None
    coarse_frequency = best[1]
    fine_start = max(10.0, coarse_frequency - 1.0)
    for step in range(101):
        consider(fine_start + step * 0.02)
    assert best is not None
    residual, frequency, peak, offsets = best
    if not math.isfinite(residual) or peak <= 0.0:
        return None, None, None, {}
    return frequency, peak, residual, offsets


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", type=pathlib.Path)
    parser.add_argument("--rpm", type=float, required=True,
                        help="commanded open-loop rpm (diagnostic reference only in v5)")
    parser.add_argument("--sc-ke", type=float, default=DEFAULT_SC_KE,
                        help="Profiler SC_KE to compare against (default: 4-run mean)")
    parser.add_argument("--baseline-flux", type=float, default=BASELINE_FLUX_WB,
                        help="firmware SI-convention baseline flux [Wb]")
    parser.add_argument("--pole-pairs", type=int, default=POLE_PAIRS)
    args = parser.parse_args()

    samples = load_samples(args.csv)
    if len(samples) != 256:
        print(f"warning: expected 256 samples, got {len(samples)}", file=sys.stderr)

    result: dict[str, object] = {
        "format_version": 5,
        "analysis_kind": "bemf-ke-convention",
        "method": "balanced three-line frequency method (V/f): common-mode-cancelled "
                  "U-V/V-W/W-U with shared amplitude/frequency and 120-degree phases",
        "csv": str(args.csv),
        "commanded_rpm": args.rpm,
        "sc_ke_compared": args.sc_ke,
        "pole_pairs": args.pole_pairs,
        "nominal_uv_per_count": NOMINAL_UV_PER_COUNT,
    }

    # --- 1. 读取三端电压；相对板地幅值只保留为诊断 ---
    phase_ground_diagnostic: dict[str, dict[str, float]] = {}
    centred_phases: dict[str, list[float]] = {}
    for phase in ("u", "v", "w"):
        raw = [int(s[f"phase_{phase}_raw"]) for s in samples]
        centred = remove_offset(raw)
        centred_phases[phase] = centred
        half_pp = (max(centred) - min(centred)) / 2.0
        phase_ground_diagnostic[phase] = {
            "half_peak_peak_counts": half_pp,
            "peak_v": half_pp * NOMINAL_UV_PER_COUNT / 1e6,
            "rms_v": half_pp * NOMINAL_UV_PER_COUNT / 1e6 / SQRT2,
        }
    result["phase_to_ground_diagnostic"] = phase_ground_diagnostic

    # --- 2. 线间差分与过零测频（公共模消除） ---
    line_counts = {
        "uv": [u - v for u, v in zip(centred_phases["u"], centred_phases["v"])],
        "vw": [v - w for v, w in zip(centred_phases["v"], centred_phases["w"])],
        "wu": [w - u for w, u in zip(centred_phases["w"], centred_phases["u"])],
    }
    freq_by_line: dict[str, float | None] = {}
    crossing_count_by_line: dict[str, int] = {}
    for line, values in line_counts.items():
        crossings = rising_zero_crossings(values, SAMPLE_RATE_HZ)
        crossing_count_by_line[line] = len(crossings)
        freq_by_line[line] = estimate_frequency_hz_from_crossings(crossings)
    valid_freqs = [f for f in freq_by_line.values() if f is not None]
    f_e, balanced_peak_counts, balanced_residual, balanced_offsets = (
        estimate_balanced_frequency(line_counts, SAMPLE_RATE_HZ))
    result["frequency_method"] = {
        "electrical_hz_by_line_zero_crossing": freq_by_line,
        "rising_crossings_by_line": crossing_count_by_line,
        "electrical_hz_median": f_e,
        "electrical_hz_balanced_fit": f_e,
        "balanced_fit_normalized_residual": balanced_residual,
        "balanced_fit_line_offsets_counts": balanced_offsets,
        "line_spread_hz": (
            max(valid_freqs) - min(valid_freqs)) if len(valid_freqs) > 1 else None,
    }

    # --- 3. 同窗线间正弦拟合幅值，换算相峰值 ---
    line_amplitude: dict[str, dict[str, float]] = {}
    v_ll_peak: float | None = None
    v_ph_peak: float | None = None
    if f_e is not None and balanced_peak_counts is not None:
        for line, values in line_counts.items():
            peak_counts = fit_sine_peak(values, f_e, SAMPLE_RATE_HZ)
            line_amplitude[line] = {
                "sine_fit_peak_counts": peak_counts,
                "peak_v": peak_counts * NOMINAL_UV_PER_COUNT / 1e6,
            }
        line_peaks = [entry["peak_v"] for entry in line_amplitude.values()]
        independent_line_peak_median = statistics.median(line_peaks)
        v_ll_peak = balanced_peak_counts * NOMINAL_UV_PER_COUNT / 1e6
        v_ph_peak = v_ll_peak / SQRT3
        amplitude_spread_pct = (
            100.0 * (max(line_peaks) - min(line_peaks)) / v_ll_peak
            if v_ll_peak > 0.0 else None
        )
    else:
        independent_line_peak_median = None
        amplitude_spread_pct = None
    result["line_to_line_amplitude"] = {
        "by_line": line_amplitude,
        "line_peak_v_median": v_ll_peak,
        "balanced_line_peak_v": v_ll_peak,
        "independent_line_peak_v_median": independent_line_peak_median,
        "phase_peak_v_from_line": v_ph_peak,
        "line_peak_spread_pct": amplitude_spread_pct,
        "conversion": "V_ph_pk = median(V_ll_pk) / sqrt(3)",
    }

    # --- 4. 频率法磁链 + 实测转速 + 诊断 ---
    freq_diagnostic: dict[str, float | None] = {}
    if f_e is not None and v_ph_peak is not None and v_ll_peak is not None:
        omega_e = 2.0 * math.pi * f_e
        flux_si = v_ph_peak / omega_e
        rpm_measured = f_e * 60.0 / args.pole_pairs
        freq_diagnostic = {
            "rpm_measured_from_frequency": rpm_measured,
            "tracking_deviation_pct": 100.0 * (rpm_measured - args.rpm) / args.rpm,
        }
        result["flux_frequency_method"] = {
            "flux_si_wb": flux_si,
            "convention": "V_ph_pk per electrical rad/s; firmware params.rs convention",
            "note": "psi = V_ph_pk / (2*pi*f_e); commanded rpm NOT used",
        }
        flux_selected: float | None = flux_si
        ke_krpm_base = rpm_measured / 1000.0
        v_ph_rms = v_ph_peak / SQRT2
        v_ll_rms = v_ll_peak / SQRT2
    else:
        result["flux_frequency_method"] = None
        flux_selected = None
        ke_krpm_base = args.rpm / 1000.0  # 降级路径：只能用指令值，validity 会降级
        # 没有可靠频率时不再用对地相电压猜测幅值；Ke 输出 NaN 并由 validity 拒绝。
        v_ph_rms = math.nan
        v_ll_rms = math.nan

    # --- 5. Ke（两种口径）与 MCSDK 口径参考值 ---
    ke_line = v_ll_rms / ke_krpm_base
    ke_phase = v_ph_rms / ke_krpm_base
    result["ke_candidates"] = {
        "ke_base_rpm": ("measured-from-frequency" if f_e is not None
                        else "commanded (degraded)"),
        "ke_line_rms_per_krpm": ke_line,
        "ke_phase_rms_per_krpm": ke_phase,
        "flux_mcsdk_if_line_convention": ke_line_rms_to_flux_mcsdk(ke_line),
        "flux_mcsdk_if_phase_convention": ke_phase_rms_to_flux_mcsdk(ke_phase),
        "flux_mcsdk_note": (
            "MCSDK/convert-tool convention divides by mechanical speed = "
            "pole_pairs x SI flux; NEVER compare it directly with the firmware "
            "baseline (that mixing produced the historical 7x/3.83x alarms)"),
    }

    # --- 6. 口径判定 ---
    ratio_line = ke_line / args.sc_ke
    ratio_phase = ke_phase / args.sc_ke
    convention_candidate = (
        "ke-phph-rms"
        if abs(ratio_line - 1.0) <= abs(ratio_phase - 1.0)
        else "ke-phase-rms"
    )
    distance_line = abs(ratio_line - 1.0)
    distance_phase = abs(ratio_phase - 1.0)
    best_distance = min(distance_line, distance_phase)
    separation = abs(distance_line - distance_phase)
    convention_is_clear = best_distance <= 0.15 and separation >= 0.10
    convention = convention_candidate if convention_is_clear else "ambiguous"
    decision: dict[str, object] = {
        "sc_ke_matches": convention,
        "sc_ke_best_candidate": convention_candidate,
        "sc_ke_convention_validity": "ok" if convention_is_clear else "ambiguous",
        "ratio_line_vs_sc": ratio_line,
        "ratio_phase_vs_sc": ratio_phase,
        "distance_line_from_unity": distance_line,
        "distance_phase_from_unity": distance_phase,
        "decision_rule": "best distance <= 0.15 and distance separation >= 0.10",
        "flux_selected_convention": "SI (per electrical rad)",
    }
    if flux_selected is not None:
        decision.update({
            "flux_selected_wb": flux_selected,
            "baseline_flux_wb": args.baseline_flux,
            "flux_vs_baseline_factor": flux_selected / args.baseline_flux,
        })
    decision["convert_command"] = (
        "python tools/motor_profiler_convert.py convert <export.json> "
        f"--baseline profiles/candidates/rev1-gbm2804h-unapproved.json "
        f"--flux-convention {convention} --revision 2 --output <candidate.json>"
        if convention_is_clear else None)
    result["decision"] = decision

    # --- 7. 有效性 ---
    # 频率测不出 → 降级（无法裁定）。转速偏差只影响诊断，不影响频率法结果本身，
    # 但偏差过大说明工况异常（滑行过久/开环失步），建议复测而不是采信。
    tracking = freq_diagnostic.get("tracking_deviation_pct")
    frequency_spread_pct = (
        100.0 * (max(valid_freqs) - min(valid_freqs)) / f_e
        if f_e is not None and len(valid_freqs) == 3 else None
    )
    if f_e is None or v_ph_peak is None:
        measurement_validity = "degraded-no-frequency"
    elif balanced_residual is None or balanced_residual > 0.15:
        measurement_validity = "degraded-balanced-fit-residual-retest-advised"
    elif frequency_spread_pct is not None and frequency_spread_pct > 5.0:
        measurement_validity = "degraded-line-frequency-imbalance-retest-advised"
    elif amplitude_spread_pct is not None and amplitude_spread_pct > 35.0:
        measurement_validity = "degraded-line-amplitude-imbalance-retest-advised"
    elif tracking is not None and abs(tracking) > 10.0:
        measurement_validity = "degraded-tracking-anomaly-retest-advised"
    else:
        measurement_validity = "ok"
    validity = (
        "degraded-sc-ke-convention-ambiguous"
        if measurement_validity == "ok" and not convention_is_clear
        else measurement_validity)
    freq_diagnostic["line_frequency_spread_pct"] = frequency_spread_pct
    freq_diagnostic["line_amplitude_spread_pct"] = amplitude_spread_pct
    result["diagnostics"] = freq_diagnostic
    result["measurement_validity"] = measurement_validity
    result["validity"] = validity

    out_path = args.csv.with_suffix(".analysis.json")
    out_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"\nAnalysis written to {out_path}")
    if validity != "ok":
        print(f"\nWARNING: validity={validity}; treat the decision as indicative only.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
