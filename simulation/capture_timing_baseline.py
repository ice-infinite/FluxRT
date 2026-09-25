#!/usr/bin/env python3
"""Capture bounded, correlated ISR timing for trace-off/10 Hz/50 Hz runs.

FluxRT 实时性（WCET）基线采集脚本：对 trace 关闭 / 10 Hz / 50 Hz 三种配置各跑一次
有界电机场景，收集固件的 `FTIMING,` 记录并给出逐项自检结论。
FluxRT real-time (WCET) baseline capture: runs one bounded motor scenario per trace
configuration (off / 10 Hz / 50 Hz), collects the firmware `FTIMING,` records and
reports a per-scenario self-check.

职责 / Responsibility:
  - 每组都从已停机状态开始，临时打开 trace、启动电机、跑固定时长、再停机；
  - 读取 `foc_status` 打印的 `FTIMING,` 行，校验 timing 分段来自同一个 ISR 样本、
    trace 模式与请求一致、没有 deadline miss、控制环没有故障。
  Each scenario starts stopped, briefly enables trace, runs the motor for a bounded
  duration, and verifies the `FTIMING,` record: segments from one ISR sample, trace
  mode matching the request, no deadline misses, no control fault.

安全 / Safety:
  - 没有 `--allow-motor-run` 时脚本直接拒绝运行，绝不"顺手"上电；
  - 每组开始先 `foc_stop`，结束路径（含异常）在 `finally` 中再次 `foc_stop`，
    并恢复 trace 与 closed-loop 开关，保证功率级始终关闭、配置不被残留改动。
  - Refuses to energize the motor without `--allow-motor-run`; sends `foc_stop` at the
    start of every scenario and again in `finally`, then restores trace and the
    closed-loop switch, so the power stage is left disabled and the config unmodified.

输出 / Outputs:
  - `--output`：JSON（schema_version / generated_at / git_commit / scenarios[]）；
    典型路径 `simulation/results/timing_*.json`，已被 git 忽略；
  - 同名 `.log`：带 `[trace-50Hz]` 之类前缀的原始串口行，同样被 git 忽略。
  The JSON and its sibling `.log` land under the git-ignored `/simulation/results/`.

参考 / Reference: docs/构建档与优化等级.md §4, docs/performance/2026-09-23-A0关联WCET基线.md
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import subprocess
import sys
import time

import serial


# 命令行契约 / CLI contract:
#   --port / --baud     串口与波特率，默认 COM6 / 115200
#   --duration          每组场景的电机运行时长 [s]，默认 5.0，必须 > 0
#   --target-rpm        速度目标 [rpm]，默认 582.0，透传给 `foc_start`
#   --rates             trace 抽稀率列表 [Hz]，从 0,10,50 中选，默认 "0,10,50"
#                       （0 = 关闭 trace，用作时序裕量的对照基线）
#   --profile           diagnostic | production；production 已裁掉 trace，
#                       因此只允许 --rates 0，否则脚本拒绝运行
#   --closed-loop       临时 foc_cfg closedloop 1；退出时在 finally 恢复为 0
#   --allow-motor-run   安全闸门：不给出时脚本拒绝驱动电机
#   --output            JSON 结果路径，必填
# 只有 --output 必填，但 --allow-motor-run 是实际的"上电许可"开关。
# Only --output is mandatory; --allow-motor-run is the real energize-permission gate.
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM6")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--target-rpm", type=float, default=582.0)
    parser.add_argument("--rates", default="0,10,50")
    parser.add_argument("--profile", choices=("diagnostic", "production"), default="diagnostic")
    parser.add_argument("--closed-loop", action="store_true")
    parser.add_argument("--allow-motor-run", action="store_true")
    parser.add_argument("--output", type=pathlib.Path, required=True)
    return parser.parse_args()


# 与 capture_hardware_trace.py 相同：命令必须以 CRLF 结尾并立即 flush。
# Same contract as capture_hardware_trace.py: CRLF-terminated, flushed immediately.
def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


# 与 FTR 采集不同，这里给每行加上 `[场景标签]` 前缀写入 raw：JSON 里只保留结论，
# 出问题时必须能从日志还原"哪一组、什么时候、固件回了什么"。
# Unlike the FTR capture, every raw line is prefixed with the scenario label: the JSON
# keeps only verdicts, so the log must be able to replay which run said what.
def read_available(port: serial.Serial, raw: list[str], prefix: str) -> list[str]:
    lines: list[str] = []
    while port.in_waiting:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if line:
            raw.append(f"[{prefix}] {line}")
            lines.append(line)
    return lines


# 在规定时长内持续轮询收集串口输出；最后再排空一次，避免丢掉刚好落在边界上的 FTIMING 行。
# Polls the port for `seconds`, then drains once more so an FTIMING line that lands right
# on the deadline is not lost.
def collect_for(port: serial.Serial, seconds: float, raw: list[str], prefix: str) -> list[str]:
    deadline = time.monotonic() + seconds
    lines: list[str] = []
    while time.monotonic() < deadline:
        lines.extend(read_available(port, raw, prefix))
        time.sleep(0.005)
    lines.extend(read_available(port, raw, prefix))
    return lines


# `FTIMING,` 行的字段顺序，必须与 `applications/main.c` 的 `foc_print_timing()` 一致。
# Field order of one `FTIMING,` line; must match `foc_print_timing()` in main.c.
#
# 单位 / Units:
#   version/samples/invalid        协议版本、样本计数、无效样本计数 [counts]
#   step                           该 WCET 样本所处的实时控制步 [12 kHz cycles]
#   total/pre/control/post         同一次 ISR 内的分段耗时与合计 [cycles]
#   trace/sampled                  trace 使能标志 / 本拍是否真的写入样本 [0/1]；
#                                  sampled 受抽稀分频控制，所以校验"模式"只看 trace
#   peak_pre/peak_control/peak_post  各段独立峰值 [cycles] —— 不可相加
#   deadline                       软件截止周期 [cycles]，对照 12500 @12 kHz
#   misses/errors                  deadline miss 与控制错误计数 [counts]
#   fault                          控制故障位域 (control_fault_flags)
#   flags                          诊断位域 (diagnostics.flags)
#   inverter/inverter_observer/inverter_feedforward
#                                  逆变器模型总门/观测器修正/PWM 前馈 [0/1]
#   peak_* 是互相独立的峰值，不能相加；total 才是单个 ISR 样本的合计。
#   The peak_* values are independent maxima and must not be summed; only `total`
#   belongs to one ISR sample.
TIMING_FIELDS = (
    "version",
    "samples",
    "invalid",
    "step",
    "total",
    "pre",
    "control",
    "post",
    "trace",
    "sampled",
    "peak_pre",
    "peak_control",
    "peak_post",
    "deadline",
    "misses",
    "errors",
    "fault",
    "flags",
    "inverter",
    "inverter_observer",
    "inverter_feedforward",
)
EXPECTED_TIMING_VERSION = 2


# 解析一行 FTIMING；字段数不符或含非整数则返回 None（其他 shell 行同样返回 None）。
# 与 trace 解析一致：静默跳过，让无关输出不影响整轮采集。
# Parses one FTIMING line; wrong field counts or non-integer fields return None, as do
# unrelated shell lines. Skipping silently keeps other output from aborting the run.
def parse_timing(line: str) -> dict[str, int] | None:
    marker = line.find("FTIMING,")
    if marker < 0:
        return None
    try:
        raw_values = line[marker + len("FTIMING,") :].split(",")
        values = [int(value, 0) for value in raw_values]
    except ValueError:
        return None
    return dict(zip(TIMING_FIELDS, values)) if len(values) == len(TIMING_FIELDS) else None


# 记录采集时的固件提交，便于把 WCET 数字回溯到确切的二进制来源。
# Records the firmware commit so a WCET number can be traced back to a binary.
#
# 不是 git 仓库、没有 git、或命令失败时返回 "unknown"，而不是抛异常：
# 时序数据本身仍然有价值，缺少溯源信息不应让整份结果作废。
# Returns "unknown" when git is missing or the command fails: a missing provenance
# stamp must not invalidate otherwise usable timing data.
def git_commit(project: pathlib.Path) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=project,
        check=False,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


# 跑一组场景并给出结论。单组流程：停机 -> （可选）开 trace -> 起电机 ->
# 等 ARMED 确认 -> 运行 duration 秒 -> 停机 -> 读 foc_status 里的 FTIMING -> （可选）关 trace。
# Runs one scenario and returns its verdict. Per-scenario flow: stop, optionally start
# trace, start the motor, wait for the ARMED acknowledgement, run for `duration`,
# stop, read the FTIMING from foc_status, and optionally stop trace.
#
# 参数 / Parameters:
#   rate_hz         trace 抽稀率 [Hz]，0 表示本次不开 trace
#   duration        电机运行时长 [s]
#   target_rpm      速度目标 [rpm]
#   trace_supported 该构建档是否编译进 trace；Production 档为 False，
#                   此时不会发 foc_trace（固件里根本没有该命令）
#   raw             原始日志累加器，用于失败复盘
#
# 失败语义 / Failure: 固件拒绝启动或没有 ARMED 确认时直接抛 RuntimeError，
# 由 main() 的 finally 负责停机；本函数自身不做收尾。
def run_scenario(
    port: serial.Serial,
    rate_hz: int,
    duration: float,
    target_rpm: float,
    trace_supported: bool,
    raw: list[str],
) -> dict[str, object]:
    label = f"trace-{rate_hz}Hz" if rate_hz else "trace-off"
    raw.append(f"[{label}] BEGIN")
    # 每组都从确定状态开始：先停机、再关 trace（若该档支持），
    # 保证上一组残留的使能或 trace 状态不会污染本组时序。
    # Every scenario starts from a known state: stop, then trace off, so leftovers from
    # the previous run cannot pollute this run's timing.
    send(port, "foc_stop")
    if trace_supported:
        send(port, "foc_trace stop")
    collect_for(port, 0.2, raw, label)
    if rate_hz:
        send(port, f"foc_trace start {rate_hz}")
        collect_for(port, 0.2, raw, label)

    send(port, f"foc_start {target_rpm:g}")
    # 只从这 0.5 s 的应答里判断启动结果：REFUSED 说明安全门/总线电压不满足，
    # 没有 ARMED 说明命令没被处理，两种情况都必须中止而不是继续等数据。
    # The 0.5 s reply window decides the start outcome: REFUSED means a safety gate or
    # bus voltage blocked arming, and a missing ARMED means the command was not handled;
    # both must abort instead of waiting for data that will never come.
    start_lines = collect_for(port, 0.5, raw, label)
    if any("FOC start REFUSED" in line for line in start_lines):
        raise RuntimeError(f"{label}: controller refused to start")
    if not any("FOC ARMED" in line for line in start_lines):
        raise RuntimeError(f"{label}: no FOC ARMED acknowledgement")

    collect_for(port, duration, raw, label)
    # 先停机再取 FTIMING/状态：WCET 数字必须在功率级关闭后读取，
    # 否则读到的可能是一次正在进行的 ISR。
    # Stop before reading FTIMING/status: the WCET record must be read with the power
    # stage off, otherwise it could describe an ISR still in flight.
    send(port, "foc_stop")
    collect_for(port, 0.2, raw, label)
    send(port, "foc_status")
    timing_lines = collect_for(port, 0.5, raw, label)
    # 取最后一条 FTIMING：它就是本次运行结束后最新的 WCET 记录。
    # Takes the last FTIMING line, i.e. the freshest WCET record after this run.
    timing = next(
        (parsed for line in reversed(timing_lines) if (parsed := parse_timing(line)) is not None),
        None,
    )
    if trace_supported:
        send(port, "foc_trace stop")
        collect_for(port, 0.1, raw, label)
    if timing is None:
        raise RuntimeError(f"{label}: no FTIMING record received")
    # 自检项都是"结构性"判据，不是性能阈值：只判断数据本身是否可信、
    # 以及硬性错误计数是否为零。绝对 cycles 是否达标由报告离线判断。
    # The checks are structural, not performance thresholds: they test whether the data
    # itself is trustworthy and whether hard error counters are zero. Judging absolute
    # cycle counts against the budget is done offline in the report.
    issues: list[str] = []
    if timing["version"] != EXPECTED_TIMING_VERSION:
        issues.append(
            f"timing format mismatch: expected {EXPECTED_TIMING_VERSION}, got {timing['version']}"
        )
    if timing["samples"] == 0:
        issues.append("no realtime timing samples")
    if timing["invalid"] != 0:
        issues.append("invalid correlated timing sample")
    # total 必须是同一次 ISR 的分段之和；不相等说明三个分段来自不同样本，
    # 相加得到的"总耗时"没有物理意义。
    # `total` must equal the sum of the segments of one ISR sample; otherwise the
    # segments come from different samples and summing them is meaningless.
    if timing["pre"] + timing["control"] + timing["post"] != timing["total"]:
        issues.append("timing segments are not from one ISR sample")
    # trace 标志必须与本次请求一致，否则这一段 WCET 不代表被测的 trace 配置。
    # The trace flag must match the requested rate, or this WCET does not describe the
    # configuration under test.
    if timing["trace"] != (1 if rate_hz else 0):
        issues.append("trace mode mismatch in WCET sample")
    if timing["misses"] != 0:
        issues.append("ISR deadline missed")
    if timing["errors"] != 0 or timing["fault"] != 0:
        issues.append("control loop faulted")
    inverter_flags = (
        timing["inverter"],
        timing["inverter_observer"],
        timing["inverter_feedforward"],
    )
    if any(value not in (0, 1) for value in inverter_flags):
        issues.append("invalid inverter-model mode flags")
    if timing["inverter"] == 0 and (
        timing["inverter_observer"] != 0 or timing["inverter_feedforward"] != 0
    ):
        issues.append("inverter sub-gate enabled while master gate is off")
    raw.append(f"[{label}] END")
    return {
        "trace_rate_hz": rate_hz,
        "duration_s": duration,
        "target_speed_rpm": target_rpm,
        "passed": not issues,
        "issues": issues,
        # timing 原样保留固件上报的全部 FTIMING 字段，便于以后新增判据时不必重测。
        # The raw FTIMING dict is kept whole so future criteria need no re-measurement.
        "timing": timing,
    }


# 主流程：先做全部安全与参数校验，再打开串口；采完把结论写成 JSON + 原始日志。
# Main flow: all safety and argument validation happens before the port is opened, and
# the run ends by writing the JSON verdict plus the raw log.
def main() -> int:
    args = parse_args()
    rates = [int(item.strip()) for item in args.rates.split(",") if item.strip()]
    # 安全闸门放在最前面：缺少显式许可时，脚本在任何硬件动作之前就退出，
    # 避免"只想看看时序"却意外让电机上电。
    # The safety gate comes first: without the explicit permission flag the script exits
    # before any hardware action, so an accidental run cannot energize the motor.
    if not args.allow_motor_run:
        raise SystemExit("refusing to energize the motor without --allow-motor-run")
    if args.duration <= 0 or not rates or any(rate not in (0, 10, 50) for rate in rates):
        raise SystemExit("duration must be positive; rates must be selected from 0,10,50")
    # Production 档把 foc_trace 与 trace 采样一起裁掉，固件里不再存在这些命令，
    # 因此只允许 trace-off 基线；其它 rate 只会被 shell 当成未知命令拒绝。
    # The production profile compiles foc_trace and trace sampling out, so only the
    # trace-off baseline is valid; other rates would be rejected as unknown commands.
    if args.profile == "production" and any(rate != 0 for rate in rates):
        raise SystemExit("production profile compiles trace out; only --rates 0 is valid")

    # 用脚本自身位置定位工程根目录（simulation/ 的上一级），
    # 保证从任何工作目录调用都能把 git 提交号写进 JSON。
    # Locates the project root from this file's own path, so the git commit is recorded
    # no matter which working directory the script was invoked from.
    project = pathlib.Path(__file__).resolve().parents[1]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    raw: list[str] = []
    scenarios: list[dict[str, object]] = []

    with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0) as port:
        # 建立确定的初始状态：清缓冲、停机、关 trace，再（可选）临时打开闭环。
        # foc_cfg 只有在停机状态下才会被固件接受，所以这个顺序不能调换。
        # Establishes a known initial state: flush, stop, trace off, then optionally
        # enable closed loop. Firmware only accepts foc_cfg while stopped.
        time.sleep(0.25)
        port.reset_input_buffer()
        send(port, "foc_stop")
        if args.profile == "diagnostic":
            send(port, "foc_trace stop")
        if args.closed_loop:
            send(port, "foc_cfg closedloop 1")
        collect_for(port, 0.25, raw, "setup")
        try:
            for rate_hz in rates:
                scenarios.append(
                    run_scenario(
                        port,
                        rate_hz,
                        args.duration,
                        args.target_rpm,
                        args.profile == "diagnostic",
                        raw,
                    )
                )
        finally:
            # 安全 + 现场恢复：无论哪一组抛异常，都先停机，然后关 trace、
            # 把闭环开关恢复为 0（脚本只做临时接管，绝不改变上电默认配置）。
            # Safety and site restoration: on any failure, stop the motor first, then
            # disable trace and restore closed loop to 0 — the script borrows the
            # closed-loop switch and must never leave it changed.
            send(port, "foc_stop")
            if args.profile == "diagnostic":
                send(port, "foc_trace stop")
            if args.closed_loop:
                send(port, "foc_cfg closedloop 0")
            collect_for(port, 0.25, raw, "cleanup")

    # JSON 就是本脚本的交付物：schema_version 供下游做兼容判断，
    # generated_at 用 UTC，closed_loop_requested 记录"请求值"而非实测状态。
    # The JSON is the deliverable: schema_version for downstream compatibility, a UTC
    # timestamp, and closed_loop_requested recording the request rather than the
    # observed state.
    #
    # 注意 / Caveat: 只有在全部场景跑完后才会写盘。某一组抛异常（REFUSED 或收不到
    # FTIMING）时异常会穿出 for 循环，JSON 与 .log 都不会生成，此前已采到的场景与
    # 原始串口记录一并丢失（安全停机仍然生效）。失败重跑前请留意这一点。
    # Files are written only after every scenario finished: if one raises, the JSON and the
    # raw `.log` are never produced, losing the scenarios already collected (the safety
    # stop still happens). Keep this in mind when a sweep fails midway.
    document = {
        "schema_version": 1,
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "git_commit": git_commit(project),
        "port": args.port,
        "baud": args.baud,
        "closed_loop_requested": args.closed_loop,
        "build_profile": args.profile,
        "scenarios": scenarios,
    }
    args.output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    log_path = args.output.with_suffix(".log")
    log_path.write_text("\n".join(raw) + "\n", encoding="utf-8")
    # FOC_TIMING_* 标记与退出码是 CI 契约：任一组自检失败返回 1，
    # 但 JSON 与日志仍会完整写出，便于直接看是哪一条判据不过。
    # The FOC_TIMING_* markers and the exit code are the CI contract: any failed scenario
    # returns 1, while the JSON and log are still written so the failing criterion is
    # directly visible.
    print(f"FOC_TIMING_BASELINE={args.output}")
    print(f"FOC_TIMING_RAW_LOG={log_path}")
    failed = sum(not bool(scenario["passed"]) for scenario in scenarios)
    if failed:
        print(f"FOC_TIMING_CAPTURE_FAIL scenarios={len(scenarios)} failed={failed}")
        return 1
    print(f"FOC_TIMING_CAPTURE_PASS scenarios={len(scenarios)}")
    return 0


# 串口与运行时错误转成 SystemExit 文本；注意 main() 的 finally 已经先停过机。
# Serial and runtime errors become a plain SystemExit message; main()'s `finally` has
# already issued the stop by the time this handler runs.
if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, serial.SerialException) as error:
        raise SystemExit(str(error)) from error
