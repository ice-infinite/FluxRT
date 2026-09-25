//! `foc-sim` 的参考仿真命令行入口（理想转子反馈的控制律回归）。
//! The reference-simulation CLI entry point of the `foc-sim` crate.
//!
//! 职责 / Responsibility:
//!   - 解析命令行参数，调用 `foc_sim::run_reference_simulation`，可选导出 CSV，
//!     并把汇总打印成机器可读的 `FOC_SIM_PASS` 行。
//!   - `test.ps1` 会运行本 binary（`--bin foc-sim`）并以**退出码**判定成败；
//!     `FOC_SIM_PASS` 行本身由文档与人工对标引用（见 `docs/工程操作日志.md`
//!     的 S3 记录）。没有 in-tree 脚本逐字段解析该行，但字段名、顺序与小数位数是对外
//!     可见的输出，改动会让文档记录与实际输出不一致。
//!
//! 边界 / Boundary: 本文件只做参数解析与输出，不含任何控制算法；对应实机行为的
//!   仿真在 `foc-bringup-sim` 里（走固件 C ABI）。
//!
//! 实时约束 / Real-time constraints: 这是离线 PC 代码，允许分配、文件 I/O 与
//!   `println!`；真实 12 kHz ADC ISR 快环禁止这些操作，两者不可混用。
//!
//! 量纲 / Units: `--duration` `[s]`、`--target-rpm` `[rpm]`、`--load-torque`
//!   `[N*m]`、`--load-step-time` `[s]`、`--sample-every` 为控制拍数；`--csv` 为
//!   输出路径。纯 `f32`，无定点 Q 格式。
//!
//! 参考 / Reference: `docs/仿真实机相关性验证.md`

use std::env;
use std::path::PathBuf;

use foc_sim::{run_reference_simulation, SimulationConfig};

/// 进程入口：把 `run()` 的错误打印成 `FOC_SIM_ERROR: ...` 到 stderr，并以退出码 1
/// 结束；成功不打印额外内容。
/// Process entry point; prints `FOC_SIM_ERROR` to stderr and exits with code 1.
///
/// 退出码语义 / Exit codes: `0` 表示仿真成功并已打印 `FOC_SIM_PASS`；`1` 表示参数、
///   配置或写入失败。`test.ps1` 据此判断成败，所以退出码与错误前缀 `FOC_SIM_ERROR:`
///   都属于对外可见的约定。
/// `test.ps1` checks the exit code, not the text of the error prefix.
fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_SIM_ERROR: {error}");
        std::process::exit(1);
    }
}

/// 执行一次参考仿真，按需导出 CSV，最后打印 `FOC_SIM_PASS` 汇总行。
/// Runs one reference simulation, writes the CSV if requested, prints the summary.
///
/// 返回 / Returns: 成功为 `()`；失败为人类可读的错误串，由 `main` 转成
///   `FOC_SIM_ERROR` 行。注意配置错误用 `{error:?}` 的 `Debug` 形式，因为
///   `SimulationError` 只实现了 `Debug`（消息是英文静态串，不在脚本契约内）。
///
/// 输出 / Output（`test.ps1` 按退出码判定；下列文本面向人工与 CI）:
///   `FOC_SIM_TRACE=<path>` 只在给出 `--csv` 时打印，供调用方定位产物；
///   `FOC_SIM_PASS final_rpm=... target_rpm=... peak_phase_current_a=...
///   load_nm=... samples=...` 的字段名、顺序与小数位数（`{:.2}`、`{:.3}`、
///   `{:.4}`）是对外可见输出，文档按此引用；没有 in-tree 脚本逐字段解析。
/// The `FOC_SIM_PASS` fields are a documented, human/CI-read interface; `test.ps1`
/// runs this binary and judges by exit code only.
fn run() -> Result<(), String> {
    let (config, csv_path) = parse_args()?;
    let simulation = run_reference_simulation(config).map_err(|error| format!("{error:?}"))?;

    if let Some(path) = csv_path {
        simulation
            .write_csv(&path)
            .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
        println!("FOC_SIM_TRACE={}", path.display());
    }

    println!(
        "FOC_SIM_PASS final_rpm={:.2} target_rpm={:.2} peak_phase_current_a={:.3} load_nm={:.4} samples={} pwm_ticks={} control_ticks={} applied_updates={}",
        simulation.summary.final_speed_rpm,
        simulation.summary.target_speed_rpm,
        simulation.summary.peak_phase_current_a,
        simulation.summary.load_torque_nm,
        simulation.summary.sample_count,
        simulation.summary.pwm_tick_count,
        simulation.summary.control_tick_count,
        simulation.summary.applied_pwm_update_count,
    );
    Ok(())
}

/// 解析命令行参数，返回场景配置与可选的 CSV 输出路径。
/// Parses the command line into a scenario config and an optional CSV path.
///
/// 选项 / Switches: `--csv PATH`、`--duration SECONDS` `[s]`、`--target-rpm RPM`
///   `[rpm]`、`--load-step-time SECONDS` `[s]`、`--load-torque NM` `[N*m]`、
///   `--sample-every TICKS`（抽点控制拍数）；`--help`/`-h` 打印用法后以退出码 0 结束。
///
/// 陷阱 / Pitfalls: 取值选项缺少后继参数时报 `missing value after ...`；未知选项
///   直接报错而**不是**静默忽略，避免拼错参数悄悄跑成默认场景；参数按出现顺序覆盖
///   默认值，同一选项重复给出时最后一个生效。这里只做语法解析，不做范围校验——
///   非法范围由 `run_reference_simulation` 返回 `InvalidConfig`。
/// Unknown switches are rejected rather than ignored; ranges are validated later.
fn parse_args() -> Result<(SimulationConfig, Option<PathBuf>), String> {
    let mut config = SimulationConfig::default();
    let mut csv_path = None;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        let value = match argument.as_str() {
            "--csv"
            | "--duration"
            | "--target-rpm"
            | "--load-step-time"
            | "--load-torque"
            | "--sample-every"
            | "--pwm-hz"
            | "--control-hz"
            | "--actuation-delay-pwm-ticks" => args
                .next()
                .ok_or_else(|| format!("missing value after {argument}"))?,
            "--help" | "-h" => {
                print_help();
                std::process::exit(0);
            }
            _ => return Err(format!("unknown argument: {argument}")),
        };
        match argument.as_str() {
            "--csv" => csv_path = Some(PathBuf::from(value)),
            "--duration" => config.duration_s = parse_number(&argument, &value)?,
            "--target-rpm" => config.target_speed_rpm = parse_number(&argument, &value)?,
            "--load-step-time" => config.load_step_time_s = parse_number(&argument, &value)?,
            "--load-torque" => config.load_torque_nm = parse_number(&argument, &value)?,
            "--sample-every" => config.trace_decimation = parse_number(&argument, &value)?,
            "--pwm-hz" => config.timing.pwm_frequency_hz = parse_number(&argument, &value)?,
            "--control-hz" => config.timing.control_frequency_hz = parse_number(&argument, &value)?,
            "--actuation-delay-pwm-ticks" => {
                config.timing.actuation_delay_pwm_ticks = parse_number(&argument, &value)?
            }
            _ => unreachable!(),
        }
    }
    Ok((config, csv_path))
}

/// 把字符串解析为 `T`，失败时返回带选项名的错误文本。
/// Parses a numeric argument, naming the switch on failure.
///
/// 泛型 `T: FromStr` 同时服务 `f32` 与 `u32`（`--sample-every`）；错误信息保留原始
///   字符串，便于发现拼写或误带单位（例如把 `12.3V` 当伏特写进去）。
/// Generic over `FromStr`; the original text is echoed to aid diagnosis.
fn parse_number<T: std::str::FromStr>(name: &str, value: &str) -> Result<T, String> {
    value
        .parse()
        .map_err(|_| format!("invalid value for {name}: {value}"))
}

/// 打印单行用法说明（`--help`/`-h` 触发，随后以退出码 0 结束）。
/// Prints the one-line usage string, then the caller exits with code 0.
///
/// 说明文本与 README/文档中的调用示例保持一致；改选项名时必须同时改这里与
///   `parse_args`，否则出现"帮助里有、解析不认"的选项。
/// Keep this in sync with the switch list in `parse_args`.
fn print_help() {
    println!(
        "foc-sim [--csv PATH] [--duration SECONDS] [--target-rpm RPM] \
         [--load-step-time SECONDS] [--load-torque NM] [--sample-every TICKS] \
         [--pwm-hz HZ] [--control-hz HZ] [--actuation-delay-pwm-ticks TICKS]"
    );
}
