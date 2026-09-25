//! `foc-bringup-sim` 命令行入口（与固件同构的启动/观测器相关性仿真）。
//! The `foc-bringup-sim` CLI entry point, the firmware-correlated simulation.
//!
//! 职责 / Responsibility:
//!   - 解析命令行参数，调用 `foc_sim::run_bringup_simulation`（该函数逐步调用固件
//!     同一套 C ABI 快环入口），可选导出 CSV，并打印机器可读的
//!     `FOC_BRINGUP_SIM_PASS` 结果行。
//!   - 真正被下游工具消费的是 **CSV 列 schema**：`simulation/capture_hardware_trace.py`
//!     按同名同序生成实机轨迹，`simulation/compare_traces.py` 与 MATLAB 叠图按列名
//!     对齐仿真与实机数据（见 `docs/仿真实机相关性验证.md`）。改列名或
//!     列序会直接破坏对标流程。
//!   - `FOC_BRINGUP_SIM_PASS` 行当前**没有** in-tree 脚本解析（`test.ps1` 只运行
//!     `--bin foc-sim`，不运行本 binary），它面向人工与 CI 阅读。格式应保持稳定，
//!     但不要把它当作已被脚本逐字段消费的接口。
//!
//!   The CSV column schema is the automated contract consumed by the Python/MATLAB
//!   correlation tools; the `FOC_BRINGUP_SIM_PASS` line is human/CI-facing and is not
//!   field-parsed by any in-tree script.
//!
//! 边界 / Boundary: 只做参数解析与输出，不含控制算法；仿真本体与实机 C ABI 调用
//!   都在 `foc-sim` 库与 `foc-rt-bridge` 里。
//!
//! 实时约束 / Real-time constraints: 离线 PC 代码，允许分配、文件 I/O 与 `println!`；
//!   固件侧对应的 12 kHz ADC ISR 快环禁止这些操作。
//!
//! 量纲 / Units: `--duration` `[s]`、`--target-rpm` `[rpm]`、`--bus-voltage` `[V]`、
//!   `--load-torque` `[N*m]`、`--sample-every` 为控制拍数、`--smo-slide` `[V]`、
//!   `--smo-boundary` `[A]`、`--emf-filter` 无量纲、`--pll-kp` `[1/s]`、
//!   `--pll-acq-ratio` 无量纲、`--pll-ki` `[1/s^2]`、
//!   `--acquire-phase-rad/--run-phase-rad` `[rad]`、
//!   `--dead-time-ns` `[ns]`、`--dead-time-gain` 无量纲、
//!   `--dead-time-current-band` `[A]`、`--current-sign-filter-alpha` 无量纲、
//!   `--inverter-device-drop-v` `[V]`。纯 `f32`，无定点 Q 格式。
//!
//! 参考 / Reference: `docs/仿真实机相关性验证.md`

use std::env;
use std::path::PathBuf;

use foc_sim::{run_bringup_simulation, BringupSimulationConfig};

/// 进程入口：失败时打印 `FOC_BRINGUP_SIM_ERROR: ...` 到 stderr 并以退出码 1 结束。
/// Process entry point; prints `FOC_BRINGUP_SIM_ERROR` and exits with code 1.
///
/// 退出码语义 / Exit codes: `0` 表示仿真成功并已打印 `FOC_BRINGUP_SIM_PASS`；
///   `1` 表示参数、配置、桥接状态或写文件失败。脚本按这两种信号判断成败，
///   所以错误前缀同样是契约的一部分。
/// The literal error prefix is part of the script contract.
fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_BRINGUP_SIM_ERROR: {error}");
        std::process::exit(1);
    }
}

/// 执行一次 bringup 仿真，按需导出 CSV，最后打印 `FOC_BRINGUP_SIM_PASS` 汇总行。
/// Runs one bringup simulation, writes the CSV if requested, prints the summary.
///
/// 返回 / Returns: 成功为 `()`；失败为人类可读错误串（由 `main` 转成
///   `FOC_BRINGUP_SIM_ERROR` 行）。桥接层错误用 `{error:?}` 的 `Debug` 形式输出。
///
/// 输出 / Output（面向人工与 CI；自动化的契约是 CSV 列 schema）:
///   给出 `--csv` 时先打印 `FOC_BRINGUP_SIM_TRACE=<path>`，供调用方定位产物；
///   `FOC_BRINGUP_SIM_PASS` 之后依次是 `true_rpm`、`observer_rpm`（`[rpm]`）、
///   `peak_phase_current_a`（`[A]`）、`reliable_samples`（计数）、
///   `hold_speed_rmse_rpm`、`hold_angle_rmse_rad`（`[rad]`）、
///   `transient_speed_rmse_rpm`、`transient_observer_rmse_rpm`、
///   `steady_speed_mean_rpm`、`steady_speed_std_rpm`、`steady_observer_rmse_rpm`
///   （`[rpm]`）、`steady_iq_rmse_a`、`steady_id_rmse_a`（`[A]`）、`final_state`
///   （固件状态数值）、`samples`（抽点样本数）。
///   字段名、顺序与 `{:.2}`/`{:.3}`/`{:.4}`/`{:.6}` 精度目前只被文档与人工对标引用，
///   没有 in-tree 脚本逐字段解析；仍应保持稳定，因为它是对外可见的输出。各 RMSE 的
///   时间窗口见 `foc_sim::BringupSimulationSummary` 的文档。
/// The `FOC_BRINGUP_SIM_PASS` line is read by humans and CI rather than parsed field by
/// field; the CSV column schema is what the correlation scripts match. Keep both stable.
fn run() -> Result<(), String> {
    let (config, csv_path) = parse_args()?;
    let simulation = run_bringup_simulation(config).map_err(|error| format!("{error:?}"))?;

    if let Some(path) = csv_path {
        simulation
            .write_csv(&path)
            .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
        println!("FOC_BRINGUP_SIM_TRACE={}", path.display());
    }

    println!(
        "FOC_BRINGUP_SIM_PASS true_rpm={:.2} observer_rpm={:.2} peak_phase_current_a={:.3} reliable_samples={} hold_speed_rmse_rpm={:.2} hold_angle_rmse_rad={:.4} transient_speed_rmse_rpm={:.3} transient_observer_rmse_rpm={:.3} steady_speed_mean_rpm={:.3} steady_speed_std_rpm={:.4} steady_observer_rmse_rpm={:.3} steady_iq_rmse_a={:.6} steady_id_rmse_a={:.6} final_state={} samples={} pwm_ticks={} control_ticks={} applied_updates={}",
        simulation.summary.final_true_speed_rpm,
        simulation.summary.final_observer_speed_rpm,
        simulation.summary.peak_phase_current_a,
        simulation.summary.observer_reliable_samples,
        simulation.summary.hold_speed_rmse_rpm,
        simulation.summary.hold_angle_rmse_rad,
        simulation.summary.transient_speed_target_rmse_rpm,
        simulation.summary.transient_observer_rmse_rpm,
        simulation.summary.steady_true_speed_mean_rpm,
        simulation.summary.steady_true_speed_std_rpm,
        simulation.summary.steady_observer_rmse_rpm,
        simulation.summary.steady_iq_tracking_rmse_a,
        simulation.summary.steady_id_rmse_a,
        simulation.summary.final_state,
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
/// 开关型选项 / Flag switches: `--closed-loop` 打开闭环接管（同时要求观测器可用）；
///   `--dead-time` 打开死区模型；`--dead-time-compensation` 等价于同时打开
///   `--dead-time-feedforward` 与 `--observer-dead-time-compensation`；
///   `--dead-time-feedforward` 只打开 PWM 前馈补偿；`--observer-dead-time-compensation`
///   只打开观测器电压重构补偿。三个补偿选项都会**自动**把 `dead_time_enabled` 置真，
///   因为配置校验不允许在死区模型关闭时打开补偿。
/// Compensation flags also enable the dead-time model, as validation requires it.
///
/// 取值型选项 / Value switches: `--csv PATH`、`--duration SECONDS` `[s]`、
///   `--target-rpm RPM` / `--initial-speed-rpm RPM` `[rpm]`、
///   `--initial-electrical-angle-rad RAD` `[rad]`、`--bus-voltage VOLTS` `[V]`、`--load-torque NM`
///   `[N*m]`、`--sample-every TICKS`、`--smo-slide V` `[V]`、`--smo-boundary A`
///   `[A]`、`--emf-filter ALPHA`、`--pll-kp VALUE` `[1/s]`、`--pll-ki VALUE`
///   `[1/s^2]`、`--dead-time-ns NS` `[ns]`、`--dead-time-gain GAIN`、
///   `--alignment-ms MS`、`--ramp-ms MS`、`--alignment-current-a A`、
///   `--startup-current-a A`、
///   `--confirm-ms MS`、`--acquire-ms MS`、`--loss-ms MS`、
///   `--handoff-support-ratio RATIO`、
///   `--dead-time-current-band A` `[A]`、`--current-sign-filter-alpha ALPHA`、
///   `--inverter-device-drop-v VOLTS` `[V]`、`--park-delay-ticks TICKS` 与
///   `--rev-park-delay-ticks TICKS` `[control ticks]`；`--help`/`-h` 打印用法后
///   退出码 0 结束。
///
/// 陷阱 / Pitfalls: 未知选项直接报错（不静默忽略）；同一选项重复给出时最后一个生效；
///   这里只做语法解析，范围校验由 `run_bringup_simulation` 完成。
/// Unknown switches are rejected; ranges are validated later.
fn parse_args() -> Result<(BringupSimulationConfig, Option<PathBuf>), String> {
    let mut config = BringupSimulationConfig::default();
    let mut csv_path = None;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        if argument == "--closed-loop" {
            config.closed_loop_enable = true;
            continue;
        }
        if argument == "--dead-time" {
            config.dead_time_enabled = true;
            continue;
        }
        if argument == "--dead-time-compensation" {
            config.dead_time_enabled = true;
            config.dead_time_feedforward_enabled = true;
            config.observer_dead_time_compensation_enabled = true;
            continue;
        }
        if argument == "--dead-time-feedforward" {
            config.dead_time_enabled = true;
            config.dead_time_feedforward_enabled = true;
            continue;
        }
        if argument == "--observer-dead-time-compensation" {
            config.dead_time_enabled = true;
            config.observer_dead_time_compensation_enabled = true;
            continue;
        }
        let value = match argument.as_str() {
            "--csv"
            | "--duration"
            | "--target-rpm"
            | "--initial-speed-rpm"
            | "--initial-electrical-angle-rad"
            | "--alignment-ms"
            | "--ramp-ms"
            | "--startup-speed-rpm"
            | "--alignment-current-a"
            | "--startup-current-a"
            | "--bus-voltage"
            | "--load-torque"
            | "--sample-every"
            | "--pwm-hz"
            | "--control-hz"
            | "--actuation-delay-pwm-ticks"
            | "--park-delay-ticks"
            | "--rev-park-delay-ticks"
            | "--smo-slide"
            | "--smo-boundary"
            | "--emf-filter"
            | "--pll-kp"
            | "--pll-acq-ratio"
            | "--pll-ki"
            | "--acquire-phase-rad"
            | "--run-phase-rad"
            | "--confirm-ms"
            | "--acquire-ms"
            | "--loss-ms"
            | "--transition-ms"
            | "--handoff-support-ratio"
            | "--speed-preload-ratio"
            | "--iq-slew-a-per-s"
            | "--dead-time-ns"
            | "--dead-time-gain"
            | "--dead-time-current-band"
            | "--current-sign-filter-alpha"
            | "--inverter-device-drop-v" => args
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
            "--initial-speed-rpm" => config.initial_speed_rpm = parse_number(&argument, &value)?,
            "--initial-electrical-angle-rad" => {
                config.initial_electrical_angle_rad = parse_number(&argument, &value)?
            }
            "--alignment-ms" => {
                let milliseconds: f32 = parse_number(&argument, &value)?;
                config.startup_alignment_s = milliseconds / 1_000.0;
            }
            "--ramp-ms" => {
                let milliseconds: f32 = parse_number(&argument, &value)?;
                config.startup_ramp_s = milliseconds / 1_000.0;
            }
            "--startup-speed-rpm" => {
                config.startup_final_speed_rpm = parse_number(&argument, &value)?
            }
            "--alignment-current-a" => {
                config.startup_alignment_current_a = parse_number(&argument, &value)?
            }
            "--startup-current-a" => config.startup_current_a = parse_number(&argument, &value)?,
            "--bus-voltage" => config.dc_bus_voltage_v = parse_number(&argument, &value)?,
            "--load-torque" => config.load_torque_nm = parse_number(&argument, &value)?,
            "--sample-every" => config.trace_decimation = parse_number(&argument, &value)?,
            "--pwm-hz" => config.timing.pwm_frequency_hz = parse_number(&argument, &value)?,
            "--control-hz" => config.timing.control_frequency_hz = parse_number(&argument, &value)?,
            "--actuation-delay-pwm-ticks" => {
                config.timing.actuation_delay_pwm_ticks = parse_number(&argument, &value)?
            }
            "--park-delay-ticks" => config.park_prediction_ticks = parse_number(&argument, &value)?,
            "--rev-park-delay-ticks" => {
                config.reverse_park_prediction_ticks = parse_number(&argument, &value)?
            }
            "--smo-slide" => config.observer_smo_k_slide_v = parse_number(&argument, &value)?,
            "--smo-boundary" => config.observer_smo_boundary_a = parse_number(&argument, &value)?,
            "--emf-filter" => config.observer_emf_filter_alpha = parse_number(&argument, &value)?,
            "--pll-kp" => config.observer_pll_kp = parse_number(&argument, &value)?,
            "--pll-acq-ratio" => {
                config.observer_acquisition_pll_kp_ratio = parse_number(&argument, &value)?
            }
            "--pll-ki" => config.observer_pll_ki = parse_number(&argument, &value)?,
            "--acquire-phase-rad" => {
                config.observer_acquisition_maximum_phase_error_rad =
                    parse_number(&argument, &value)?
            }
            "--run-phase-rad" => {
                config.observer_run_maximum_phase_error_rad = parse_number(&argument, &value)?
            }
            "--confirm-ms" => {
                config.observer_consecutive_samples = parse_number(&argument, &value)?
            }
            "--acquire-ms" => {
                let milliseconds: f32 = parse_number(&argument, &value)?;
                config.observer_acquisition_timeout_s = milliseconds / 1_000.0;
            }
            "--loss-ms" => {
                let milliseconds: f32 = parse_number(&argument, &value)?;
                config.observer_loss_timeout_s = milliseconds / 1_000.0;
            }
            "--transition-ms" => {
                let milliseconds: f32 = parse_number(&argument, &value)?;
                config.startup_transition_s = milliseconds / 1_000.0;
            }
            "--handoff-support-ratio" => {
                config.handoff_torque_support_ratio = parse_number(&argument, &value)?
            }
            "--speed-preload-ratio" => {
                config.speed_pi_preload_ratio = parse_number(&argument, &value)?
            }
            "--iq-slew-a-per-s" => {
                config.closed_loop_current_slew_a_per_s = parse_number(&argument, &value)?
            }
            "--dead-time-ns" => config.dead_time_ns = parse_number(&argument, &value)?,
            "--dead-time-gain" => {
                config.dead_time_compensation_gain = parse_number(&argument, &value)?
            }
            "--dead-time-current-band" => {
                config.dead_time_current_zero_band_a = parse_number(&argument, &value)?
            }
            "--current-sign-filter-alpha" => {
                config.inverter_current_sign_filter_alpha = parse_number(&argument, &value)?
            }
            "--inverter-device-drop-v" => {
                config.inverter_device_drop_v = parse_number(&argument, &value)?
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
///   字符串，便于发现拼写或误带单位。
/// Generic over `FromStr`; the original text is echoed to aid diagnosis.
fn parse_number<T: std::str::FromStr>(name: &str, value: &str) -> Result<T, String> {
    value
        .parse()
        .map_err(|_| format!("invalid value for {name}: {value}"))
}

/// 打印单行用法说明（`--help`/`-h` 触发，随后以退出码 0 结束）。
/// Prints the one-line usage string, then the caller exits with code 0.
///
/// 用法里列出全部开关，必须与 `parse_args` 保持同步，否则出现"帮助里有、解析不认"
///   的选项；对标文档中的复现命令引用同一组开关名。
/// Keep this in sync with the switch list in `parse_args`.
fn print_help() {
    println!(
        "foc-bringup-sim [--csv PATH] [--duration SECONDS] [--target-rpm RPM] \
         [--initial-speed-rpm RPM] [--initial-electrical-angle-rad RAD] \
         [--closed-loop] [--alignment-ms MS] [--ramp-ms MS] [--startup-speed-rpm RPM] \
         [--alignment-current-a A] \
         [--startup-current-a A] \
         [--bus-voltage VOLTS] [--load-torque NM] [--sample-every TICKS] \
         [--pwm-hz HZ] [--control-hz HZ] [--actuation-delay-pwm-ticks TICKS] \
         [--park-delay-ticks TICKS] [--rev-park-delay-ticks TICKS] \
         [--smo-slide V] [--smo-boundary A] [--emf-filter ALPHA] \
         [--pll-kp VALUE] [--pll-acq-ratio RATIO] [--pll-ki VALUE] \
         [--acquire-phase-rad RAD] [--run-phase-rad RAD] \
         [--confirm-ms MS] [--acquire-ms MS] [--loss-ms MS] [--transition-ms MS] \
         [--handoff-support-ratio RATIO] [--speed-preload-ratio RATIO] \
         [--iq-slew-a-per-s A_PER_S] \
         [--dead-time] [--dead-time-compensation] \
         [--dead-time-feedforward] [--observer-dead-time-compensation] \
         [--dead-time-ns NS] [--dead-time-gain GAIN] [--dead-time-current-band A] \
         [--current-sign-filter-alpha ALPHA] [--inverter-device-drop-v VOLTS]"
    );
}
