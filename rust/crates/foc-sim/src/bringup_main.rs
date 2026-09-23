use std::env;
use std::path::PathBuf;

use foc_sim::{run_bringup_simulation, BringupSimulationConfig};

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_BRINGUP_SIM_ERROR: {error}");
        std::process::exit(1);
    }
}

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
        "FOC_BRINGUP_SIM_PASS true_rpm={:.2} observer_rpm={:.2} peak_phase_current_a={:.3} reliable_samples={} hold_speed_rmse_rpm={:.2} hold_angle_rmse_rad={:.4} transient_speed_rmse_rpm={:.3} transient_observer_rmse_rpm={:.3} steady_speed_mean_rpm={:.3} steady_speed_std_rpm={:.4} steady_observer_rmse_rpm={:.3} steady_iq_rmse_a={:.6} steady_id_rmse_a={:.6} final_state={} samples={}",
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
    );
    Ok(())
}

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
            | "--bus-voltage"
            | "--load-torque"
            | "--sample-every"
            | "--smo-slide"
            | "--smo-boundary"
            | "--emf-filter"
            | "--pll-kp"
            | "--pll-ki"
            | "--dead-time-ns"
            | "--dead-time-gain"
            | "--dead-time-current-band" => args
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
            "--bus-voltage" => config.dc_bus_voltage_v = parse_number(&argument, &value)?,
            "--load-torque" => config.load_torque_nm = parse_number(&argument, &value)?,
            "--sample-every" => config.trace_decimation = parse_number(&argument, &value)?,
            "--smo-slide" => config.observer_smo_k_slide_v = parse_number(&argument, &value)?,
            "--smo-boundary" => config.observer_smo_boundary_a = parse_number(&argument, &value)?,
            "--emf-filter" => config.observer_emf_filter_alpha = parse_number(&argument, &value)?,
            "--pll-kp" => config.observer_pll_kp = parse_number(&argument, &value)?,
            "--pll-ki" => config.observer_pll_ki = parse_number(&argument, &value)?,
            "--dead-time-ns" => config.dead_time_ns = parse_number(&argument, &value)?,
            "--dead-time-gain" => {
                config.dead_time_compensation_gain = parse_number(&argument, &value)?
            }
            "--dead-time-current-band" => {
                config.dead_time_current_zero_band_a = parse_number(&argument, &value)?
            }
            _ => unreachable!(),
        }
    }
    Ok((config, csv_path))
}

fn parse_number<T: std::str::FromStr>(name: &str, value: &str) -> Result<T, String> {
    value
        .parse()
        .map_err(|_| format!("invalid value for {name}: {value}"))
}

fn print_help() {
    println!(
        "foc-bringup-sim [--csv PATH] [--duration SECONDS] [--target-rpm RPM] \
         [--closed-loop] [--bus-voltage VOLTS] [--load-torque NM] [--sample-every TICKS] \
         [--smo-slide V] [--smo-boundary A] [--emf-filter ALPHA] \
         [--pll-kp VALUE] [--pll-ki VALUE] [--dead-time] [--dead-time-compensation] \
         [--dead-time-feedforward] [--observer-dead-time-compensation] \
         [--dead-time-ns NS] [--dead-time-gain GAIN] [--dead-time-current-band A]"
    );
}
