use std::env;
use std::path::PathBuf;

use foc_sim::{run_reference_simulation, SimulationConfig};

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_SIM_ERROR: {error}");
        std::process::exit(1);
    }
}

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
        "FOC_SIM_PASS final_rpm={:.2} target_rpm={:.2} peak_phase_current_a={:.3} load_nm={:.4} samples={}",
        simulation.summary.final_speed_rpm,
        simulation.summary.target_speed_rpm,
        simulation.summary.peak_phase_current_a,
        simulation.summary.load_torque_nm,
        simulation.summary.sample_count,
    );
    Ok(())
}

fn parse_args() -> Result<(SimulationConfig, Option<PathBuf>), String> {
    let mut config = SimulationConfig::default();
    let mut csv_path = None;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        let value = match argument.as_str() {
            "--csv" | "--duration" | "--target-rpm" | "--load-step-time" | "--load-torque"
            | "--sample-every" => args
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
        "foc-sim [--csv PATH] [--duration SECONDS] [--target-rpm RPM] \
         [--load-step-time SECONDS] [--load-torque NM] [--sample-every TICKS]"
    );
}
