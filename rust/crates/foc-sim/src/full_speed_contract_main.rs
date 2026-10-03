use std::env;
use std::path::PathBuf;

use foc_sim::full_speed_contract::run_full_speed_contract;

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_FULL_SPEED_CONTRACT_ERROR: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut root = PathBuf::from(".");
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--root" => root = PathBuf::from(args.next().ok_or("--root requires a path")?),
            "--help" | "-h" => {
                println!("foc-full-speed-contract-check [--root PROJECT_ROOT]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }
    let result = run_full_speed_contract(&root)?;
    println!(
        "FLUXRT_FULL_SPEED_CONTRACT_PASS cases={} rows={} metadata={} trace={} metrics={}",
        result.case_count,
        result.row_count,
        result.metadata_path.display(),
        result.trace_path.display(),
        result.metrics_path.display(),
    );
    Ok(())
}
