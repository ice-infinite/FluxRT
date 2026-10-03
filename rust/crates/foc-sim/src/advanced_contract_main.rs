//! Host-only advanced-FOC scenario runner. It performs no serial or target I/O.

use std::env;
use std::path::PathBuf;

use foc_sim::advanced_contract::run_advanced_contract;

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_ADVANCED_CONTRACT_ERROR: {error}");
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
                println!("foc-advanced-contract-check [--root PROJECT_ROOT]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }
    let result = run_advanced_contract(&root)?;
    println!(
        "FLUXRT_ADVANCED_CONTRACT_PASS cases={} metadata={} trace={}",
        result.case_count,
        result.metadata_path.display(),
        result.trace_path.display()
    );
    Ok(())
}
