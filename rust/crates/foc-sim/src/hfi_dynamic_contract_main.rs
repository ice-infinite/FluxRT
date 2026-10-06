//! Host-only dynamic active-HFI contract runner. It performs no target I/O.

use std::env;
use std::path::PathBuf;

use foc_sim::hfi_dynamic_contract::run_hfi_dynamic_contract;

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_HFI_DYNAMIC_CONTRACT_ERROR: {error}");
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
                println!("foc-hfi-dynamic-contract-check [--root PROJECT_ROOT]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }
    let result = run_hfi_dynamic_contract(&root)?;
    println!(
        "FLUXRT_HFI_DYNAMIC_CONTRACT_PASS cases={} metadata={} metrics={}",
        result.case_count,
        result.metadata_path.display(),
        result.metrics_path.display()
    );
    Ok(())
}
