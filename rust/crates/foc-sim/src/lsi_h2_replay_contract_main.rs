//! Host-only H2 replay runner. It performs no target or serial I/O.

use std::env;
use std::path::PathBuf;

use foc_sim::lsi_h2_replay_contract::run_lsi_h2_replay_contract;

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_LSI_H2_REPLAY_ERROR: {error}");
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
                println!("foc-lsi-h2-replay-contract-check [--root PROJECT_ROOT]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }
    let result = run_lsi_h2_replay_contract(&root)?;
    println!(
        "FLUXRT_LSI_H2_REPLAY_PASS cases={} rows={} metadata={} trace={}",
        result.case_count,
        result.row_count,
        result.metadata_path.display(),
        result.trace_path.display()
    );
    Ok(())
}
