//! P4.2D Rust motion-contract runner. It performs host-only file I/O and never
//! opens a serial port or touches target hardware.

use std::env;
use std::path::PathBuf;

use foc_sim::motion_contract::{default_output_paths, run_motion_contract};

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_MOTION_CONTRACT_ERROR: {error}");
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
                println!("foc-motion-contract-check [--root PROJECT_ROOT]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }

    let result = run_motion_contract(&root)?;
    result.write_default_outputs(&root)?;
    let (metadata, trace) = default_output_paths(&root);
    println!(
        "FLUXRT_MOTION_CONTRACT_PASS D0=PASS D1=PASS D2=PASS D3=PASS D4=PASS cases={} rows={} metadata={} trace={}",
        result.case_count,
        result.row_count,
        metadata.display(),
        trace.display()
    );
    Ok(())
}
