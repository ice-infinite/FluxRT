//! D0/D1 simulation-contract checker. This binary reads only canonical files and
//! never opens serial ports or touches target hardware.

use std::env;
use std::path::PathBuf;

use foc_sim::simulation_contract::load_simulation_contract;

fn main() {
    if let Err(error) = run() {
        eprintln!("FOC_CONTRACT_ERROR: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let mut project_root = PathBuf::from(".");
    let mut output = None;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--root" => {
                project_root = PathBuf::from(
                    args.next()
                        .ok_or_else(|| "missing value after --root".to_owned())?,
                );
            }
            "--out" => {
                output = Some(PathBuf::from(
                    args.next()
                        .ok_or_else(|| "missing value after --out".to_owned())?,
                ));
            }
            "--help" | "-h" => {
                println!("foc-contract-check [--root PROJECT_ROOT] [--out RESULT_PATH]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}")),
        }
    }

    let bundle = load_simulation_contract(&project_root).map_err(|error| error.to_string())?;
    let result = bundle.gate_result();
    if let Some(path) = output {
        result.write(&path).map_err(|error| error.to_string())?;
        println!("FOC_CONTRACT_RESULT={}", path.display());
    }
    print!("{}", result.as_text());
    Ok(())
}
