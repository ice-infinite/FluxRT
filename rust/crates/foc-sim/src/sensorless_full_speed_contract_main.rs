use std::env;
use std::path::PathBuf;

use foc_sim::sensorless_full_speed_contract::run_sensorless_full_speed_contract;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|path| path.parent())
        .and_then(|path| path.parent())
        .expect("workspace layout")
        .to_path_buf();
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--root" => root = PathBuf::from(args.next().ok_or("missing --root value")?),
            "--help" | "-h" => {
                println!("foc-sensorless-full-speed-contract-check [--root PROJECT_ROOT]");
                return Ok(());
            }
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }
    let result = run_sensorless_full_speed_contract(&root)?;
    println!(
        "FLUXRT_SENSORLESS_FULL_SPEED_PASS cases={} rows={} metadata={} metrics={}",
        result.case_count,
        result.row_count,
        result.metadata_path.display(),
        result.metrics_path.display(),
    );
    Ok(())
}
