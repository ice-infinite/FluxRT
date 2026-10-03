use std::env;
use std::path::PathBuf;

use foc_sim::simulation_contract::feedback_sensor::run_feedback_sensor_gate;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut root = PathBuf::from(".");
    let mut output = PathBuf::from("simulation/results/feedback");
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--root" => root = PathBuf::from(args.next().ok_or("--root requires a path")?),
            "--out" => output = PathBuf::from(args.next().ok_or("--out requires a path")?),
            _ => return Err(format!("unknown argument: {argument}").into()),
        }
    }
    let result = run_feedback_sensor_gate(&root)?;
    result.write(&output)?;
    println!(
        "FLUXRT_FEEDBACK_SENSOR_PASS D0=PASS D2=PASS D3=PASS D4=PASS feature_off=PASS cases={} rows={}",
        result
            .metadata
            .iter()
            .find(|(key, _)| key == "case_count")
            .map(|(_, value)| value.as_str())
            .unwrap_or("0"),
        result.rows.len()
    );
    Ok(())
}
