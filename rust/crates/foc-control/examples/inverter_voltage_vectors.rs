//! 生成 CM2 Rust/MATLAB 逆变器电压模型对拍向量。

use std::error::Error;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

use foc_control::{
    InverterVoltageModel, InverterVoltageModelConfig, ObserverVoltageSource, PhaseCurrents,
    PwmCommand,
};

#[derive(Clone, Copy)]
struct Vector {
    pwm: PwmCommand,
    currents: PhaseCurrents,
    vbus: f32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let output = std::env::args_os()
        .nth(1)
        .map(PathBuf::from)
        .ok_or("usage: inverter_voltage_vectors <output.csv>")?;
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }

    let config = InverterVoltageModelConfig {
        enabled: true,
        dead_time_s: 550.0e-9,
        pwm_period_s: 1.0 / 12_000.0,
        compensation_gain: 0.75,
        current_zero_band_a: 0.005,
        current_sign_filter_alpha: 0.25,
        device_drop_v: 0.08,
        observer_voltage_correction_enabled: true,
        feedforward_enabled: true,
        source: ObserverVoltageSource::CommandModel,
    };
    let vectors = [
        Vector {
            pwm: pwm(0.60, 0.40, 0.50),
            currents: currents(0.020, -0.015, -0.005),
            vbus: 12.3,
        },
        Vector {
            pwm: pwm(0.58, 0.42, 0.50),
            currents: currents(0.004, -0.004, 0.000),
            vbus: 12.3,
        },
        Vector {
            pwm: pwm(0.55, 0.45, 0.50),
            currents: currents(-0.004, 0.004, 0.000),
            vbus: 12.3,
        },
        Vector {
            pwm: pwm(0.52, 0.48, 0.50),
            currents: currents(-0.020, 0.015, 0.005),
            vbus: 12.3,
        },
        Vector {
            pwm: pwm(0.98, 0.02, 0.50),
            currents: currents(-0.020, 0.010, 0.010),
            vbus: 10.0,
        },
        Vector {
            pwm: pwm(0.02, 0.98, 0.50),
            currents: currents(0.020, -0.010, -0.010),
            vbus: 15.0,
        },
        Vector {
            pwm: pwm(0.50, 0.50, 0.50),
            currents: currents(0.000, 0.000, 0.000),
            vbus: 12.3,
        },
    ];

    let mut model = InverterVoltageModel::try_new(config).ok_or("invalid model config")?;
    let mut writer = BufWriter::new(File::create(&output)?);
    writeln!(
        writer,
        "step,pwm_a,pwm_b,pwm_c,current_a,current_b,current_c,vbus_v,dead_time_s,pwm_period_s,device_drop_v,zero_band_a,filter_alpha,compensation_gain,filtered_a,filtered_b,filtered_c,loss_a_v,loss_b_v,loss_c_v,observer_pwm_a,observer_pwm_b,observer_pwm_c,feedforward_pwm_a,feedforward_pwm_b,feedforward_pwm_c"
    )?;
    for (step, vector) in vectors.into_iter().enumerate() {
        let filtered = model.update_phase_currents(vector.currents);
        let loss = model.phase_voltage_loss_v(vector.vbus);
        let observer = model.observer_equivalent_pwm(vector.pwm, vector.vbus);
        let feedforward = model.feedforward_pwm(vector.pwm, vector.vbus);
        writeln!(
            writer,
            "{step},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.12},{:.12},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9}",
            vector.pwm.duty_a,
            vector.pwm.duty_b,
            vector.pwm.duty_c,
            vector.currents.a,
            vector.currents.b,
            vector.currents.c,
            vector.vbus,
            config.dead_time_s,
            config.pwm_period_s,
            config.device_drop_v,
            config.current_zero_band_a,
            config.current_sign_filter_alpha,
            config.compensation_gain,
            filtered.a,
            filtered.b,
            filtered.c,
            loss.a,
            loss.b,
            loss.c,
            observer.duty_a,
            observer.duty_b,
            observer.duty_c,
            feedforward.duty_a,
            feedforward.duty_b,
            feedforward.duty_c,
        )?;
    }
    writer.flush()?;
    println!("INVERTER_MODEL_RUST_VECTORS={}", output.display());
    Ok(())
}

fn pwm(a: f32, b: f32, c: f32) -> PwmCommand {
    PwmCommand {
        duty_a: a,
        duty_b: b,
        duty_c: c,
    }
}

fn currents(a: f32, b: f32, c: f32) -> PhaseCurrents {
    PhaseCurrents { a, b, c }
}
