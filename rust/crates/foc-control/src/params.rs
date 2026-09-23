use core::f32::consts::PI;

use foc_algorithm::PiParam;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorParameters {
    pub pole_pairs: u8,
    pub stator_resistance_ohm: f32,
    pub ld_h: f32,
    pub lq_h: f32,
    pub flux_linkage_wb: f32,
    pub rated_current_a: f32,
    pub max_speed_rpm: f32,
    pub nominal_bus_voltage_v: f32,
    pub inertia_kg_m2: f32,
    pub viscous_friction_nm_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlParameters {
    pub motor: MotorParameters,
    pub id_pi: PiParam,
    pub iq_pi: PiParam,
    pub speed_pi: PiParam,
    pub pwm_frequency_hz: u32,
    pub speed_loop_frequency_hz: u32,
    pub voltage_utilization: f32,
    pub default_target_speed_rpm: f32,
}

pub const ST_RAW_CURRENT_KP: f32 = 3378.0 / 1024.0;
pub const ST_RAW_CURRENT_KI_PER_TICK: f32 = 2252.0 / 4096.0;
pub const ST_RAW_SPEED_KP: f32 = 2730.0 / 256.0;
pub const ST_RAW_SPEED_KI_PER_TICK: f32 = 562.0 / 16384.0;
pub const ST_CURRENT_CONVERSION_COUNTS_PER_AMP: f32 = 65536.0 * 0.33 * 1.53 / 3.3;

/// Parameters transcribed from the generated MCSDK 6.4.1 project for
/// NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T.
///
/// MCSDK stores PI gains in ADC/current and normalized-voltage integer units.
/// The formulas below preserve those gains while converting them to SI units.
/// The Workbench mechanical values (0.291 and 0.937) are simulation estimates;
/// they must be identified again on the physical motor/load before tuning.
pub fn st_gbm2804_reference_parameters() -> ControlParameters {
    const MCSDK_REFERENCE_PWM_HZ: f32 = 30_000.0;
    const CONTROL_PWM_HZ: f32 = 12_000.0;
    const SPEED_HZ: f32 = 1_000.0;
    const BUS_V: f32 = 13.0;
    const VOLTAGE_UTILIZATION: f32 = 0.95;
    let max_voltage = BUS_V * VOLTAGE_UTILIZATION / libm::sqrtf(3.0);
    let volts_per_normalized_count = max_voltage / 32767.0;
    let current_gain_scale = ST_CURRENT_CONVERSION_COUNTS_PER_AMP * volts_per_normalized_count;
    let current_kp = ST_RAW_CURRENT_KP * current_gain_scale;
    // Preserve the reference project's continuous-time integral gain while
    // running this project's current loop at 12 kHz.
    let current_ki = ST_RAW_CURRENT_KI_PER_TICK * current_gain_scale * MCSDK_REFERENCE_PWM_HZ;

    // SPEED_UNIT is U_01HZ: 10 units/Hz. Convert the MCSDK input to rad/s.
    let speed_units_per_rad_s = 10.0 / (2.0 * PI);
    let speed_kp = ST_RAW_SPEED_KP * speed_units_per_rad_s / ST_CURRENT_CONVERSION_COUNTS_PER_AMP;
    let speed_ki = ST_RAW_SPEED_KI_PER_TICK * speed_units_per_rad_s
        / ST_CURRENT_CONVERSION_COUNTS_PER_AMP
        * SPEED_HZ;

    ControlParameters {
        motor: MotorParameters {
            pole_pairs: 7,
            stator_resistance_ohm: 5.29,
            ld_h: 0.001_058,
            lq_h: 0.001_058,
            // Workbench M1_MOTOR_RATED_FLUX is in its internal angular basis.
            flux_linkage_wb: 0.034_739_897 / (2.0 * PI),
            rated_current_a: 0.8,
            max_speed_rpm: 1572.0,
            nominal_bus_voltage_v: BUS_V,
            // Workbench raw values are retained in documentation. These SI
            // conversions are only a host-plant starting point, not identified data.
            inertia_kg_m2: 0.291e-4,
            viscous_friction_nm_s: 0.937e-5,
        },
        id_pi: PiParam {
            kp: current_kp,
            ki: current_ki,
            ts: 1.0 / CONTROL_PWM_HZ,
            out_min: -max_voltage,
            out_max: max_voltage,
            integrator_min: -max_voltage,
            integrator_max: max_voltage,
        },
        iq_pi: PiParam {
            kp: current_kp,
            ki: current_ki,
            ts: 1.0 / CONTROL_PWM_HZ,
            out_min: -max_voltage,
            out_max: max_voltage,
            integrator_min: -max_voltage,
            integrator_max: max_voltage,
        },
        speed_pi: PiParam {
            kp: speed_kp,
            ki: speed_ki,
            ts: 1.0 / SPEED_HZ,
            out_min: -0.8,
            out_max: 0.8,
            integrator_min: -0.8,
            integrator_max: 0.8,
        },
        pwm_frequency_hz: CONTROL_PWM_HZ as u32,
        speed_loop_frequency_hz: SPEED_HZ as u32,
        voltage_utilization: VOLTAGE_UTILIZATION,
        default_target_speed_rpm: 524.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_parameters_preserve_generated_ratios() {
        let p = st_gbm2804_reference_parameters();
        assert_eq!(p.pwm_frequency_hz, 12_000);
        assert_eq!(p.speed_loop_frequency_hz, 1_000);
        assert_eq!(p.motor.pole_pairs, 7);
        assert!((p.id_pi.kp - 7.19).abs() < 0.6);
        assert_eq!(p.id_pi, p.iq_pi);
        assert!((p.speed_pi.kp - 0.001_7).abs() < 0.000_1);
    }
}
