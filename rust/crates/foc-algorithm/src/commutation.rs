//! Commutation_SixStep 的等价实现。

use crate::math::{wrap_angle_0_to_2pi, PI};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SixStepOutput {
    pub phase_a: i32,
    pub phase_b: i32,
    pub phase_c: i32,
    pub sector: i32,
}

pub fn six_step_from_sector(sector: i32) -> SixStepOutput {
    let normalized = (sector - 1).rem_euclid(6) + 1;
    let (phase_a, phase_b, phase_c) = match normalized {
        1 => (1, -1, 0),
        2 => (1, 0, -1),
        3 => (0, 1, -1),
        4 => (-1, 1, 0),
        5 => (-1, 0, 1),
        6 => (0, -1, 1),
        _ => (0, 0, 0),
    };
    SixStepOutput {
        phase_a,
        phase_b,
        phase_c,
        sector: normalized,
    }
}

pub fn six_step_from_angle(theta_rad: f32) -> SixStepOutput {
    let wrapped = wrap_angle_0_to_2pi(theta_rad);
    let sector = ((wrapped / (PI / 3.0)) as i32 + 1).min(6);
    six_step_from_sector(sector)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_c_reference_vectors() {
        assert_eq!(
            six_step_from_sector(1),
            SixStepOutput {
                phase_a: 1,
                phase_b: -1,
                phase_c: 0,
                sector: 1
            }
        );
        assert_eq!(six_step_from_sector(7).sector, 1);
        let out = six_step_from_angle(PI / 3.0 + 0.01);
        assert_eq!(out.sector, 2);
        assert_eq!(out.phase_c, -1);
    }
}
