//! Transform_ClarkePark 的等价实现。

use crate::math::{INV_SQRT_3, SQRT_3};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Abc {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AlphaBeta {
    pub alpha: f32,
    pub beta: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Dq {
    pub d: f32,
    pub q: f32,
}

#[inline]
pub fn clarke(input: Abc) -> AlphaBeta {
    AlphaBeta {
        alpha: input.a,
        beta: (input.a + 2.0 * input.b) * INV_SQRT_3,
    }
}

#[inline]
pub fn clarke_two_phase(ia: f32, ib: f32) -> AlphaBeta {
    clarke(Abc {
        a: ia,
        b: ib,
        c: -(ia + ib),
    })
}

#[inline]
pub fn inverse_clarke(input: AlphaBeta) -> Abc {
    Abc {
        a: input.alpha,
        b: -0.5 * input.alpha + 0.5 * SQRT_3 * input.beta,
        c: -0.5 * input.alpha - 0.5 * SQRT_3 * input.beta,
    }
}

#[inline]
pub fn park(input: AlphaBeta, theta_rad: f32) -> Dq {
    let s = libm::sinf(theta_rad);
    let c = libm::cosf(theta_rad);
    Dq {
        d: input.alpha * c + input.beta * s,
        q: -input.alpha * s + input.beta * c,
    }
}

#[inline]
pub fn inverse_park(input: Dq, theta_rad: f32) -> AlphaBeta {
    let s = libm::sinf(theta_rad);
    let c = libm::cosf(theta_rad);
    AlphaBeta {
        alpha: input.d * c - input.q * s,
        beta: input.d * s + input.q * c,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::PI;

    fn assert_near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn matches_c_reference_vectors() {
        let abc = Abc {
            a: 1.0,
            b: -0.5,
            c: -0.5,
        };
        let ab = clarke(abc);
        assert_near(ab.alpha, 1.0, 1e-6);
        assert_near(ab.beta, 0.0, 1e-6);

        let abc_back = inverse_clarke(ab);
        assert_near(abc_back.a, abc.a, 1e-6);
        assert_near(abc_back.b, abc.b, 1e-6);
        assert_near(abc_back.c, abc.c, 1e-6);

        let dq = park(ab, PI * 0.5);
        assert_near(dq.d, 0.0, 1e-5);
        assert_near(dq.q, -1.0, 1e-5);
        let ab_back = inverse_park(dq, PI * 0.5);
        assert_near(ab_back.alpha, ab.alpha, 1e-5);
        assert_near(ab_back.beta, ab.beta, 1e-5);
    }
}
