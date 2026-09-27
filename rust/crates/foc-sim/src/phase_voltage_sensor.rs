//! Deterministic PC-only three-phase voltage sensor model for A24.
//!
//! The model uses a fixed delay line and allocates nothing per sample.  Its
//! defaults are transparent, while offset, gain, ADC quantisation, delay and
//! explicit fault injection can be enabled independently.  Thresholds belong
//! to the simulation fixture and are not production calibration.

const PHASE_COUNT: usize = 3;
pub const MAX_PHASE_VOLTAGE_DELAY_TICKS: usize = 8;
const DELAY_CAPACITY: usize = MAX_PHASE_VOLTAGE_DELAY_TICKS + 1;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Phase {
    U = 0,
    V = 1,
    W = 2,
}

impl Phase {
    const fn index(self) -> usize {
        self as usize
    }

    const fn bit(self) -> u8 {
        1_u8 << self.index()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaseVoltageSensorConfig {
    pub adc_max_code: u16,
    pub full_scale_v: f32,
    pub offset_v: [f32; PHASE_COUNT],
    pub gain: [f32; PHASE_COUNT],
    pub effective_bits: u8,
    pub delay_ticks: u8,
    pub low_rail_max_code: u16,
    pub high_rail_min_code: u16,
}

impl Default for PhaseVoltageSensorConfig {
    fn default() -> Self {
        Self {
            adc_max_code: 4095,
            full_scale_v: 18.3,
            offset_v: [0.0; PHASE_COUNT],
            gain: [1.0; PHASE_COUNT],
            effective_bits: 12,
            delay_ticks: 0,
            low_rail_max_code: 8,
            high_rail_min_code: 4087,
        }
    }
}

impl PhaseVoltageSensorConfig {
    pub fn is_valid(self) -> bool {
        let adc_bits = 16_u32 - self.adc_max_code.leading_zeros();
        self.adc_max_code > 0
            && self.full_scale_v.is_finite()
            && self.full_scale_v > 0.0
            && self.offset_v.iter().all(|value| value.is_finite())
            && self
                .gain
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
            && self.effective_bits > 0
            && u32::from(self.effective_bits) <= adc_bits
            && usize::from(self.delay_ticks) <= MAX_PHASE_VOLTAGE_DELAY_TICKS
            && self.low_rail_max_code < self.high_rail_min_code
            && self.high_rail_min_code <= self.adc_max_code
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PhaseVoltageFault {
    None,
    Dropout,
    ForceStale { age_ticks: u32 },
    LowSaturation { phase: Phase },
    HighSaturation { phase: Phase },
    OpenStuck { phase: Phase },
    PhaseOffset { phase: Phase, offset_v: f32 },
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct EncodedFrame {
    sequence: u32,
    raw: [u16; PHASE_COUNT],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaseVoltageSensorSample {
    pub sequence: u32,
    pub age_ticks: u32,
    pub available: bool,
    pub configuration_valid: bool,
    pub raw: [u16; PHASE_COUNT],
    pub volts: [f32; PHASE_COUNT],
    pub low_rail_phase_mask: u8,
    pub high_rail_phase_mask: u8,
}

impl Default for PhaseVoltageSensorSample {
    fn default() -> Self {
        Self {
            sequence: 0,
            age_ticks: 0,
            available: false,
            configuration_valid: false,
            raw: [0; PHASE_COUNT],
            volts: [0.0; PHASE_COUNT],
            low_rail_phase_mask: 0,
            high_rail_phase_mask: 0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PhaseVoltageSensor {
    config: PhaseVoltageSensorConfig,
    frames: [EncodedFrame; DELAY_CAPACITY],
    write_index: usize,
    frame_count: usize,
    next_sequence: u32,
    stuck_raw: [u16; PHASE_COUNT],
    stuck_valid_mask: u8,
}

impl PhaseVoltageSensor {
    pub fn try_new(config: PhaseVoltageSensorConfig) -> Option<Self> {
        if !config.is_valid() {
            return None;
        }
        Some(Self {
            config,
            frames: [EncodedFrame::default(); DELAY_CAPACITY],
            write_index: 0,
            frame_count: 0,
            next_sequence: 0,
            stuck_raw: [0; PHASE_COUNT],
            stuck_valid_mask: 0,
        })
    }

    pub const fn config(&self) -> PhaseVoltageSensorConfig {
        self.config
    }

    pub fn reset(&mut self) {
        self.frames = [EncodedFrame::default(); DELAY_CAPACITY];
        self.write_index = 0;
        self.frame_count = 0;
        self.next_sequence = 0;
        self.stuck_raw = [0; PHASE_COUNT];
        self.stuck_valid_mask = 0;
    }

    pub fn sample(
        &mut self,
        true_phase_voltage_v: [f32; PHASE_COUNT],
        fault: PhaseVoltageFault,
    ) -> PhaseVoltageSensorSample {
        if !true_phase_voltage_v.iter().all(|value| value.is_finite()) {
            // A rejected conversion still consumed one trigger. Advance the
            // sequence so recovery cannot reuse the failed tick's id and break
            // the V19 `control_sequence - sample_sequence == age` relation.
            let sequence = self.next_sequence;
            self.next_sequence = self.next_sequence.wrapping_add(1);
            return PhaseVoltageSensorSample {
                sequence,
                configuration_valid: true,
                ..PhaseVoltageSensorSample::default()
            };
        }

        let mut raw = [0_u16; PHASE_COUNT];
        for (phase, raw_code) in raw.iter_mut().enumerate() {
            let measured =
                true_phase_voltage_v[phase] * self.config.gain[phase] + self.config.offset_v[phase];
            *raw_code = self.encode(measured);
        }

        match fault {
            PhaseVoltageFault::LowSaturation { phase } => raw[phase.index()] = 0,
            PhaseVoltageFault::HighSaturation { phase } => {
                raw[phase.index()] = self.config.adc_max_code;
            }
            PhaseVoltageFault::OpenStuck { phase } => {
                let index = phase.index();
                if self.stuck_valid_mask & phase.bit() == 0 {
                    self.stuck_raw[index] = self.latest_raw(index).unwrap_or(raw[index]);
                    self.stuck_valid_mask |= phase.bit();
                }
                raw[index] = self.stuck_raw[index];
            }
            PhaseVoltageFault::PhaseOffset { phase, offset_v } if offset_v.is_finite() => {
                raw[phase.index()] = self.encode(self.decode(raw[phase.index()]) + offset_v);
            }
            _ => {}
        }
        if !matches!(fault, PhaseVoltageFault::OpenStuck { .. }) {
            self.stuck_valid_mask = 0;
        }

        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        self.frames[self.write_index] = EncodedFrame { sequence, raw };
        self.write_index = (self.write_index + 1) % DELAY_CAPACITY;
        self.frame_count = (self.frame_count + 1).min(DELAY_CAPACITY);

        let delay = usize::from(self.config.delay_ticks);
        if self.frame_count <= delay {
            return PhaseVoltageSensorSample {
                sequence,
                age_ticks: self.config.delay_ticks.into(),
                configuration_valid: true,
                ..PhaseVoltageSensorSample::default()
            };
        }
        let read_index = (self.write_index + DELAY_CAPACITY - delay - 1) % DELAY_CAPACITY;
        let output = self.frames[read_index];
        let forced_age = match fault {
            PhaseVoltageFault::ForceStale { age_ticks } => age_ticks,
            _ => u32::from(self.config.delay_ticks),
        };
        let available = !matches!(fault, PhaseVoltageFault::Dropout);
        self.build_sample(output, forced_age, available)
    }

    fn encode(&self, volts: f32) -> u16 {
        let clipped = volts.clamp(0.0, self.config.full_scale_v);
        let ideal = (clipped * f32::from(self.config.adc_max_code) / self.config.full_scale_v)
            .round() as u32;
        let adc_bits = 16_u32 - self.config.adc_max_code.leading_zeros();
        let dropped_bits = adc_bits - u32::from(self.config.effective_bits);
        let quantum = 1_u32 << dropped_bits;
        let quantized = ((ideal + quantum / 2) / quantum) * quantum;
        quantized.min(u32::from(self.config.adc_max_code)) as u16
    }

    fn decode(&self, raw: u16) -> f32 {
        f32::from(raw) * self.config.full_scale_v / f32::from(self.config.adc_max_code)
    }

    fn latest_raw(&self, phase: usize) -> Option<u16> {
        if self.frame_count == 0 {
            return None;
        }
        let latest = (self.write_index + DELAY_CAPACITY - 1) % DELAY_CAPACITY;
        Some(self.frames[latest].raw[phase])
    }

    fn build_sample(
        &self,
        frame: EncodedFrame,
        age_ticks: u32,
        available: bool,
    ) -> PhaseVoltageSensorSample {
        let mut volts = [0.0_f32; PHASE_COUNT];
        let mut low_mask = 0_u8;
        let mut high_mask = 0_u8;
        for (phase, (&raw_code, volts_out)) in frame.raw.iter().zip(volts.iter_mut()).enumerate() {
            *volts_out = self.decode(raw_code);
            if raw_code <= self.config.low_rail_max_code {
                low_mask |= 1_u8 << phase;
            }
            if raw_code >= self.config.high_rail_min_code {
                high_mask |= 1_u8 << phase;
            }
        }
        PhaseVoltageSensorSample {
            sequence: frame.sequence,
            age_ticks,
            available,
            configuration_valid: true,
            raw: frame.raw,
            volts,
            low_rail_phase_mask: low_mask,
            high_rail_phase_mask: high_mask,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(sensor: &mut PhaseVoltageSensor, values: [f32; 3]) -> PhaseVoltageSensorSample {
        sensor.sample(values, PhaseVoltageFault::None)
    }

    #[test]
    fn baseline_and_common_mode_offset_are_deterministic() {
        let mut base = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig::default()).unwrap();
        let mut offset = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig {
            offset_v: [0.08; 3],
            ..PhaseVoltageSensorConfig::default()
        })
        .unwrap();
        let truth = [4.1, 6.2, 8.3];
        let a = sample(&mut base, truth);
        let b = sample(&mut base, truth);
        let shifted = sample(&mut offset, truth);
        assert_eq!(a.raw, b.raw);
        let base_lines = [a.volts[0] - a.volts[1], a.volts[1] - a.volts[2]];
        let shifted_lines = [
            shifted.volts[0] - shifted.volts[1],
            shifted.volts[1] - shifted.volts[2],
        ];
        assert!((base_lines[0] - shifted_lines[0]).abs() < 0.01);
        assert!((base_lines[1] - shifted_lines[1]).abs() < 0.01);
    }

    #[test]
    fn gain_quantisation_and_delay_are_explicit() {
        let mut sensor = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig {
            gain: [1.01; 3],
            effective_bits: 9,
            delay_ticks: 2,
            ..PhaseVoltageSensorConfig::default()
        })
        .unwrap();
        assert!(!sample(&mut sensor, [4.0, 5.0, 6.0]).available);
        assert!(!sample(&mut sensor, [5.0, 6.0, 7.0]).available);
        let delayed = sample(&mut sensor, [6.0, 7.0, 8.0]);
        assert!(delayed.available);
        assert_eq!(delayed.sequence, 0);
        assert_eq!(delayed.age_ticks, 2);
        assert!(delayed.volts[0] > 4.0);
        assert_eq!(u32::from(delayed.raw[0]) % 8, 0);
    }

    #[test]
    fn dropout_and_forced_stale_keep_explicit_age_and_availability() {
        let mut sensor = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig::default()).unwrap();
        let dropped = sensor.sample([4.0, 5.0, 6.0], PhaseVoltageFault::Dropout);
        assert!(!dropped.available);
        let stale = sensor.sample(
            [4.1, 5.1, 6.1],
            PhaseVoltageFault::ForceStale { age_ticks: 9 },
        );
        assert!(stale.available);
        assert_eq!(stale.age_ticks, 9);
    }

    #[test]
    fn rail_faults_expose_phase_masks() {
        let mut sensor = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig::default()).unwrap();
        let low = sensor.sample(
            [4.0, 5.0, 6.0],
            PhaseVoltageFault::LowSaturation { phase: Phase::U },
        );
        assert_eq!(low.low_rail_phase_mask, Phase::U.bit());
        let high = sensor.sample(
            [4.0, 5.0, 6.0],
            PhaseVoltageFault::HighSaturation { phase: Phase::V },
        );
        assert_eq!(high.high_rail_phase_mask, Phase::V.bit());
    }

    #[test]
    fn open_stuck_and_single_phase_offset_are_observable() {
        let mut sensor = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig::default()).unwrap();
        let first = sample(&mut sensor, [4.0, 5.0, 6.0]);
        let stuck = sensor.sample(
            [7.0, 8.0, 9.0],
            PhaseVoltageFault::OpenStuck { phase: Phase::W },
        );
        assert_eq!(stuck.raw[2], first.raw[2]);
        let inconsistent = sensor.sample(
            [7.0, 8.0, 9.0],
            PhaseVoltageFault::PhaseOffset {
                phase: Phase::V,
                offset_v: 0.8,
            },
        );
        assert!(inconsistent.volts[1] > 8.7);
    }

    #[test]
    fn invalid_configuration_and_non_finite_truth_fail_closed() {
        assert!(PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig {
            delay_ticks: (MAX_PHASE_VOLTAGE_DELAY_TICKS + 1) as u8,
            ..PhaseVoltageSensorConfig::default()
        })
        .is_none());
        let mut sensor = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig::default()).unwrap();
        let bad = sample(&mut sensor, [f32::NAN, 1.0, 2.0]);
        assert!(!bad.available);
        assert!(bad.configuration_valid);
        assert_eq!(bad.sequence, 0);
        let recovered = sample(&mut sensor, [4.0, 5.0, 6.0]);
        assert!(recovered.available);
        assert_eq!(recovered.sequence, 1);
    }

    #[test]
    fn rejected_non_finite_tick_preserves_wrapping_sequence_progress() {
        let mut sensor = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig::default()).unwrap();
        sensor.next_sequence = u32::MAX;
        let bad = sample(&mut sensor, [4.0, f32::INFINITY, 6.0]);
        assert_eq!(bad.sequence, u32::MAX);
        assert!(!bad.available);
        let recovered = sample(&mut sensor, [4.0, 5.0, 6.0]);
        assert_eq!(recovered.sequence, 0);
        assert!(recovered.available);
    }
}
