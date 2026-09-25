//! 离线仿真的整数多速率调度器。
//!
//! 这里只描述“载波边界、控制拍、占空保持与命令生效延迟”，不属于目标固件
//! ABI，也不负责 STM32 定时器配置。目标侧将来必须由平台层提供同样的整数比保证。

use foc_control::PwmCommand;

/// PC 仿真的载波/控制时间计划。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MultiRateTimingConfig {
    /// PWM 载波频率 `[Hz]`，同时也是被控对象推进频率。
    pub pwm_frequency_hz: u32,
    /// 完整控制链执行频率 `[Hz]`。
    pub control_frequency_hz: u32,
    /// 控制输出到物理占空生效之间的整 PWM 拍延迟。
    pub actuation_delay_pwm_ticks: u32,
}

impl Default for MultiRateTimingConfig {
    fn default() -> Self {
        Self {
            pwm_frequency_hz: 12_000,
            control_frequency_hz: 12_000,
            actuation_delay_pwm_ticks: 1,
        }
    }
}

/// 非法多速率配置。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimingConfigError {
    ZeroFrequency,
    NonIntegerRatio,
    DelayExceedsControlPeriod,
}

impl MultiRateTimingConfig {
    /// 校验整数分频与单待处理命令约束，并返回每控制拍包含的 PWM 拍数。
    pub fn validate(self) -> Result<u32, TimingConfigError> {
        if self.pwm_frequency_hz == 0 || self.control_frequency_hz == 0 {
            return Err(TimingConfigError::ZeroFrequency);
        }
        if !self
            .pwm_frequency_hz
            .is_multiple_of(self.control_frequency_hz)
        {
            return Err(TimingConfigError::NonIntegerRatio);
        }
        let pwm_ticks_per_control = self.pwm_frequency_hz / self.control_frequency_hz;
        if self.actuation_delay_pwm_ticks > pwm_ticks_per_control {
            return Err(TimingConfigError::DelayExceedsControlPeriod);
        }
        Ok(pwm_ticks_per_control)
    }

    pub fn pwm_period_s(self) -> f32 {
        1.0 / self.pwm_frequency_hz as f32
    }

    pub fn control_period_s(self) -> f32 {
        1.0 / self.control_frequency_hz as f32
    }
}

#[derive(Clone, Copy, Debug)]
struct PendingPwm {
    apply_at_pwm_tick: u64,
    command: PwmCommand,
}

/// 固定整数比的离线调度器：控制只在分频点执行，PWM 命令在其余载波周期保持。
#[derive(Clone, Copy, Debug)]
pub struct MultiRateScheduler {
    config: MultiRateTimingConfig,
    pwm_ticks_per_control: u32,
    pwm_tick: u64,
    control_ticks: u64,
    applied_updates: u64,
    active_pwm: PwmCommand,
    pending_pwm: Option<PendingPwm>,
}

impl MultiRateScheduler {
    pub fn new(config: MultiRateTimingConfig) -> Result<Self, TimingConfigError> {
        let pwm_ticks_per_control = config.validate()?;
        Ok(Self {
            config,
            pwm_ticks_per_control,
            pwm_tick: 0,
            control_ticks: 0,
            applied_updates: 0,
            active_pwm: PwmCommand::default(),
            pending_pwm: None,
        })
    }

    /// 进入一个 PWM 周期：先让到期命令生效，再判断本周期是否执行控制链。
    pub fn begin_pwm_tick(&mut self) -> bool {
        if let Some(pending) = self.pending_pwm {
            if pending.apply_at_pwm_tick == self.pwm_tick {
                self.active_pwm = pending.command;
                self.pending_pwm = None;
                self.applied_updates += 1;
            }
        }
        self.control_due()
    }

    /// 提交本控制拍的新命令。零延迟立即生效，非零延迟在未来载波边界生效。
    pub fn submit_control_output(&mut self, command: PwmCommand) {
        debug_assert!(self.control_due());
        debug_assert!(self.pending_pwm.is_none());
        self.control_ticks += 1;
        if self.config.actuation_delay_pwm_ticks == 0 {
            self.active_pwm = command;
            self.applied_updates += 1;
        } else {
            self.pending_pwm = Some(PendingPwm {
                apply_at_pwm_tick: self.pwm_tick + u64::from(self.config.actuation_delay_pwm_ticks),
                command,
            });
        }
    }

    pub fn finish_pwm_tick(&mut self) {
        self.pwm_tick += 1;
    }

    pub fn control_due(&self) -> bool {
        self.pwm_tick
            .is_multiple_of(u64::from(self.pwm_ticks_per_control))
    }

    pub fn active_pwm(&self) -> PwmCommand {
        self.active_pwm
    }

    pub fn pwm_tick_count(&self) -> u64 {
        self.pwm_tick
    }

    pub fn control_tick_count(&self) -> u64 {
        self.control_ticks
    }

    pub fn applied_update_count(&self) -> u64 {
        self.applied_updates
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn command(value: f32) -> PwmCommand {
        PwmCommand {
            duty_a: value,
            duty_b: value,
            duty_c: value,
        }
    }

    #[test]
    fn rejects_non_integer_or_overlapping_plans() {
        assert_eq!(
            MultiRateTimingConfig {
                pwm_frequency_hz: 24_000,
                control_frequency_hz: 10_000,
                actuation_delay_pwm_ticks: 0,
            }
            .validate(),
            Err(TimingConfigError::NonIntegerRatio)
        );
        assert_eq!(
            MultiRateTimingConfig {
                pwm_frequency_hz: 24_000,
                control_frequency_hz: 12_000,
                actuation_delay_pwm_ticks: 3,
            }
            .validate(),
            Err(TimingConfigError::DelayExceedsControlPeriod)
        );
    }

    #[test]
    fn holds_each_command_for_two_pwm_ticks_with_one_control_tick_delay() {
        let mut scheduler = MultiRateScheduler::new(MultiRateTimingConfig {
            pwm_frequency_hz: 24_000,
            control_frequency_hz: 12_000,
            actuation_delay_pwm_ticks: 2,
        })
        .unwrap();

        let mut active = Vec::new();
        for tick in 0..6 {
            let due = scheduler.begin_pwm_tick();
            if due {
                scheduler.submit_control_output(command(0.6 + tick as f32 * 0.01));
            }
            active.push(scheduler.active_pwm().duty_a);
            scheduler.finish_pwm_tick();
        }

        let expected = [0.5, 0.5, 0.6, 0.6, 0.62, 0.62];
        assert!(active
            .iter()
            .zip(expected)
            .all(|(actual, expected)| (actual - expected).abs() < 1.0e-6));
        assert_eq!(scheduler.pwm_tick_count(), 6);
        assert_eq!(scheduler.control_tick_count(), 3);
        assert_eq!(scheduler.applied_update_count(), 2);
    }

    #[test]
    fn target_single_rate_applies_at_the_next_pwm_boundary() {
        let mut scheduler = MultiRateScheduler::new(MultiRateTimingConfig::default()).unwrap();
        assert!(scheduler.begin_pwm_tick());
        scheduler.submit_control_output(command(0.61));
        assert_eq!(scheduler.active_pwm().duty_a, 0.5);
        scheduler.finish_pwm_tick();
        assert!(scheduler.begin_pwm_tick());
        assert_eq!(scheduler.active_pwm().duty_a, 0.61);
    }
}
