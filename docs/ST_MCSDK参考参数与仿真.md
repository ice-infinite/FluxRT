# ST MCSDK 参考参数与 PC 闭环仿真

## 1. 参考工程和本工程的关系

只读参考工程：

```text
C:\Users\y2389_4rq4ld9\.st_workbench\Projects2\P-IHM03-Potentiometer
```

当前实现工程：

```text
E:\File\RT-Thread\projects\FluxRT
```

参考工程由 ST Motor Control Workbench / MCSDK 6.4.1 生成，硬件组合为
`NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T`。本次没有修改或复制
ST 的生成代码；只读取生成参数与控制流程，在独立 Rust 代码中建立可测试的
浮点实现。

这不是 MCSDK 的逐指令或逐定点位等价移植。当前目标是保持控制拓扑、时序、
限幅和物理参数一致，方便先在 PC 上验证架构，再在板上重新辨识与整定。

## 2. 参数基线

| 项目 | 参考值 | 来源 |
|---|---:|---|
| 极对数 | 7 | `pmsm_motor_parameters.h` / `.wbdef` |
| 相电阻 | 5.29 Ω | `pmsm_motor_parameters.h` |
| Ld / Lq | 1.058 mH / 1.058 mH | `pmsm_motor_parameters.h` |
| 线电压常数 | 5.0 V RMS/kRPM | `.wbdef` |
| 额定电流 | 0.8 A | `pmsm_motor_parameters.h` |
| 最大转速 | 1572 rpm | `pmsm_motor_parameters.h` |
| 母线标称电压 | 13 V | `power_stage_parameters.h` |
| ADC 参考电压 | 3.3 V | `.ioc` / `parameters_conversion.h` |
| 母线分压 | 0.0625（1/16） | `.ioc` / `power_stage_parameters.h` |
| 相端诊断分压 | 10 kΩ / 2.2 kΩ，理论满量程 18.3 V | IHM16M1 官方原理图；仅名义值 |
| PWM / 电流环 | 30 kHz | `drive_parameters.h` |
| 速度环 | 1 kHz | `drive_parameters.h` |
| 默认目标转速 | 524 rpm | `drive_parameters.h` |
| 过压 / 欠压 | 15 V / 7 V | `drive_parameters.h` |
| 过温 / 回差 | 110 °C / 10 °C | `drive_parameters.h` |

生成的 Id/Iq PI 定点参数为 `Kp=3378/1024`、每次调用的
`Ki=2252/4096`；速度 PI 为 `Kp=2730/256`、每次调用的
`Ki=562/16384`。`foc-control/src/params.rs` 使用参考工程的分流电阻、
放大倍数、电流 ADC 换算和归一化电压范围，把这些定点比例换算成 SI 浮点增益。
因此不能只把 `3378` 当成浮点 Kp。

表中的 30 kHz 是 ST 参考值。当前工程实机和 PC plant 已改为 12 kHz；代码先按
参考 30 kHz 恢复连续时间 Ki，再使用 `ts=1/12000 s` 离散执行，从而避免简单替换
频率后把积分带宽按 16/30 缩小。

Workbench 中的机械原始值为 `INERTIA=0.291`、`FRICTION=0.937`，但生成工程
没有把单位和辨识可信度带入控制源码。当前 PMSM plant 使用的 SI 值只是仿真起点，
已在参数代码中明确标注；换真实电机或带载后必须重新辨识，不能当成实机证明。

## 3. 对齐的控制流程

参考工程高频路径：

```text
三相电流
  → Clarke / Park
  → Id、Iq PI
  → 电压圆限幅（MaxVd=95%）
  → 逆 Park
  → SVPWM
```

参考工程中频路径：

```text
目标机械转速 - 反馈机械转速
  → 速度 PI（1 kHz）
  → Iq 给定
  → Id 给定为 0
```

Rust 对应实现位于：

- `rust/crates/foc-control/src/controller.rs`：电流环、速度环、圆限幅和级联控制。
- `rust/crates/foc-control/src/params.rs`：ST 参考参数及定点到 SI 的换算。
- `rust/crates/foc-control/src/startup.rs`：1.000 s 定向、1.164 s 升到
  582 rpm、25 ms 观测器过渡。
- `rust/crates/foc-control/src/observer.rs`：可替换的 BEMF + PLL 适配器。
- `rust/crates/foc-control/src/voltage.rs`：观察器电压来源；当前只批准 `CommandModel`。
- `rust/crates/foc-rt-bridge/src/lib.rs`：同一控制代码对 C 的稳定 ABI。

`foc_rust_configure_st_reference()` 一次性装入当前基线。C 侧仍然保留最终硬件
使能权：即使 Rust 参数正确，只要 `foc_platform_init()` 没有返回 OK，启动仍会被拒绝。

### 3.1 ST 观察器实际使用的电压

ST 参考工程的高频任务先把上一拍 `FOCVars[M1].Valphabeta` 放入 `STO_Inputs`，电流控制器
随后计算本拍 `Valphabeta` 并保存；STO 同时读取平均 Vbus。也就是说，默认 STO-PLL 使用
命令电压模型与母线电压，不读取 TP6/TP7/TP8 的相端 ADC。

FluxRT 因此把默认路径明确命名为 `CommandModel`：上一拍三相 duty 先去掉 SVPWM 共模，
再乘实测 Vbus 得到 αβ 电压。IHM16M1 的 10 kΩ/2.2 kΩ、3.3 V、12 bit 只形成
`18.3/4095 = 4.468864 mV/count` 的名义诊断换算；没有逐板测量时必须标记为
`nominal-not-calibrated` 和 `observer=disabled`。

## 4. 硬件接口如何解耦

全部控制器外部能力集中在 `rust/crates/foc-control/src/ports.rs`：

```rust
pub trait FeedbackPort {
    fn read_feedback(&mut self) -> Result<FeedbackSnapshot, HardwareFault>;
}

pub trait PwmPort {
    fn apply_pwm(&mut self, command: PwmCommand) -> Result<(), HardwareFault>;
    fn disable_pwm(&mut self);
}

pub trait SafetyPort {
    fn power_stage_faulted(&self) -> bool;
}
```

这里把同一 PWM 周期的电流、母线和转子反馈做成一个快照，避免分别读取造成跨周期
数据混用。`ControlRuntime<H>` 只依赖这些 trait，不认识 STM32、RT-Thread 或寄存器。

- PC：`SimHardware` 实现三个接口。
- STM32G431：C ISR/HAL 通过 `foc_rust_bridge.h` 提供快照并落地 PWM。
- 后续国产 MCU：重写 C 平台适配层，不改控制器和 plant 测试。
- 单元测试：可以只替换某一个 fake，注入欠压、ADC 异常、输出拒绝或功率级故障。

Rust 内部需要模拟的硬件能力已经公开为 trait；芯片寄存器接口没有泄漏进算法。
跨 C/Rust 边界继续使用定宽 C ABI，而不是 trait object。

## 5. PC 闭环包含什么

`rust/crates/foc-sim` 不是预先录制的数据回放，它实际运行以下回路：

```text
524 rpm 指令
  → 1 kHz 速度 PI
  → 12 kHz Id/Iq PI + 圆限幅 + SVPWM
  → 三相平均值逆变器
  → dq PMSM 电气方程 + 机械方程
  → 三相电流、真实电角度和机械速度假传感器
  └───────────────────────────────反馈
```

自动场景包括：

1. 0 → 524 rpm 速度阶跃并收敛；
2. 加入 0.004 N·m 负载扰动后恢复；
3. 注入功率级故障后立即禁用 PWM；
4. BEMF + PLL 对非零速旋转电势的跟踪；
5. ST 启动阶段和时长切换。

运行全部验证：

```powershell
cd E:\File\RT-Thread\projects\FluxRT
.\test.ps1
```

只看仿真：

```powershell
cd E:\File\RT-Thread\projects\FluxRT\rust
cargo run -p foc-sim --locked
```

程序打印 `FOC_SIM_PASS`、最终转速、目标转速和峰值相电流。真正的验收条件在
`foc-sim/src/lib.rs` 的断言里，不能靠打印一行 PASS 绕过。

## 6. 为什么闭环仿真先使用理想角度

当前 plant 主闭环使用 `SimHardware` 的理想转子角度。这样可以先独立验证速度环、
电流环、调制、plant、负载扰动和安全路径，不把 STO-PLL 参数误差混入所有问题。

参考工程实机采用 STO-PLL，低速时必须先开环升速到观测器可靠区。本工程已经有：

- 与参考时长一致的 `RevUpSequencer`；
- 可替换的 `RotorEstimator` 接口；
- 浮点 `BemfPllEstimator` 及非零速跟踪测试。

尚未声称完成的是“从静止开始，全程仅用估算角度并稳定切入闭环”的联合仿真，以及
与 ST 固定点 STO-PLL 的逐采样对拍。这两项应在拿到实板电流极性、ADC 同步点、相序、
死区和电机辨识结果后继续整定。现有理想角度闭环是软件架构和控制 plant 的证据，
不是无感启动或实机可转的证据。

## 7. 接实板时的顺序

1. 完成 `foc/platform/stm32g431/` 的 TIM1、ADC1/2、DMA、COMP/Break 和 STSPIN830
   使能脚；先验证硬件关断链。
2. 核对 U/V/W 相序、电流正负号、ADC 中点与电角度方向。
3. 低压限流，仅运行电流采样和 PWM 同步，不先闭环转动。
4. 对比 C 侧反馈快照和 Rust telemetry，验证 Clarke/Park 后的 Id/Iq。
5. 再接 `RevUpSequencer + BemfPllEstimator`，检查 524 rpm 最低可靠速度附近的收敛。
6. 最后才整定速度环、负载扰动和保护阈值。

每一步都要把主机测试、目标交叉编译、烧录、示波器波形、实机转动和保护动作分开记录。
