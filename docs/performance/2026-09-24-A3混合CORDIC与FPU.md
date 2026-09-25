# 阶段 1.3：混合 CORDIC/FPU 模长优化与实机 WCET

## 1. 结论

本批只优化快环的矢量模长计算，不改 12 kHz 时基、控制参数、观察器、默认开环和保护阈值。

- `sin/cos`、`atan2` 继续使用 STM32G431 CORDIC；
- `magnitude(x,y)` 改为 FPU 平方和加 `VSQRT.F32`；
- Diagnostic + Rust `s` 的 BIN 从 122,788 B 降至 122,516 B，减少 272 B；
- 三次 582 rpm、开环、trace 关闭、5 秒实测的最坏完整 ISR 从 8,267 降至
  7,957 cycles，减少 310 cycles（3.75%）；
- 三次均为 0 invalid、0 deadline miss、0 error、0 fault。

这是阶段 1.3 的第一项通过，不代表整个数学后端已完成。`sin/cos`、`atan2` 的分项误差、
平均周期和最坏周期仍待测量。

## 2. 为什么不再用 CORDIC 求模长

原实现为 CORDIC modulus 准备 Q15 输入，需要缩放、饱和、打包、保存/恢复 `PRIMASK`、
写 CORDIC 寄存器、轮询和反缩放。STM32G431 的 Cortex-M4F 已有单精度硬件开方；当前
输入本来就是 `f32`，因此直接计算：

```text
magnitude_squared = x*x + y*y
magnitude = VSQRT.F32(magnitude_squared)
```

更短，也不占用共享 CORDIC 和中断临界区。非有限输入或平方和溢出时 C 适配器返回 0，
Rust bridge 仍在同一拍用 `CpuMath` 重算。

目标 ELF 的反汇编确认 `foc_math_accel_magnitude` 包含：

```text
vmul.f32
vfma.f32
vsqrt.f32
```

该函数不再包含 CORDIC modulus 寄存器事务。`VSQRT.F32` 用 GNU Arm 内联汇编明确发出；
以后换编译器或 MCU 时必须重新验证，不能把这段实现直接复制到没有硬件单精度开方的目标。

## 3. 被否决并回退的候选

先测试过“当 dq 电压平方和明确低于圆限幅 98% 边界时跳过 magnitude”的候选。Host
增加了目标用例和 1,089 点 dq 网格等价测试，完整测试当时通过，但实机 WCET 反而变差：

| 运行 | 完整 ISR | control | invalid/miss/error/fault |
|---|---:|---:|---|
| 1 | 8,417 | 7,559 | 0/0/0/0 |
| 2 | 8,429 | 7,571 | 0/0/0/0 |
| 3 | 8,429 | 7,571 | 0/0/0/0 |

最坏值比 8,267 cycles 基线增加 162 cycles，因此该候选全部回退。回退后曾重新构建并
核对基线 BIN SHA-256 仍为
`DD91427930F050A584F85E2BD75DFE960B9F12D3E2436CFBC27189CFBDFE6B6B`，随后才实现
FPU 模长。结论是：不能仅凭减少一次数学调用推断 WCET，分支、比较和 LTO 结果必须上板测。

原始记录位于被 `.gitignore` 排除的本地结果目录：

- `simulation/results/timing_diagnostic_s_magnitude_fastpath_run1_openloop_582rpm_12v_20260923.json`
- `simulation/results/timing_diagnostic_s_magnitude_fastpath_run2_openloop_582rpm_12v_20260923.json`
- `simulation/results/timing_diagnostic_s_magnitude_fastpath_run3_openloop_582rpm_12v_20260923.json`

## 4. 构建与静态证据

### Diagnostic + Rust `s`

| 项目 | 结果 |
|---|---:|
| text | 120,752 B |
| data | 1,764 B |
| bss | 9,964 B |
| Flash/BIN | 122,516 B |
| RAM | 11,728 B |

| 文件 | SHA-256 |
|---|---|
| `cmake-build/fluxrt.bin` | `DE037DB47EA3469A85624086AEDE47DE9D5E66CE108C7BBBCA518C708067FFF5` |
| `cmake-build/fluxrt.hex` | `69A01B40A137006D80F66803D4F2D8DD04EF8303C81FABF3B5E6682A071AEBB4` |
| `cmake-build/rtthread.elf` | `FB940CC7E300C6C6C18BB337A9807D40AF9F5C8296C5C6EB03255FFBF11377C2` |

### Production + Rust `s`

| 项目 | 结果 |
|---|---:|
| text | 91,568 B |
| data | 1,148 B |
| bss | 6,852 B |
| Flash/BIN | 92,716 B |
| RAM | 8,000 B |

| 文件 | SHA-256 |
|---|---|
| `cmake-build-production/fluxrt.bin` | `03B551C9D4C423CA1C6DCAC99F8CC39CC5CF880316230A0DFCBFBDACD95B4057` |
| `cmake-build-production/fluxrt.hex` | `57B1FEDE954DFD00335881BEBF9BB8580316F70D4566B1660D901F0B41900485` |
| `cmake-build-production/rtthread.elf` | `39652F2212F87FB7C987FEB40F74738336F7EFE889941A27201190D4A5DA0126` |

`test.ps1` 已在当前完整工作树通过 Rust 算法 67 项、控制 10 项、bridge 8 项、仿真
10 项，共 95 项，同时通过格式、Clippy、CPU/STM32G4 目标库、C Host 1/1 和 PMSM
闭环仿真。除本轮两个逻辑文件外，49 个被修改的 C/Rust/头文件/链接脚本经剥离注释后
与 `HEAD` 的代码本体一致。

## 5. 低功率实机结果

条件：NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T，约 12.28 V，电源上限
2 A，软件过流 trip 1.15 A，空载自由旋转，默认开环，目标 582 rpm，trace 关闭，每次
5 秒。下载使用 STM32CubeProgrammer 2.19.0，verify/reset 通过。

| 运行 | 完整 ISR | pre | control | post | 软件 deadline | invalid/miss/error/fault |
|---|---:|---:|---:|---:|---:|---|
| 1 | 7,957 | 472 | 7,099 | 386 | 12,500 | 0/0/0/0 |
| 2 | 7,957 | 472 | 7,099 | 386 | 12,500 | 0/0/0/0 |
| 3 | 7,957 | 472 | 7,099 | 386 | 12,500 | 0/0/0/0 |

170 MHz / 12 kHz 的物理周期约为 14,167 cycles；本批最坏 7,957 cycles 占物理周期
约 56.2%，同时低于 12,500 cycles 软件截止。原始记录：

- `simulation/results/timing_diagnostic_s_fpu_magnitude_current_run1_openloop_582rpm_12v_20260924.json`
- `simulation/results/timing_diagnostic_s_fpu_magnitude_current_run2_openloop_582rpm_12v_20260924.json`
- `simulation/results/timing_diagnostic_s_fpu_magnitude_current_run3_openloop_582rpm_12v_20260924.json`

## 6. 证据边界与下一步

当前证据达到：

- S1：Host、Rust、格式、Clippy 和 PC 仿真；
- S2：Diagnostic/Production 交叉构建、size、hash 和反汇编；
- S4：Diagnostic 下载、校验、复位和串口通信；
- S5：同一套硬件上的三次低压、限流、空载、开环短测。

每组结束均执行 `foc_stop`。最终再次复位并回读：`uninitialized`、`closed=0`、0 steps、
0 error、0 fault、duty 0/0/0，未自动启动电机，TIM1 输出和门极保持关闭。

未达到 S6：没有编码器/测速仪独立真值、示波器对拍、故障注入、负载、温升、长测或
多样本板卡证据；Production 只构建，未烧录。下一步保持当前安全参数不变，增加仅诊断
使用的分项计时，分别测 CORDIC `sin/cos` 与 `atan2` 的平均/最坏周期及 CPU 回退，之后
再决定是否采用固定延迟读、Q 格式调整或 CPU 近似。
