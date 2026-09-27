# EXP-B3 S2：Identification 构建门

## 1. 结论

EXP-B3 已完成 **S2 静态目标构建门**：FluxRT 现在有独立的 Identification 主档，只有
该档能在纯应用层启动 `Ls(I)` 状态机；普通电机 `foc_start` 在 Identification 和
Calibration 中由应用层、平台层双重拒绝。

本次没有 Shell/管理入口、PWM/ADC 注入适配器、烧录、串口或电机运行。S2 PASS 只表示
Host 门、STM32 交叉构建、size/map 和 ISR 静态对比通过，不表示板端 WCET 或功率实验通过。

## 2. 构建契约

四个主档必须且只能定义一个：

```text
FLUXRT_DIAGNOSTIC_BUILD
FLUXRT_CALIBRATION_BUILD
FLUXRT_IDENTIFICATION_BUILD
FLUXRT_PRODUCTION_BUILD
```

Identification 的能力掩码只有 `FLUXRT_BUILD_CAP_LSI_IDENTIFICATION`。Calibration 和
Identification 额外派生 `FLUXRT_MOTOR_ARM_DISABLED_BUILD`；该宏同时控制应用层 Shell
拒绝和平台层 `foc_platform_control_start()` 的 fail-closed 返回。其他三个主档调用
`foc_lsi_request_start()` 时保持 `IDLE`。

## 3. Host 证据

执行完整 `test.ps1`：

- profile 工具：140 tests PASS；
- simulation：7 tests PASS；
- Rust：72 + 32 + 18 + 17 tests PASS，并通过 clippy；
- Cortex-M4F CPU/CORDIC Rust 静态库构建 PASS；
- C Host CTest：8/8 PASS。

C Host 覆盖四档能力矩阵、仅 Identification 的状态机启动授权，以及 Calibration /
Identification 的平台 normal-arm 拒绝。状态机测试继续覆盖完整单档序列、任意运行阶段
abort、故障、软件 trip、过流、母线越界、非法输入、总超时和手工复位。

## 4. STM32 目标构建矩阵

工具链：Arm GNU Toolchain 15.3.1；Cargo/Rustc 1.98.1；Rust 优化等级 `s`。

| Profile | ROM (`text+data`) | RAM (`data+bss`) | BIN SHA-256 |
|---|---:|---:|---|
| Diagnostic | 130,880 B | 23,120 B | `624D440BE3A0B93C4582D3FC23CDC4E28536E98049F6D1F97E74A7140B95AC53` |
| Calibration | 100,304 B | 13,072 B | `3BDE71847184AB596FC2ECFEF712E96FA05AA5B6F295C48CCBA96C3D1BCACE4F` |
| Identification | 97,292 B | 8,952 B | `DAC031A739F9356901F721B17C38CB771A55E13F40044902197FD243ED69AD59` |
| Production | 98,396 B | 8,952 B | `0D27BF43F80CEB4E9093099EB8ADDE21061474BA54DA57B692D81C3906F3E390` |

命令模板：

```powershell
.\build.ps1 -Regenerate -Profile <Profile> -RustOptLevel s
```

Diagnostic 仅剩 192 B Flash 余量；后续 Identification 管理入口不得加入 Diagnostic，
也不能复用其大段 Shell/浮点格式化路径。

## 5. ISR 与链接证据

从 Identification、Production 的平台对象中提取 `.text.ADC1_2_IRQHandler`：

```text
size = 1208 B / 1208 B
sha256 = 4BEB9573D77E101D84C281C5A8353B9E3F76D548E96E80C9C3CCC8FD0E86FA62
```

两份机器码完全相同，所以本次构建门没有给 ADC 快中断增加指令。这个结论是静态的；
只有将来真正加入注入适配器后，才需要重新做板端同拍 WCET 和 deadline 统计。

`foc_lsi_identification.c` 在 Identification 目标对象中产生 1,350 B `.text`；因为 S2
刻意没有管理入口，最终 map 将这些输入 section 放在地址 0 并全部丢弃，最终 ELF 中
`foc_lsi_*` 可运行符号数为 0。Identification/Calibration 最终 ELF 也没有正常
`foc_platform_control_start` 实现；Production 中该函数仍为 600 B。

## 6. 证据边界与下一步

- 板上仍保持此前 CM4.4 Diagnostic rev6 基线；本次四个镜像均未烧录。
- rev7 仍是未审批、未集成、未烧录候选。
- 下一步 S4 只添加无功率 `status/dump/reset/abort` 管理入口和重启恢复验证；普通 arm
  必须继续明确拒绝。
- PWM/ADC 注入、applied-voltage 重构和原始样本缓冲属于后续独立门，不随 S4 自动开放。
