# 阶段 1.1 构建优化矩阵与 Production 候选（2026-09-23）

## 1. 结论

Rust `opt-level=s` 被选为新默认：相对 `3`，Diagnostic BIN 从 127,892 B 降到
122,812 B，回收 5,080 B；实机完整 ISR WCET 从 8,098 增至 8,257 cycles，约增加
2%，仍只占 12 kHz 物理周期约 58.3%。

`z` 的 BIN 为 122,012 B，只比 `s` 再小 800 B，但 WCET 增至 8,857 cycles，因此仅
保留为尺寸下界。Production + `s` 裁掉在线浮点调参和 trace 后，BIN 为 93,004 B，
Flash 剩余 38,068 B（29.04%），实机 WCET 为 8,160 cycles。

## 2. 构建与实机矩阵

共同条件：全局 C `-Os`、FOC C 适配层 `-O3`、Rust LTO、12 kHz PWM/控制环、CORDIC、
默认开环、582 rpm、约 12.28 V、空载、每组 5 秒、trace 关闭。

| 构建 | text | data | bss | BIN/Flash load | Flash 剩余 | 完整 ISR WCET | 物理周期占比 | 结果 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| Diagnostic + `3` | 126,128 | 1,764 | 9,964 | 127,892 | 3,180 | 8,098 | 57.2% | PASS |
| Diagnostic + `s` | 121,048 | 1,764 | 9,964 | 122,812 | 8,260 | 8,257 | 58.3% | PASS，选为默认 |
| Diagnostic + `z` | 120,248 | 1,764 | 9,964 | 122,012 | 9,060 | 8,857 | 62.5% | PASS，仅尺寸下界 |
| Production + `s` | 91,856 | 1,148 | 6,852 | 93,004 | 38,068 | 8,160 | 57.6% | PASS，生产候选 |

四组均为 `invalid=0`、`misses=0`、`errors=0`、`fault=0`。12 kHz 物理周期按 170 MHz
约为 14,167 cycles，70% 阶段门槛约为 9,917 cycles；四组都通过该门槛。

原始结果位于本机 ignored 目录：

- `simulation/results/timing_diagnostic_o3_final_openloop_582rpm_12v_20260923.json/.log`；
- `simulation/results/timing_diagnostic_s_final_openloop_582rpm_12v_20260923.json/.log`；
- `simulation/results/timing_diagnostic_oz_final_openloop_582rpm_12v_20260923.json/.log`；
- `simulation/results/timing_production_s_openloop_582rpm_12v_20260923.json/.log`。

## 3. 最终产物

板上最终恢复为 Diagnostic + `s`：

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `cmake-build/fluxrt.bin` | 122,812 B | `EF66C0A1FAE31F32C183A3351F6C1DA36526A81D9DF45B0807F44DAD8E4FD9A1` |
| `cmake-build/fluxrt.hex` | 345,519 B | `CDBFB76B45395BAECE7DF424080181AF753C50191D13D8147C5BC35265AA15E7` |
| `cmake-build/rtthread.elf` | 2,965,712 B | `0B9EF74B0BF4EE98FE7848DDA84AEDDBD13E89DAE1B4457419E964A64E640543` |

Production + `s` 候选：

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `cmake-build-production/fluxrt.bin` | 93,004 B | `7FFB36F867E39058F1EDE4EC797DF5513D11A429DCB8E1159FD64CDCE0A0B28E` |
| `cmake-build-production/fluxrt.hex` | 261,671 B | `7A5EFE4BC2C7C05137C3CEC46B4039688DAF1554C2BB3338A9EA334EC99F44E9` |
| `cmake-build-production/rtthread.elf` | 2,872,220 B | `B92600B4F6B8B533E26C76218C0392CEBAFE8A8B061525A6FF18A4ADC7E14EAF` |

## 4. 裁剪依据

Diagnostic map 显示在线 `foc_cfg` 的 `strtof` 引入 newlib `strtod`、断言和
`vfiprintf` 依赖链；trace 环形缓冲占 3,072 B BSS。Production 构建：

- 保留 `foc_start`、`foc_stop`、`foc_status`；
- 保留同拍 WCET、deadline、过流、母线、驱动故障、PWM 输出复核和立即关断；
- 裁掉 `foc_cfg`、`foc_trace`、trace ISR 采样与输出；
- `nm` 验证最终 ELF 不含 `g_foc_trace*`、`foc_cfg`、`strtod/strtof`、`vfiprintf`；
- `nm` 验证仍含 `foc_platform_control_start/stop`、`ADC1_2_IRQHandler` 和三个安全命令。

这属于按职责裁剪诊断能力，不是删除安全检查。Production 仍不是发布证明，因为尚未
完成独立速度/角度真值、故障注入、温升长测和多样本一致性。

## 5. 构建系统修正

每个 Rust 优化等级使用独立 `build/rust-target-3|s|z`。实施时曾发现 RT-Thread 生成的
CMake 仍指向旧 `build/rust-target`，会出现“编译了新档位但链接旧静态库”的误标风险；
现已在 `custom.cmake` 中覆盖 `rtt_FOC` 的链接目录，并用最终 BIN 尺寸和三档实机 WCET
重新验证。

`build.ps1` 还会：

- 检查 `-BuildOnly` 请求与 CMake cache 是否一致；
- 为 Diagnostic/Production 使用不同输出目录；
- 结束时恢复 PATH 和 RT-Thread 环境变量，避免后续 Python 解析到错误解释器。

## 6. 验证边界与下一步

- S1：Rust 94 项、格式、Clippy、CPU/CORDIC 交叉库、C Host 1/1 和 PMSM 仿真通过；
- S2：Diagnostic `3/s/z`、Production `s` 全部目标构建通过；
- S4：四个实测固件均完成 SWD 下载、verify 和 reset；
- S5：四组低压限流空载 5 秒开环通过，均无 fault 和 deadline miss；
- S6：未完成示波器 DWT 对拍、独立编码器真值、故障注入和长测。

下一步进入阶段 1 的快环热点优化：共享 Clarke、中间量复用和减少 CORDIC 事务。在此
之前不提升默认闭环权限，也不放宽观察器失锁阈值。
