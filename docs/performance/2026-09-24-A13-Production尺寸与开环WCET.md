# A13：Production 构建尺寸与开环 WCET 对比

日期：2026-09-24  
基线：`main` / `41796e52928768c75b5ee9d162ed405bb1d19c0a`，包含未提交的 A0～A12 工作区  
板卡：NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T

## 1. 结论

Production 档的主要收益是容量，不是电流环计算速度：

- ROM 比 Diagnostic 少 31,056 B，Flash 占用从 95.88% 降到 72.19%；
- RAM 比 Diagnostic 少 5,264 B，占用从 43.14% 降到 27.08%；
- 同口径 582 rpm 开环 5 s 的完整 ISR WCET 只从 8,906 降到 8,855 cycles，改善
  51 cycles（约 0.57%）；
- 两个镜像均为 0 error、0 fault、0 deadline miss；
- Production 当前编译掉运行时调参，因此 `closed_loop_enable=0` 不能在线改为 1。
  本轮只能证明开环实时基线，不能证明 Production 闭环性能或发布就绪。

## 2. 构建与裁剪检查

共同配置：STM32G431、12/12 kHz、Rust `opt-level=s`、CORDIC + FPU、角补偿 `0/0`。

| 指标 | Diagnostic | Production | 差值 |
|---|---:|---:|---:|
| text | 123,912 B | 93,472 B | -30,440 B |
| data | 1,764 B | 1,148 B | -616 B |
| bss | 12,372 B | 7,724 B | -4,648 B |
| ROM = text + data | 125,676 B | 94,620 B | -31,056 B |
| RAM = data + bss | 14,136 B | 8,872 B | -5,264 B |
| Flash 占用 | 95.88% | 72.19% | -23.69 个百分点 |
| Flash 余量 | 5,396 B | 36,452 B | +31,056 B |

`arm-none-eabi-nm` 核对结果：Production 不再包含以下符号，而 Diagnostic 保留：

- `foc_cfg` 与 `__fsym_foc_cfg`；
- `foc_trace` 与 `__fsym_foc_trace`；
- `strtof`；
- `g_foc_trace_buffer`（Diagnostic 为 4,608 B，即 64 × 72 B）。

Production `build.ninja` 对应用层与 FOC 平台层均注入了
`FLUXRT_PRODUCTION_BUILD=1`；CMake cache 为 `production / s`。24/12 kHz 和数学诊断
候选在该档被强制忽略，仍使用 12/12 kHz 正式路径。

## 3. 受限实机对比

条件：板端母线约 12.275 V；用户此前确认电源最大 2 A、电机空载且可自由旋转；软件
trip 1.15 A；目标 582 rpm；每个镜像开环运行 5 s；trace 关闭。采集脚本在每组开始和
异常/结束路径都发送 `foc_stop`。

| 项目 | A12 Diagnostic | A12 Production |
|---|---:|---:|
| 样本数 | 66,077 | 66,078 |
| 完整 ISR WCET | 8,906 cycles | 8,855 cycles |
| 周期占用（截止 12,500） | 71.25% | 70.84% |
| 软件余量 | 3,594 cycles | 3,645 cycles |
| WCET pre/control/post | 487 / 7,686 / 733 | 486 / 7,689 / 680 |
| 独立峰值 pre/control/post | 507 / 7,694 / 733 | 499 / 7,698 / 680 |
| 峰值相电流估计 | 约 909 mA | 约 903 mA |
| error / fault / miss | 0 / 0 / 0 | 0 / 0 / 0 |
| 最终状态 | open-loop-hold，已停机 | open-loop-hold，已停机 |

`pre/control/post` 的独立峰值不能相加；表中的完整 WCET 是同一拍关联值。Production
省掉的主要是 trace 后处理，控制段 7,689 与 Diagnostic 的 7,686 cycles 属于测量噪声
范围，不能宣称 Production 让 Rust 控制算法更快。

原始结果（构建产物目录外、Git 忽略）：

- `simulation/results/timing-a12-diagnostic-openloop-20260924.json/.log`
- `simulation/results/timing-a12-production-openloop-20260924.json/.log`

## 4. 固件哈希

Diagnostic（最终恢复到板上）：

- `fluxrt.bin`：`2D970F493637AADA58F9AE40BBEFA4B42253A824E4FD23C3A691B5DA9CB9E092`
- `fluxrt.hex`：`B421D142EA4E473D0BB34462DA56FDF99631E42F656FCC7F2A942B5D8AD90359`
- `rtthread.elf`：`ECF9768304D016E44ECD581C945F5FCDA0E0762866E0996025E6B82ED709A791`

Production（已验证烧录与开环运行，随后被 Diagnostic 覆盖）：

- `fluxrt.bin`：`C84C8F08FCACBD45A36180583681EA2CB5BCCED944DE31625E68FE4FAE0C8EA1`
- `fluxrt.hex`：`E667A36852D9DAF141229A0B2B8CAEBDA143FEABB11DD67A5B503249B165F29D`
- `rtthread.elf`：`84F5A89F3B0A73DC91B97F124BB07DF5762FF0CE58D2908844BF9F65E10348FF`

## 5. 最终板卡状态

已重新烧录并校验 A12 Diagnostic。最终串口回读：

- `closed_loop=0`；Park/RevPark 预测为 `0/0 milli-ticks`；
- state `uninitialized`，steps/errors/misses 均为 0；
- duty `0/0/0`，平台 flags `0x0000231f`，不含 armed/output-active；
- 12/12 kHz、ratio 1、actuation delay 1 PWM tick；
- 母线约 12.275 V。

## 6. 后续架构要求

Production 不能依赖现场 Shell 打开闭环，这是正确的安全方向；但未来闭环产品必须有
“经过审批的不可变参数档案”，在编译期或带版本/CRC 的参数区注入，而不是把
`foc_cfg` 放回 Production。推荐顺序：

1. 继续保持当前安全默认 `closed_loop_enable=0`；
2. 完成电机参数辨识、独立角度真值和闭环验收；
3. 建立 `ProductionMotorProfile`：版本、板卡 ID、电机 ID、参数 CRC、审批状态；
4. 只有审批档案可以在启动配置阶段设置闭环权限，运行中仍禁止改快环参数；
5. 再对 Production 做闭环 WCET、故障注入、长测和多样本验证。

因此，本轮 Production 是“容量与开环实时基线通过”，不是“生产发布通过”。
