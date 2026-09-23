# A0 关联 WCET 与固件尺寸基线（2026-09-23）

## 1. 结论

本轮修正了旧时序数据的两个口径错误：旧 `maximum_isr_cycles` 没有覆盖 ADC ISR
入口到 ADC 标志清理的完整区间，旧 `pre/control/post` 又是三个不同控制拍的独立最大值，
不能相加。新固件同时保存完整 ISR WCET 及其同一拍的三个分段，并继续单独保存各段峰值。

默认开环在 trace 关闭、10 Hz、50 Hz 三组 5 秒试验中均无控制错误和 deadline miss。
闭环三组中只有 50 Hz 组跑满；trace 关闭和 10 Hz 组约 2.3 秒后触发
`FOC_RUST_FAULT_OBSERVER_LOST (0x8)`。不同重复试验的通过模式并不固定，因此当前证据指向
观察器可靠性裕量不足，不能归因于某个 trace 频率。

## 2. 固件与配置

| 项目 | 值 |
|---|---|
| 基线提交 | `e87079286070155a85ff85540b4d924c91060d1e` |
| 当前结果提交 | 本报告所在提交 |
| MCU / 板卡 | STM32G431RB / NUCLEO-G431RB + X-NUCLEO-IHM16M1 |
| 电机 | GBM2804H-100T，空载且可自由旋转 |
| 供电 | 实测约 12.26～12.31 V；电源限流 2 A |
| PWM / 控制环 | 12 kHz / 12 kHz |
| 观察器 | Rust SMO-PLL，每拍更新 |
| 软件截止 / 物理周期 | 12,500 / 约 14,167 cycles（170 MHz） |
| 默认闭环权限 | `closed_loop_enable=0`，试验结束后已恢复 |
| PA5 时序探针 | `FOC_ISR_TIMING_PROBE=n`，默认关闭 |

最终烧录并校验的固件：

| 文件 | 大小 | SHA-256 |
|---|---:|---|
| `cmake-build/fluxrt.bin` | 127,820 B | `FEE8CFB8E98B0AB382A6FE2E49ECF2C7D398134C9FBE7D48B7DC7720996CBE75` |
| `cmake-build/fluxrt.hex` | 359,604 B | `AA5EA5A9797BC69E62643556354F89DF2790916DA1F16D604696098BEA6572D7` |
| `cmake-build/rtthread.elf` | 2,964,496 B | `2BB7068D2BCD8894602390A131B2F0969CC510095505A0A8601A33E33DBBD32F` |

链接统计为 `text=126,056 B`、`data=1,764 B`、`bss=9,964 B`；Flash load
127,820/131,072 B（97.52%，剩余 3,252 B），RAM 11,728/32,768 B（35.79%）。
相对 `LOG-20260923-002` 基线，关联时序能力增加 768 B ROM、56 B RAM；下一工作包必须先
处理 Flash 配置和裁剪，不能继续无预算地加入功能。

## 3. 同拍时序定义

测量区间从 `ADC1_2_IRQHandler()` 入口开始，到 ADC1/ADC2 injected flags 清理之后结束。
同一拍满足：

```text
total_cycles = precontrol_cycles + control_cycles + postcontrol_cycles
```

- `precontrol`：IRQ 入口、电流重构、保护检查到 Rust 快环调用前；
- `control`：一次 `foc_rust_realtime_step()`；
- `postcontrol`：输出校验、CCR 更新、trace 采样和 ADC flags 清理；
- `peak_pre/control/post`：各段跨所有拍的独立峰值，只用于定位热点，禁止相加；
- `FTIMING`：机器可读记录，同时带 samples、invalid、deadline miss、控制错误和 fault。

可选 `FOC_ISR_TIMING_PROBE=y` 会让 PA5（NUCLEO LD2/D13）在相同 ISR 区间输出高脉冲；
该分支已完成目标构建（Flash 127,908 B），随后恢复默认关闭并重建；尚未用示波器完成
DWT 对拍。

## 4. 默认开环基线

命令由 `simulation/capture_timing_baseline.py` 自动执行，每组都先停机、配置 trace、运行
5 秒、停止功率级，再读取 `foc_status`；最终再次 `foc_stop`。

| Trace | 样本 | 完整 WCET | 同拍 pre/control/post | 独立段峰值 | 软件截止余量 | 结果 |
|---:|---:|---:|---:|---:|---:|---|
| 关闭 | 66,100 | 8,161 | 506 / 7,269 / 386 | 507 / 7,300 / 386 | 4,339 | PASS |
| 10 Hz | 66,233 | 10,000 | 472 / 7,207 / 2,321 | 507 / 7,288 / 2,321 | 2,500 | PASS |
| 50 Hz | 66,098 | 10,030 | 472 / 7,257 / 2,301 | 507 / 7,288 / 2,321 | 2,470 | PASS |

三组均为 `invalid=0`、`misses=0`、`errors=0`、`fault=0`。原始本机证据位于
`simulation/results/timing_openloop_582rpm_12v_20260923.json/.log`，该目录按仓库规则不提交。

## 5. 闭环复测

| Trace | 有效样本 | 完整 WCET | 同拍 pre/control/post | miss/error/fault | 结果 |
|---:|---:|---:|---:|---:|---|
| 关闭 | 27,515 | 8,707 | 472 / 7,849 / 386 | 0 / 1 / `0x8` | FAIL：观察器失锁 |
| 10 Hz | 27,635 | 10,121 | 472 / 7,349 / 2,300 | 0 / 1 / `0x8` | FAIL：观察器失锁 |
| 50 Hz | 66,146 | 10,209 | 492 / 7,416 / 2,301 | 0 / 0 / `0x0` | PASS |

本机原始证据位于
`simulation/results/timing_closedloop_final_582rpm_12v_20260923.json/.log`。失败组仍安全进入
故障关断，未发生 deadline miss；但这不是闭环稳定性通过证据。

## 6. 已达到与未达到

- 已达到：关联 WCET 数据结构、同拍校验 Host 测试、完整 ISR DWT 区间、三种 trace 状态、
  size/map、烧录校验、默认开环实机基线、故障与时序结果分离判定；
- 未达到：PA5 示波器脉冲与 DWT 数值对拍、独立编码器/测速真值、闭环重复稳定；
- 安全状态：试验后 `foc_stop`、trace 停止、`closed_loop_enable=0`，功率级保持关闭；
- 下一步：先执行 A0 的 `O3/Os/Oz` 尺寸与 WCET 对比并建立 production/diagnostic 构建档，
  同时把 `OBSERVER_LOST` 作为独立问题保留，不在结构整改中偷偷放宽 50 ms 失锁阈值。
