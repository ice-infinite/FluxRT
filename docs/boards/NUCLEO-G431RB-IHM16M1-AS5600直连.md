# NUCLEO-G431RB + IHM16M1 直连 AS5600

## 1. 目的和边界

本接法把 DengFOC 电机上的 AS5600 直接接入 STM32G431，作为独立转子角度真值，避免
两块控制器、同步线和双串口时间映射。当前只批准 **USB/ST-LINK + 3.3 V 传感器诊断**：

- 12.3 V 主电源保持断开；
- 不运行 PWM，不初始化电机输出；
- 诊断固件编译期定义 `FLUXRT_MOTOR_ARM_DISABLED_BUILD`，`foc_start` 必须拒绝；
- I2C 只在有界 Shell 命令中运行，不进入 12 kHz ADC ISR。

本页记录的是已经在当前实板通过 S4 的接线与读取结果，不代表电角度零偏、极对数、
HFI 或全速域闭环已经批准。

## 2. 接线

| AS5600 插座 | NUCLEO-G431RB | Morpho 位置 | 作用 |
|---|---|---|---|
| `VCC` | `+3V3` | `CN7-16` | 传感器供电，禁止接 5 V |
| `GND` | `GND` | `CN7-20` 或 `CN7-22` | 公共地 |
| `IO5` | `PC8` | `CN10-2` | `I2C3_SCL` |
| `IO23` | `PC11` | `CN7-2` | `I2C3_SDA` |
| `IO21` | 不接 | - | 当前诊断不用 |

引脚复用依据：STM32G431 数据手册中 PC8/PC11 分别具有 `I2C3_SCL/I2C3_SDA`，
NUCLEO-G431RB 用户手册 Figure 18 给出 CN10-2/PC8 与 CN7-2/PC11。参考资料：

- [STM32G431 数据手册](https://www.st.com/resource/en/datasheet/stm32g431rb.pdf)
- [NUCLEO-G431RB 用户手册](https://www.st.com/resource/en/user_manual/dm00556337-stm32g4-nucleo-64-boards-mb1367-stmicroelectronics.pdf)
- [X-NUCLEO-IHM16M1 原理图](https://www.st.com/resource/en/schematic_pack/x-nucleo-ihm16m1_schematic.pdf)

## 3. 板级冲突

IHM16M1 上 PC8 通过 R87（0 Ω）进入 H3 网络，PB10 又通过 R84（0 Ω）接到同一网络。
因此该诊断档有三个硬约束：

1. PB10 保持高阻输入；
2. 不同时接 Hall H3 信号；
3. PC8/PC11 只由 I2C3 平台端口拥有。

这也是本实现没有把 I2C 初始化放进通用算法层的原因：板级复用和 GPIO 所有权只存在于
`foc/platform/stm32g431/foc_as5600_stm32g431.c`。

## 4. 软件分层

| 层 | 路径 | 职责 |
|---|---|---|
| 公共解码/跟踪 | `foc/runtime/foc_as5600.c` | 12 bit 解码、跨零展开、速度和标准反馈样本；无 HAL/RTOS |
| G431 平台端口 | `foc/platform/stm32g431/foc_as5600_stm32g431.c` | I2C3、PC8/PC11、总线恢复、错误/引脚诊断 |
| 操作入口 | `applications/foc_as5600_shell.c` | 停机后单点/有界批量采样，输出机器可读 `FENC` |
| 构建安全门 | `FOC_AS5600_TRUTH_DIAGNOSTIC` | Basic-Drive + Diagnostic 专用，普通 motor arm 编译禁用 |
| 受限定向候选 | `FOC_AS5600_ALIGNMENT_CANDIDATE` | 仅开放固定令牌的0.20 A/1 s ISR硬停止事务，不开放普通start |

I2C3 使用 170 MHz PCLK1 和保守约 55 kHz Standard-mode timing。第一次接线验证优先保证
上拉、线序和 ACK 稳定；后续若需要把编码器送入实时闭环，应改为定时器调度的非阻塞/DMA
采集与固定时戳，不能直接把当前 HAL 阻塞读取搬进 ISR。

## 5. 构建与命令

`.config` 选择：

```text
CONFIG_FLUXRT_G431_PROFILE_BASIC_DRIVE=y
CONFIG_FOC_AS5600_TRUTH_DIAGNOSTIC=y
CONFIG_FOC_AS5600_ALIGNMENT_CANDIDATE=y
```

构建：

```powershell
.\build.ps1 -Regenerate -Profile Diagnostic -RustOptLevel s
```

Shell：

```text
foc_encoder_status
foc_encoder_watch 20 50
```

`FENC` 字段顺序：

```text
FENC,index,ok,time_ms,raw_count,angle_mdeg,present,read_count,error_count,
     hal_status,hal_error,i2c_isr,scl_high,sda_high
```

## 6. 2026-10-06 实板结果

- 当前固件：106,248 B，SHA-256
  `BED680E26F98FA591CD10A7BF35B68F1F6FAD5C6177A0FF940DA248374A1B841`；
- STM32CubeProgrammer：下载、校验、复位成功；
- 首次单点：`raw=2983`，`262.177°`；连续 11 次均成功，错误 0；
- 后续单点：`raw=1174`，`103.183°`，证明手动改变转子位置后角度链有响应；
- I2C：地址 `0x36` ACK，SCL/SDA 空闲均为高，HAL status/error 均为 0；
- 动态双向采集：300/300有效、0错误、跨零2次；两个方向累计约1.398圈和1.357圈，
  段内采样间隔中位数105 ms、最大106 ms；段间Shell重发命令形成499/505 ms间隙；
- 安全：`foc_start` 明确拒绝；`disabled`、`duty=0/0/0`、`fault=0`、`miss=0`，
  Vbus ADC 原始值为 0；12.3 V 未接入。

当前结论是“G431 直连 AS5600 的静态、位置变化和双向跨零读取 S4 通过”。第二次独立
方向窗口从轴伸出端观察顺时针手转，100/100点有效，计数净增加1721，正向/反向累计
1785/64 count，方向优势96.54%，因此项目机械方向约定确认：**顺时针 = AS5600计数增加 =
`direction=+1`**。窗口只覆盖约0.42圈而非完整一圈，但已超过方向判定所需的0.25圈且
无总线错误；完整跨零能力由上一轮双向2.75圈数据独立覆盖。

DengFOC当前源码和既有Profiler记录均给出7极对，故`pole_pairs=7`可继续作为已有实测/项目
候选，不是本次手转新测值。历史同一电机、相序和传感器的稳定对齐候选为
`0.9658693323 rad`（加法口径）。2026-10-06已用G431直连传感器完成一次固定0.20 A/1 s
受限定向对齐，得到新候选`0.8387807509 rad`；尾段最大偏差0.0150330678 rad，与历史候选
圆周差0.1270885814 rad，均通过预设门限，但仍标记未批准。这些完成前不能把
`calibrated` 置 1，也不能批准 HFI/全速域闭环。

2026-10-06 已进一步烧录受限定向对齐候选，固件 SHA-256 为
`1A169220D3F3905ADB991BAA0F77FAFF7B9C1C66F19D34855DD7EDAC4244648E`。无主电检查确认
普通 `foc_start` 仍被拒绝，AS5600 可读。固定`P55-ALIGN1`随后只执行一次并在12,000拍
自动关断：0 deadline/error、峰值约148 mA、最终disabled/duty0/fault0。本次通过的是
S5候选复验，不等于参数审批或全速域能力批准。详细包络与结果见
[G431-AS5600受限定向对齐准备门](../performance/2026-10-06-P5.5-G431-AS5600受限定向对齐准备门.md)。
