# A19：Calibration 板端安全门与静态原始码基线

## 1. 结论

A19 已进入**进行中**。独立 `Calibration` 固件已经完成首次目标板下载、启动和串口
拒绝门验证：固件自报 `Build profile=calibration`，执行 `foc_start 582` 被明确拒绝，
拒绝前后控制步数、错误、deadline miss 和三相 duty 都为 0。电机没有运行。

随后在输出保持关闭的条件下完成一窗 12 kHz、256 拍三相相电压 ADC 原始码采集，并在
结束路径关闭 `IO_BEMF`、再次执行 `foc_stop`。这只证明 Calibration 的 S4 安全门和原始
采集链路工作，不是电压标定结果。用户随后用万用表报告 `VS=12.30 V`、`VDD=3.33 V`、
停机 U/V/W 相端均为 `1.60 V`；由于测量时电机/LA1010 是否断开、仪表型号和外部注入
状态尚未确认，这组数据只作为预检查，A19 不能标为完成，也不能把 ADC 码送入观察器。
在器材等待期间，软件侧补齐 PC9 `divider-off/on` 显式采集模式和 CSV 自动入档/拟合流程，
并把新 Calibration 下载到板上重复验证拒绝门。在 VS 已放电、无外部注入的条件下完成
off/on 各三窗基线；PC9 切换效果可见，但数据仍没有可信电压真值，不能用于增益拟合。

用户随后决定先按 ST 官方默认路径继续。A19.1 已把 3.3 V、12 bit、10 kΩ/2.2 kΩ 写成
`nominal-not-calibrated`、`diagnostic-only` 的换算模型，并把 Rust 观察器电压来源集中到
默认 `CommandModel`；`PhaseVoltage/Hybrid` 仍不可用。这个决定允许官方兼容主线继续，
但不把 A19 逐板实测标定伪装为完成。详细证据见
[A19.1 官方名义电压模型与 CommandModel 基线](2026-09-24-A19.1-官方名义电压模型与CommandModel基线.md)。

## 2. 硬件、固件和工具

| 项目 | 本次值 |
|---|---|
| 控制板 / 功率板 | NUCLEO-G431RB / X-NUCLEO-IHM16M1 |
| 电机 | GBM2804H-100T；本次未启动 |
| ST-LINK | `00400027544B500120343637`，V3J9M3 |
| 串口 | COM6，115200 8N1 |
| 烧录工具 | STM32CubeProgrammer 2.19.0 |
| 固件 | `cmake-build-calibration/fluxrt.bin`，98,248 B |
| BIN SHA-256 | `C9DE27D276580C2A867894DADB79E402B094451AA45DA88A17D4E314D5322BD6` |
| 构建 | `build.ps1 -Profile Calibration -RustOptLevel s` |
| 母线 | 首次约 12.30 V；本次新固件烧录和双模式采集时已断电放电，最终固件回读 0 mV |
| VDD | 用户万用表报告 3.33 V；仪表型号/准确度待记录 |
| 功率条件 | 主电源关闭；电机/LA1010 沿用上一轮已断开状态；功率输出始终关闭 |

LA1010 已识别，且按 IHM16M1 原理图核对为 `TP6=OUTW`、`TP7=OUTV`、`TP8=OUTU`，
因此当前接线 `CH0/CH1/CH3` 对应 `W/V/U`。这些是功率相端而不是 3.3 V PWM 或分压后的
BEMF 点。A19 的静态增益拟合仍需要万用表；LA1010 留给 A20 的动态相序和开关时序预筛查。

### 2.1 万用表预检查

用户报告黑表笔以板卡 GND 为参考时：

| 测点 | 读数 | 当前解释 |
|---|---:|---|
| VS | 12.30 V | 与非同时刻固件粗略值 12.275～12.301 V 相差不超过约 25 mV，仅是合理性检查 |
| VDD | 3.33 V | 可作为后续 ADC 元数据候选，仍需记录万用表型号/准确度 |
| TP8 / U | 1.60 V | 停机浮空相端预检查，不是受控 0 V 点 |
| TP7 / V | 1.60 V | 同上 |
| TP6 / W | 1.60 V | 同上 |

三相相同只能说明当前万用表分辨率下共模电位接近；若电机仍接在 CN3，三相绕组本身会
把相端耦合在一起。即使电机已拔，关闭的半桥、指示 LED、BEMF 网络和漏电路径仍可让
相端停在非零浮空电位。因此 `1.60 V` 不能作为偏置、零点或校准拟合输入。下一次注入前
必须确认电机和 LA1010 均已断开，并用同一 VS/GND 派生、带串联限流的低阻抗受控电压源。

## 3. S4 下载和双层拒绝门

CubeProgrammer 使用 SWD under-reset 下载，完成全片相关扇区擦除、写入、逐字节 verify 和
复位。启动自报的关键行是：

```text
Build profile=calibration Rust opt-level=s.
Rust ABI=0x000c0000 context=1232/2048 init/config=0; platform cfg/init/bind=0/0/0.
Monitor feedback=0 flags=0x0004231f PWM/control=12000/12000Hz ... Outputs DISABLED.
Commands: foc_status, foc_phase_capture, foc_stop, foc_start(refuses). Calibration is capture-only.
```

实际发送启动命令得到：

```text
foc_start 582
FOC start REFUSED: Calibration profile is capture-only; motor arm is compiled out.
foc_start: command failed -1.
```

拒绝后的 `foc_status` 仍为：

```text
steps=0 errors=0 ISRmax=0/12500 cycles misses=0
duty=0/0/0 per-mille
closed=0 target=0rpm estimate=0rpm
```

因此本次只形成 S4“成功烧录、启动、通信和 arm 拒绝”证据，不形成 S5 电机运行证据。

## 4. 静态窗口与采集工具修正

第一次转储时，固件和原始日志实际包含连续序号 `0..255`，但脚本仅统计 255 拍。原因是
最后一条 `FPV` 在固定 `dump` 等待窗口之后到达，被紧接着的 `foc_status` 读取；旧脚本只
解析 `dump` 调用的返回片段，而没有解析整个串口会话。

`simulation/capture_phase_voltage_window.py` 已改为：完成 `finally` 中的
`foc_phase_capture stop` 和 `foc_stop` 后，再统一解析整份 `raw` 会话缓冲。原始失败日志
被保留为 `phase_voltage_raw_attempt1.log`，没有冒充成功数据。修改后重新执行同一采集，
获得 256/256、序号 0..255 连续的 CSV。

### 4.1 器材等待期间的软件准备

原实现的 `foc_phase_capture start` 总会把 PC9 拉低，无法独立取得“分压网络断开”基线；
旧 CSV 也没有记录采集模式，后续容易把两类数据混用。本轮把模式变成固定窗契约的一部分：

| 命令/字段 | 行为 |
|---|---|
| `foc_phase_capture start off` | PC9 在整个 256 拍窗口保持高电平；采集关闭基线 |
| `foc_phase_capture start on` | PC9 在窗口内拉低；完成、stop、急停均恢复高电平 |
| `foc_phase_capture start` | 兼容旧脚本，等价于 `on`；新证据不应省略模式 |
| `status` / `FPV_META` | 报告本窗 `capture-divider` / `divider`，不是物理量标定结果 |
| CSV / JSON | 每行 CSV 写 `divider_mode`；JSON 写模式、统计、序号和 CSV/日志哈希 |

固定窗状态契约升到 v2；初始化默认模式为 `off`，非法枚举拒绝，ARMED 期间仍禁止读出。
`tools/foc_calibration_tool.py` 同步支持 `init-draft`、`add-point`、`fit`、`validate`：
`add-point` 从 CSV 自动计算均值/总体标准差并登记本地 SHA-256；拟合只使用 `on` 点，`off`
基线不能凑审批点。具体命令见 `profiles/calibration/README.md`。

本轮完整 Host/Rust/仿真回归通过；Diagnostic、Calibration、Production 三档目标构建均通过。
新 Calibration BIN 为 97,416 B，SHA-256
`05D238136AFD16D0E253A39738CA34E775CE1C379432379031453D390A2C269B`。它已通过指定
ST-LINK 序列号执行 SWD under-reset 下载、verify 和 reset；启动再次自报 `calibration`，
`foc_start 582` 被拒绝，拒绝前后 steps/error/miss/duty 均为 0。

### 4.2 A19.1 官方名义模型

新固件增加一条不会被误读成逐板标定的名义模型：

```text
source=st-ihm16m1-nominal quality=nominal-not-calibrated observer=disabled
Vref=3300mV divider=10000/2200 full-scale=18300mV scale=4469uV/count
```

`foc_phase_voltage_model_validate()` 会拒绝“名义参数却设置 observer-eligible”的组合；
`raw→mV` 只接受 divider-on，divider-off 原始码不会被换算成伪物理量。采集 JSON 保存固件
实际输出的模型元数据，而不是由 PC 脚本猜测参数。当前板上已更新为 98,248 B Calibration
`C9DE...2BD6`；S4 下载/verify/reset、arm 拒绝和一窗 divider-off 元数据采集均通过。

## 5. 原始码结果

证据目录：`profiles/calibration/evidence/a19-s4-20260924/`。

| 通道 | 最小 | 均值 | 最大 | 总体标准差 |
|---|---:|---:|---:|---:|
| U raw | 109 | 114.1914 | 120 | 2.3532 |
| V raw | 78 | 83.7461 | 89 | 2.3639 |
| W raw | 60 | 85.0117 | 109 | 16.6169 |
| Current U raw | 1946 | 1950.6367 | 1955 | 1.4859 |
| Current V raw | 1931 | 1932.4180 | 1934 | 0.7763 |
| Bus raw | 952 | 952.0000 | 952 | 0.0000 |

W 通道离散明显高于 U/V，但停机相端可能浮动，且当前没有同时测得的相端电压、分压端
电压和万用表参考值，所以不能把差异判定为硬件故障、比例误差或真实电压变化。

| 文件 | SHA-256 |
|---|---|
| `phase_voltage_raw_attempt1.log` | `2E7BE641E4C00514FD58EA9817AB4CB274834156B56836894FB7A64AA6F72AD4` |
| `phase_voltage_raw.csv` | `D324E7EB7A71D4D6F8662ABCE451BD7D97AAC260481F2701F7937975DF7E90DA` |
| `phase_voltage_raw.log` | `806D23BE229C6E63BB98F35AB06BAF20B73C2FE5FAFF200B23003707600320E1` |

成功日志末尾再次确认 `steps/errors/misses=0`、`duty=0/0/0`，随后输出
`IO_BEMF disabled` 和 `gate enable and TIM1 outputs are disabled`。

### 5.1 divider-off/on 无注入重复基线

证据目录：`profiles/calibration/evidence/a19-divider-baseline-20260924/`。主电源已关闭，
无外部电压注入；每种模式连续采三窗，每窗 256 拍、12 kHz，所有 CSV 序号均为 0..255，
CSV/日志 SHA-256 与同名 JSON 全部复核通过。

| 通道 | off 三窗均值的平均 | off 窗间范围 | on 三窗均值的平均 | on 窗间范围 | off-on |
|---|---:|---:|---:|---:|---:|
| U raw | 383.2122 | 0.3672 | 74.7682 | 0.1836 | 308.4440 |
| V raw | 339.3802 | 0.4102 | 35.1771 | 0.0391 | 304.2031 |
| W raw | 339.7018 | 1.1680 | 35.1146 | 2.2617 | 304.5872 |

U/V 的窗间均值重复性小于 1 count；W 的均值也能重复到约 2.3 counts，但每个窗口内部
总体标准差仍约 15.8～16.6 counts，明显高于 U/V 的约 1.0～1.7 counts。off→on 时三相
均值一致下降约 304～308 counts，证明显式 PC9 模式在板端确实改变了采样网络；由于相端
没有被低阻抗已知电压钳住，这个差值不是分压比、偏置或物理电压。

聚合数据和边界保存在 `baseline_comparison.json`，SHA-256 为
`72E45538B68CC573E91687DE6B98484D34A584A59567021478F8236CC3B9D328`。最终再次执行
`foc_phase_capture stop` 和 `foc_stop`；回读 state=complete、unread=0、steps/errors/misses=0、
duty=0、Vbus=0 mV，并明确打印 `IO_BEMF disabled` 和功率输出关闭。状态中的
`capture-divider=on` 只表示最后一窗的请求模式，不表示 stop 后 PC9 仍使能。

## 6. 未完成和下一门

A19 仍缺：

1. 板卡可追溯序列号和 IHM16M1 硬件版本；
2. 万用表型号/准确度和稳定直流注入源；
3. 用低阻抗已知 0 V 源确认真正零点；当前无注入浮空基线不能替代；
4. U/V/W 每路至少三个已知电压点，并重点复查 W 通道约 16 counts 的窗内波动；
5. 实际 10 kΩ/2.2 kΩ、ADC Vref、偏置/增益、残差和 R²；
6. `fluxrt-phase-voltage-calibration-v1` 的真实 draft/approved 记录及人工审批。

受控多点测量完成前保持 A19 实测支线为进行中，不进入依赖实测电压的 A20～A25 Hybrid
支线，也不把原始码接入 Rust 控制或观察器。官方兼容 `CommandModel` 主线可以独立继续做
target 逆变器补偿、观察器可靠性和 Rev-Up 整定。当前板上保留 Calibration `C9DE...2BD6`，
安全默认是输出关闭；需要恢复 Diagnostic 时可下载 `cmake-build/fluxrt.bin`，当前构建
SHA-256 为 `F2BC72DECCC13399090674E7E33357F6A01E655CCECDEB704C2D7B5AB3AF4E33`。
