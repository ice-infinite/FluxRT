# `Ls(I)` PC 可辨识性证据（2026-09-27）

## 结论

- 开头这组结果是 EXP-B3 的**历史 PC/S3 前置证据，不是实机辨识结果**；S5.0 实测窗
  已证明 0.5 count 噪声假设过于乐观，当前判定以本文 S5.1 小节为准。
- 12 kHz、名义 626.535 counts/A、历史 0.5 count 高斯噪声 + ADC 量化、0.4 V 对称扰动下，
  `0.893 / 1.058 / 1.113333 mH` 在 0.2/0.4/0.6 A 三档均可区分。
- 9 个工况各运行 30 次，分类率全部 100%；估计极差/均值为 1.57%～2.52%，模拟
  最大量化电流 0.6704 A。
- 使用重构后的 applied voltage 时，均值误差绝对值不超过 0.26%；若误用 command
  voltage，不扣 550 ns 死区平均压降，结果系统性偏低约 1.75%～2.77%。
- 该**历史模型**的 Host 可辨识性门曾为 PASS，但 `hardware_permission=not-granted`；
  不能再把它作为当前序列的通过证据。

## 文件

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `lsi_host_identifiability.json` | 9 工况 × 30 次的聚合值、模型假设和判定 | `e3414b6184ee7e5319dbf57cfdd43e0d550029cf958bd4544cb04adb54e684c6` |

生成命令：

```powershell
python simulation\analyze_lsi_identifiability.py --trials 30 --seed 431 `
  --output profiles\identification\evidence\lsi-20260927\lsi_host_identifiability.json
```

## 不宣称

- 模型是单轴线性 RL；饱和只通过选择不同 truth L 表示，没有铁损、磁滞或转子位置效应。
- ADC 比例和噪声是名义/合成值，尚未用板上静止原始采样估计真实噪声谱。
- PASS 只说明“12 kHz 信息量可能足够”，不证明目标实现、PWM 安全或实机参数正确。

## S5.0 一次性实机筛查

用户确认 12.3 V/2 A、空载可自由旋转和异常可立即断电后，S4.9 Identification 固件只执行
一次 `foc_lsi_start LSI1`。bounded session 安全执行 PASS：449 控制拍后自动 COMPLETE，
192 点连续、无 fault/miss/overflow/contract/storage error，最大名义相电流 0.2155 A，完整
ISR WCET 11,078/12,750 cycles。随后额外 `foc_stop`，输出保持关闭，token=1/1。

离线 applied-voltage 拟合得到 1.740 mH，但六个脉冲对极差/均值 15.68%，且与 LCR 折算
1.113 mH 相差 56.29%。因此硬件执行通过，参数筛查拒绝升级，不修改 profile/revision。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `s5_0_run_0p2a_01.log` | 唯一一次启动的关键串口状态与 192 点原始窗 | `0A374B0ACFA818F8B9EBAA2F8BBA1E250668B719E13295AEE4FB942AF10FF493` |
| `s5_0_run_0p2a_01.csv` | 名义 SI 换算、command/applied voltage | `8066D25A7F35F279E93B8FFD2EF899719084652492DB83BCEC23B0DB628CD553` |
| `s5_0_run_0p2a_01.json` | 契约、安全、WCET、拟合与拒绝判定 | `620ED9AB642122A255E8419CD82232DCA4F6B6BBA0E7509DF1A0843C5FD36D23` |

详细边界见
[`EXP-B3 S5.0 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.0-0.2A一次性实机筛查.md)。

## S5.1 模型比例、实测噪声与 6×6 候选

ST MCSDK 参考工程的 0.33 Ω、1.53、0.0625 和 525/550 ns 与 FluxRT 名义模型一致；
4095/4096 口径差仅约 0.0244%，按官方比例重算仍为 1.740 mH。Rs/L 联合拟合得到
4.79 Ω/1.726 mH，说明换算常数或固定 Rs 不能解释与 LCR 相差 56.29%。

S5.0 cooldown U 相标准差为 1.761 count。按 1.76 count 重跑后，旧 4 拍/极性×6 对方案
最低分类率 93.33%，当前 Host 门 FAIL；6 拍/极性×6 对候选在同一代理下 9 个工况、每点
30 次全部分类正确，最大极差/均值 4.44%，Host 门 PASS。候选把 bias 从 120 拍缩到 24 拍，
raw 窗从 192 点缩到 120 点，有功时间从 14 ms 缩到 8 ms；仍不构成硬件许可。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `s5_1_model_sensitivity.json` | 官方比例、死区、器件压降、联合 Rs/L 与短窗噪声审查 | `40580206D471A39DCA87CB3793F05EF18E04F4014A01CFA44EA4B1B4270529B9` |
| `lsi_host_identifiability_measured_noise.json` | 1.76 count 下旧 4×6 方案 FAIL | `830E3BDFE5C5CE70C5DB9D76E0DBC90899FC3733F6BC4A10609A6EC41945CC11` |
| `lsi_host_identifiability_redesign_6x6_measured_noise.json` | 1.76 count 下 6×6 候选 PASS | `AB378668CF58822072E3CF0E0CE6B9B3FB48328C75969C8C603651A8BB8F411D` |

详细边界见
[`EXP-B3 S5.1 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.1-模型比例与实测噪声审查.md)。

## S5.2 目标代码状态

应用层默认值已改为 24 拍 bias + 6 拍/极性×6 对 + 24 拍 cooldown；Host/全回归和四档
交叉构建通过。新 Identification BIN/HEX 为
`A0897315497D9AF0DF638E55A053CCAF55A747C12906EEC772FA6DB743228566` /
`744DBF78112FDA26274D06B64331F99DED54FB4B7BBDD4407E5E4B716BDEEEFB`，但尚未烧录。
板上仍是旧 S4.9，下一门是主功率关闭后的 S5.3 无功率验证，不允许直接带电启动。

详见
[`EXP-B3 S5.2 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.2-6x6序列目标整改门.md)。

## S5.3 无功率板端门

用户确认主功率关闭后，新 Identification HEX 完成 SWD download/verify/reset。固件自报
`identification + s`，配置回读为 `12000/256/24/6/6/24`；错误口令、低 Vbus 正确口令和
普通 motor start 均拒绝，token 始终 0/0。独立安全窗 256 点 sequence/tick 连续，Vbus
raw 0～5、CCR 全 0、ARR=7083、flags=128，0 overflow/contract/storage error。最终
IDLE/OFF、duty/step/error/miss=0。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `s5_3_unpowered_gate.json` | 固件身份、配置、拒绝门、256 点摘要和最终安全状态 | `9B9D5C2DF255B90BB8BA4FB78FB7B1850688D4F2E39FCB0313526FB1924BD2F0` |

S5.3 不证明 120 点 active 窗、377 拍会话或 active WCET，再次带电权限仍未授予。详见
[`EXP-B3 S5.3 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.3-6x6无功率板端门.md)。

## S5.4 锁转子 6×6 一次性实机门

用户确认转子已固定、主电源接通后，沿用同一硬件会话已确认的 12.3 V/2 A/可立即断电
条件，只执行一次 `LSI1`，不自动重试。377 拍后自动 COMPLETE，120 点严格为
24 bias + 36 positive + 36 negative + 24 cooldown；无 fault/miss/capture error，最大名义
相电流 0.2330 A，active ISR WCET 11,053/12,750 cycles，随后两次显式停机。

6 个独立脉冲对为 1.886～2.016 mH，极差/均值 6.73%，相对旧窗的 15.68% 明显改善；
但固定 Rs 总体值 1.950 mH 仍比 LCR 相值高 75.15%。Rs/L 联合、常量桥压降仿射模型和
理想中心对齐 PWM 子周期积分都不能把结果拉回 1.1133 mH。因此硬件执行与窗内重复性通过，
参数升级拒绝；下一次 powered run 未授权。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `s5_4_locked_6x6_run_01.log` | 一次启动的状态、120 点 raw 与最终停机 | `7AE6866D67F704DA5DF489BF11F952FF2FAF5D7158581D9965DE44539609BEEE` |
| `s5_4_locked_6x6_run_01.csv` | 名义 SI 换算与 command/applied voltage | `2E1A6AC5E64E58EA4229FFE45DD864D8737B250A2531A9EC05EFBE706CE5C990` |
| `s5_4_locked_6x6_run_01.json` | 契约、安全、WCET、拟合与拒绝判定 | `4CA0239934A343ED1E225AC7976B7F2E454A24BDA1996305105789D01A83F513` |
| `s5_4_locked_6x6_model_audit.json` | 比例、Rs/L、常量压降与子周期 PWM 审查 | `951451CD90F794D10571CEE62EDE74EFA699BD5F286E16F1703CF084892D6212` |

详细边界见
[`EXP-B3 S5.4 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.4-锁转子6x6一次性实机门.md)。

## S5.5A 锁转子 LCR 同角度重复性

在转子保持固定、12.3 V 主电源未连接的条件下，用户使用 LT1 的 1 kHz / 0.6 V 显示档
按正序、反序、正序完成三轮 Q/R/L 测量。同一线对 L 极差/均值最大仅 0.375%，但 VU
均值 2.8207 mH、WV/WU 约 2.13 mH，线对间极差/均值 29.12%。这证明线对差异可重复，
不证明单一角度已经识别 `Ld/Lq`；动态 1.950 mH 相对简单线间均值除二仍高 65.05%。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `s5_5a_lcr_locked_angle_a_repeat.csv` | 三轮原始 Q/R/L、测量顺序和无主电源条件 | `CA0381A1090E25706DE920C9B113273DC68D9CA2B15C31E9DD2EF94399F84FF8` |
| `s5_5a_lcr_locked_angle_a_repeat.json` | 同线对重复性、跨线对差异和 S5.4 对比 | `573426AA3F07001F12EFE57AE8D367D7142B3837F5FB0F98DD27B9827025AC3B` |

下一门为 S5.5B 无功率机械角度扫描；任何 powered repeat 仍未授权。详见
[`EXP-B3 S5.5A 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.5A-锁转子LCR同角度重复性.md)。

## S5.5B LCR 手动角度粗扫

用户随后手动向多个方向转动电机，观察到线间 L 约 1.96～2.60 mH。该数据没有精确角度、
极值线对和逐点 Q/R，作为定性筛查证明转子位置影响具有工程量级；不要求继续手持逐 5°
测量。与 S5.5A 合并的观察包络为 1.96～2.8207 mH，但它不是 `Ld/Lq` 估计。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `s5_5b_lcr_manual_angle_sweep.json` | 用户报告的粗扫范围、S5.5A 合并包络和延期判定 | `88689B082017913F3B80271320534DC08DF290AF2708033603B956D130AB8018` |

下一门为 S5.5C 离线动态电流/电压真值设计和参数范围仿真；powered repeat 仍未授权。详见
[`EXP-B3 S5.5B 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.5B-LCR手动角度粗扫.md)。

## S5.5C 动态真值与参数包络

离线精确离散 RL 回归把 S5.4 名义结果复算为1.9503 mH。电流/电压各±5%时拟合仍为
1.9503～2.0402 mH；把相对比例从0.1扫到3.0，最低也没有进入0.98～1.4103 mH诊断包络，
因此简单乘法比例不是冲突的单一解释。Rust在5个L点×参考/匹配模型共10个10 s闭环场景
全部完成，峰值相电流最大0.806 A；这是PC功能包络，不是参数或硬件证明。

| 文件 | 内容 | SHA-256 |
|---|---|---|
| `s5_5c_truth_and_uncertainty_envelope.json` | 比例区分、10组Rust包络和动态真值最小测量契约 | `9C4308DBD47D499EE34FB43DDFD734AB525DB5D400EB91CAAE3B3C550A4B5A22` |

A26下一次powered门需要独立动态相电流、差分PWM电压和同时间基时序；现有仪器不足以完成
这一级真值。参数更新和powered repeat仍拒绝。详见
[`EXP-B3 S5.5C 报告`](../../../../docs/performance/2026-09-27-EXP-B3-S5.5C-动态真值与参数包络.md)。
