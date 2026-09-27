# EXP-B1 实测：反电动势（BEMF）测量与 Ke 量纲判定（v5，三窗重复性收口版）

> 工程定位：FluxRT `docs/experiments/` 临时实测文档（实验对象是 FluxRT 固件 + 实物电机）
> 对应任务：**A26 参数辨识的磁链量纲判定**（`profiles/identification/evidence/motor-profiler-20260925/README.md` 遗留问题）
> 测量方法：**频率法（V/f 法）主基准**——磁链由同一采集窗的幅值与电频率直接解出，不依赖指令转速
> 风险等级：中（电机会转动，空载、低压、限流）
> 预计耗时：准备 15 分钟 + 实测 10 分钟 + 分析 10 分钟
> 版本：v5（2026-09-27 完成三次有效窗与未审批 rev7 候选；分析器 v5 配套）

---

## 0. 本次实测要回答的三个问题

| # | 问题 | 判定方法 | 状态 |
|---|---|---|---|
| 1 | **Profiler 的 `SC_KE` 是线电压 RMS 还是相电压 RMS 口径？** | 实测 Ke 两种口径换算 → 哪个与 SC_KE 吻合（两者差 √3 = 1.732） | **ambiguous**：三窗均无法匹配；Profiler 快照未完成且自身离散 43.2%，禁止 convert |
| 2 | **工程基线磁链 `0.005529 Wb`（SI 口径）的绝对值是否正确？** | 频率法实测磁链与基线同口径对比（factor ≈ 1 即正确） | **三窗支持**：均值 5.376 mWb、偏差 −2.77%、极差/均值 3.41% |
| 3 | Profiler 读回的 Rs 是否可信？ | 万用表 DC + 表笔补偿 + 温度归一化 | **EXP-B2 已测得当前温度 `Rs=4.9667 Ω`**；与 profiler 4.875 Ω 仅差 1.9%，不再判定 profiler 明显低读；待补测温度后最终裁定（详见 EXP-B2 v7） |

> EXP-B2 当前结论：按既有星形模型换算，当前温度 DC `Rs=4.9667 Ω`（已扣 0.1 Ω 表笔，待记录温度）；1 kHz 小信号 `Ls=1.114 mH`；内部接法与星形模型一致但仅靠端子阻抗不能唯一证明。本实验主要继续解决 **Ke 口径与磁链绝对值**。

**背景**：Motor Profiler 修复后（2026-09-25）读回 `SC_KE` 四次均值 3.752，但按官方单位（线电压 RMS）换算磁链得 `0.021166 Wb`，与工程基线 `0.005529 Wb` 差 **3.83 倍**。**预分析（2026-09-25，算术验证）已确认这 3.83 倍里大部分是口径混淆**：

- 固件基线 `0.005529 Wb` 是**教科书 SI 口径**（每电弧度，`ψ = V_ph_pk / ω_e`），与数据库 Ke=4.964 Vll_rms/kRPM 经标准电机理论精确自洽（GBM2804H，p=7）；
- `motor_profiler_convert.py` 的换算除以**机械**角速度，得到的是 `p × ψ`（含极对数折叠），两者相差 **p=7 倍**——这正是交接表"差约 7 倍"的来源；`3.83 ≈ 7 × 0.547`（0.547 是 run1 未完成 Ke 与数据库之比）；
- 因此 SC_KE 若按线 RMS 解释，run3（4.337）对应的 SI 磁链只比基线低 **−12.6%**；若按相 RMS 解释，run1（2.715）对应值只比基线低 **−5.3%**——都不是 3.83 倍量级的错误。

本实验的作用从"排除 3.83 倍错误"收敛为：**裁定 SC_KE 的口径（线/相 RMS，差 √3）+ 用独立测量确认基线磁链的绝对值**。

### 0.1 2026-09-27 实机结果与重复性结论

固件为 CM4.4 Diagnostic，母线 12.30 V、外部电源限流 2 A、电机空载。烧录文件
`cmake-build/fluxrt.hex` SHA-256 为
`9292869F76861B18B93F5ACC1FABE435EF5213A60DCBDC2BDBA1016719104AD9`。
各次采集均由脚本异常路径保护，结束后再次执行 `foc_stop`、`closedloop 0`，并确认
平台 flags=`0004231f`（无 armed/fault/trip）、duty=`500/500/500`、默认时序已恢复
`582 rpm / 2000 ms / 5000 ms`。

| 窗口 | 请求/实际 rpm | 停机到 arm | 联合拟合残差 | ψ_SI | 结果 |
|---|---:|---:|---:|---:|---|
| `bemf_1000rpm_run1` | 1000 / 249.8 | 123.9 ms | 1.36% | 7.452 mWb | 旧脚本未同步开环终速且等待回显，跟踪偏差 −75.0%，拒绝 |
| `bemf_1000rpm_run2` | 1000 / 149.8 | 0.207 ms | 9.80% | 4.940 mWb | 1000 rpm 开环失步，频率/幅值不平衡，拒绝 |
| `bemf_0582rpm_run3` | 582 / 537.3 | 0.165 ms | 3.57% | **5.418 mWb** | 测量门通过；较基线 5.529 mWb 低 2.02% |
| `bemf_0582rpm_run4` | 582 / 564.2 | 0.144 ms | 3.84% | **5.447 mWb** | 测量门通过 |
| `bemf_0582rpm_run5` | 582 / 547.7 | 0.148 ms | 4.12% | **5.263 mWb** | 测量门通过 |

三次有效窗的 `ψ_SI` 均值为 **5.375687 mWb**，极差/均值 **3.4139%**、样本变异
系数 1.8354%，通过 `<10%` 重复性门。均值相对工程基线 5.529026 mWb 低 2.773%，
因此只能表述为“在名义分压精度内与基线一致”；A19 逐板增益标定后仍要重算绝对值。

三次 `Ke_line` 均值为 4.826 Vll_rms/kRPM，但相对 Profiler `SC_KE=3.75248` 的
口径判定均为 `ambiguous`。这不否定直接 BEMF 测量，而是说明未完成、四次离散达
43.2% 的 `SC_KE` 快照不适合作为换算输入。rev7 因而直接使用三窗 SI 磁链均值，
保持 `approvals=false`、`closed_loop_enable=0`，且没有集成进固件。

证据目录：`profiles/identification/evidence/bemf-20260927/`。

---

## 1. 测量原理：频率法（V/f 法）

### 1.1 物理基础

PMSM 转子断电（栅极关断）后靠惯性自由旋转时，转子磁场扫过定子绕组产生反电动势。对同步电机**无转差概念**，存在两个严格物理关系：

```text
电频率与转速（同步关系，任何时刻严格成立）：
    f_e = p × n / 60                    [Hz]     p=极对数, n=机械转速[rpm]

相电压峰值与磁链（法拉第定律，正弦 BEMF）：
    V_ph_pk = ψ × ω_e = ψ × 2π × f_e   [V]      ψ=每电弧度磁链[Wb]（SI 口径）
```

两式来自**同一个采集窗**（256 拍 @ 12 kHz = 21.3 ms）时，联立消去转速：

```text
┌─────────────────────────────────────────────────────────┐
│  ψ_SI = V_ph_pk / (2π × f_e)                            │
│                                                         │
│  转速 n 完全不出现：磁链直接从波形本身的幅值和频率解出。  │
└─────────────────────────────────────────────────────────┘
```

这是测功机行业"发电机法"和 VESC 磁链辨识的标准做法。**它不需要任何转速计**——频率就是转速的精确表达（`n = 60·f_e/p`），而频率从波形里测，比任何外置转速测量都直接。

### 1.2 为什么用频率而不用指令转速做基准

v1 方案用 `foc_start 1000` 的指令转速做 Ke 基准，频率只做交叉检查。这有两处固有偏差，频率法全部消除：

| 偏差源 | 机理 | 量级 | 频率法 |
|---|---|---:|---|
| 开环强拖跟踪偏差 | 转子实际转速 ≠ 强拖角速度（负载角 + 可能的猎振） | 1~3% | ✅ 消除 |
| 停机滑行衰减 | `foc_stop` 后串口往返 ~50 ms，电机已在减速，指令值过期 | 1~2% | ✅ 消除（f_e 与幅值同窗同时测） |
| 分压模型误差 | 名义 10k/2.2k，未逐板标定 | 2~5% | ❌ 仍在（只能靠 A19 标定） |

**自测验证（合成波形，2026-09-27）**：模拟"指令 1000 rpm 但实际只有 900 rpm"的场景，频率法给出磁链 factor = 0.9998（旧指令基准法会偏 10%），且 `validity` 正确降级提示工况异常——**结果不受工况污染，同时工况异常会被暴露**，两者兼得。

### 1.3 频率和幅值怎么测：线间差分 + 平衡三线联合拟合

```text
三个端子 ADC 测的是对板地电压，中性点浮动时会带未知公共模。
先构造三个线间序列：U-V、V-W、W-U，公共模在相减时消失。

辅助诊断仍对每个线间序列找上升沿过零点并做线性插值：
        t_cross = (i-1 + |v[i-1]| / (|v[i-1]|+|v[i]|)) / 12000
        f_cross = (过零点数-1) / (末过零时刻 - 首过零时刻)

主结果把三条线一起拟合：
  V_uv = A·sin(ωt+30°+φ)  + C_uv
  V_vw = A·sin(ωt-90°+φ)  + C_vw
  V_wu = A·sin(ωt+150°+φ) + C_wu

三条线共用频率 ω、幅值 A 和未知初相 φ，只允许各自有独立 DC 偏置。
在 10..200 Hz 内粗扫并细化，使三线总归一化残差最小；
V_ll_pk=A，V_ph_pk=A/√3。
```

数据量：582 rpm 时 21.3 ms 只有约 1.45 个电周期，单线过零法经常不足两个上升沿；联合拟合利用三条线的固定相位关系，仍能求出共同频率/幅值，并用总残差拒绝非正弦数据。线间差分是主结果；对板地三相半峰峰值只保留为诊断，不能直接当作相反电动势。

### 1.4 Ke 与口径判定

由频率反推实测转速 `n = 60·f_e/p`，然后：

```text
Ke(线RMS)  = V_ll_rms  / (n/1000)        其中 V_ll_rms  = V_ph_pk / √2 × √3
Ke(相RMS)  = V_ph_rms  / (n/1000)        其中 V_ph_rms  = V_ph_pk / √2

判定：ratio_line = Ke(线RMS)/SC_KE 与 ratio_phase = Ke(相RMS)/SC_KE
      哪个更接近 1，SC_KE 就是哪个口径。两种口径差 √3=1.732，
      总误差 <10%，判定可靠。
```

### 1.5 磁链口径速查（勿混用）

| 口径 | 定义 | 谁在用 | 换算 |
|---|---|---|---|
| **教科书 SI** | `ψ = V_ph_pk / ω_e`（每**电**弧度） | **本工程固件**（`params.rs` 基线 `0.005529 Wb`） | 本实验输出 `flux_si_wb` |
| MCSDK 折叠 | `V_ph_pk / ω_mech`（每**机械**弧度） | `motor_profiler_convert.py`、MCSDK Workbench | = **p × SI**（GBM2804H 差 7 倍） |

**操作规则**：候选 JSON / 固件里手填磁链一律用 SI 口径（`flux_si_wb`）；convert 工具的磁链输出除以 p 才是固件值；任何"差整数倍/√3 倍"的磁链告警，先查口径再怀疑数值。

---

## 2. 精度预算（判 √3 口径差异绰绰有余）

| 误差源 | @582 rpm | 频率法是否受影响 | 说明 |
|---|---:|---|---|
| 频率/幅值（三线联合拟合） | 由残差与重复窗约束 | 是（这就是基准） | 21.3 ms 约 1.45 周期；单窗不足以定案，必须三次重复 |
| 窗口内幅值 droop | <1% | 是 | 机械时间常数（秒级）>> 窗口 |
| 分压模型（名义 10k/2.2k） | ~2-5% | 是 | **未逐板标定**（A19 待完成）——绝对精度的主导项 |
| 开环跟踪偏差 | 1~3% | **否（已消除）** | v1 方案的误差源 |
| 滑行衰减 | 1~2% | **否（已消除）** | v1 方案的误差源 |

**合成精度**：磁链/Ke 绝对精度 ~3-6%（主导项是分压模型）。判定目标中：√3=1.73 倍口径差异 → 远大于误差，可靠；基线绝对值确认 → 与分压误差同量级，结论应表述为"在名义分压精度内与基线一致"。

**转速选择**：1000 rpm 在本台架两次实测均未同步跟随，因此不能为了增加窗口周期数而强行提速。脚本默认改为已验证可启动的 `--rpm 582`；通过联合拟合处理短窗，再以三次重复性补足单窗信息量。

---

## 3. 设备与前置条件

- [ ] NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T 电机（**空载、可自由旋转、无桨叶**）
- [ ] 可调限流电源：12.0~12.6 V，限流 2 A
- [ ] **板上固件必须是 FluxRT Diagnostic**（不是 Motor_Profiler！）——验证方法见步骤 0
- [ ] USB 连接，串口 COM6 @ 115200（ST-Link VCP）
- [ ] PC 上已装 Python + pyserial（`pip install pyserial`）
- [ ] **Motor Pilot / 其它串口程序完全退出**（否则抢串口）
- [ ] 电机周边无松动物体；手边有随时可断电的手段（电源开关或拔线）

---

## 4. 实测步骤

### 步骤 0：确认板上固件是 FluxRT Diagnostic

打开串口终端（115200），按复位键，看启动横幅：

```text
必须看到：FBOOT,p=diagnostic,o=s
如果看到 ST Motor Profiler 相关输出（或无 FluxRT 横幅）：
    → 需要重烧 FluxRT Diagnostic 固件：
      cd E:\File\RT-Thread\projects\FluxRT
      .\build.ps1 -BuildOnly          # 确认能构建
      # 然后用 STM32_Programmer_CLI 或 IDE 烧录 cmake-build\fluxrt.hex
```

> 2026-09-27 本机已重新构建 CM4.4 Diagnostic：`fluxrt.hex` SHA-256 前缀 `9292869F`，`fluxrt.bin` SHA-256 前缀 `50A977E3`。本次构建只证明 S3 目标构建通过；当前 Windows 未枚举到 ST-LINK/VCP，尚不能确认板上实际镜像或烧录该文件。

### 步骤 1：预检（不转电机）

```powershell
cd E:\File\RT-Thread\projects\FluxRT
python -c "import serial; print(serial.__version__)"   # 确认 pyserial
```

串口终端里执行 `foc_status`，确认：
- `FBOOT` 状态行各状态码为 0（OK）；
- `FSTAT` 显示 `disabled`；
- `FMON` 的 Vbus 在 12000 mV 左右（母线已上电）。

### 步骤 2：采集（脚本自动完成，时序已匹配 CM4.4）

```powershell
python simulation\capture_bemf_measurement.py `
    --port COM6 --rpm 582 `
    --output profiles\identification\evidence\bemf-20260927\bemf_0582rpm_run4.csv `
    --allow-motor-run
```

脚本流程（全自动，约 14 秒）：先读取并保存当前开环终速/时序，显式 `closedloop 0`，把 `startup/align/ramp` 同步到本次请求，随后 `foc_start 582` 强拖（对齐 2 s + 升速 5 s + 保持 3 s）→ 记录一次 `foc_status` → 把 `foc_stop` 与 `foc_phase_capture start on` 连续排入 Shell → 抓 21.3 ms 窗口 → dump 256 个样本 → 恢复原开环终速/时序。异常路径也会尽力停机、关闭闭环并恢复配置。

默认时序可通过 `--alignment-s/--ramp-s/--hold-s` 显式修改，但必须与当前固件配置一致；脚本会把三项时序和“停机命令到采集 arm”的主机侧延迟写入同名 `.json`，禁止再使用旧版 1 s + 1.2 s 等待值。

**注意**：`--rpm 582` 同时设置 `foc_start` 目标和临时的开环终速，并作为跟踪诊断基准；**不参与磁链/Ke 的计算**（频率与幅值由波形自身联合解出）。

**你会看到/听到**：电机先短促定位（对齐），然后加速旋转约 3 秒，然后突然停止（惯性滑行几圈后停）。脚本最后打印 JSON 摘要。

**安全观察点（人工确认）**：
- [ ] 电机加速阶段无异响、无异常温升
- [ ] 停机后电机确实自由滑行停止（没有被抱死）
- [ ] 脚本结束后串口打印 `foc_status`，`FOC f=` 标志位无 FAULT

**失败排查**：

| 现象 | 原因 | 处理 |
|---|---|---|
| `foc_start was refused` | 母线不在 7~18 V 窗口 / 平台诊断位未齐 | 看 `FSTART,refused` 后的状态码，检查电源 |
| `phase capture was refused` | 采集器认为电机仍 armed（停机竞态） | 重跑一次；若仍失败，手动 `foc_stop` 后再跑 |
| 样本数不是 256 | dump 被串口流量截断 | 重跑；确认没有其它程序占用串口 |
| 波形全是平的（无正弦） | 分压没开 / 转速太低 / 电机没转 | 检查 dump 里 `FPV_MODEL` 的 divider=on；提高 rpm 重试 |
| 分析输出 `degraded-no-frequency` | 转速过低导致过零点不足 | 提高 `--rpm` 重测（见第 2 章转速下限） |

### 步骤 3：重复 3 次（重复性要求）

同一 rpm 再跑 2 次（run2/run3），输出文件名递增：

```powershell
python simulation\capture_bemf_measurement.py --port COM6 --rpm 582 `
    --output ...\bemf_0582rpm_run4.csv --allow-motor-run
python simulation\capture_bemf_measurement.py --port COM6 --rpm 582 `
    --output ...\bemf_0582rpm_run5.csv --allow-motor-run
```

### 步骤 4：分析（v5 平衡三线联合拟合，裁定口径）

```powershell
python simulation\analyze_bemf_measurement.py `
    profiles\identification\evidence\bemf-20260927\bemf_0582rpm_run3.csv `
    --rpm 582 --sc-ke 3.75248
```

对三次有效 582 rpm 采集分别运行。**v5 输出字段导读**：

| 字段（JSON 路径） | 含义 | 判读 |
|---|---|---|
| `frequency_method.electrical_hz_balanced_fit` | 平衡三线联合拟合的共同电频率 | 582 rpm 理论值为 67.9 Hz；停机滑行会略低 |
| `frequency_method.balanced_fit_normalized_residual` | 三线联合拟合总残差/总方差 | ≤0.15 才通过测量门 |
| `frequency_method.electrical_hz_by_line_zero_crossing` | 单线过零频率 | 仅辅助诊断；短窗允许某条为 null |
| `line_to_line_amplitude.phase_peak_v_from_line` | 联合拟合线峰值除以 √3 | 磁链幅值输入 |
| `line_to_line_amplitude.line_peak_spread_pct` | 各线独立拟合幅值离散 | >35% 会降级并要求复测 |
| `flux_frequency_method.flux_si_wb` | **频率法磁链（SI 口径）** | 核心结果，可直接入候选 |
| `diagnostics.rpm_measured_from_frequency` | 频率反推实测转速 | —— |
| `diagnostics.tracking_deviation_pct` | 实测 vs 指令转速偏差 | **诊断量**：>10% 触发降级 |
| `ke_candidates.ke_base_rpm` | Ke 用的转速基准 | 必须显示 `measured-from-frequency` |
| `ke_candidates.ke_line_rms_per_krpm` / `ke_phase_rms_per_krpm` | 两种口径的 Ke | 判口径用 |
| `decision.sc_ke_matches` | 口径判定 | `ke-phph-rms`、`ke-phase-rms` 或 `ambiguous` |
| `decision.flux_vs_baseline_factor` | SI 对 SI 磁链比值 | ≈1 基线正确 |
| `validity` | 有效性 | `ok` 才可采信裁定（见下） |

**validity 主要取值**：

| 取值 | 含义 | 动作 |
|---|---|---|
| `ok` | 频率测出且跟踪偏差 <10% | 采信裁定 |
| `degraded-no-frequency` | 过零点不足，频率测不出 | **不输出磁链**；提高 rpm 重测 |
| `degraded-balanced-fit-residual-retest-advised` | 联合拟合残差 >15% | 波形不符合平衡三相正弦模型，复测 |
| `degraded-line-frequency-imbalance-retest-advised` | 三条线间频率离散 >5% | 检查采样/噪声并复测，不采信本次裁定 |
| `degraded-line-amplitude-imbalance-retest-advised` | 三条线间幅值离散 >35% | 检查接线、分压通道和电机并复测 |
| `degraded-tracking-anomaly-retest-advised` | 频率法结果本身仍可信（不依赖指令值），但跟踪偏差 >10% 说明工况异常（滑行过久/开环失步） | 磁链值可记录；**口径裁定建议复测一次**再定稿 |
| `degraded-sc-ke-convention-ambiguous` | 测量门通过，但两种 `SC_KE` 口径都未落入 ±15% 或优劣间隔不足 10% | 磁链可进入重复性统计；禁止执行 convert/审批 `SC_KE` |

### 步骤 5：填表（本节是交付物，实测后填写）

#### 5.1 条件记录

| 项目 | 值 |
|---|---|
| 日期/操作人 | ____ |
| 固件版本与 SHA-256 | ____ |
| 母线电压（实测 Vbus 打印值） | ____ V |
| 电源限流 | ____ A |
| 室温/电机温度（起始） | ____ °C |
| 3 次运行间的间隔 | ____ |

#### 5.2 原始数据（v5 分析器输出，三次有效 582 rpm 运行）

| 量 | run3 | run4 | run5 | 说明 |
|---|---:|---:|---:|---|
| f_e 联合拟合 [Hz] | 62.680 | 65.820 | 63.900 | `electrical_hz_balanced_fit` |
| 联合拟合归一化残差 | 0.0357 | 0.0384 | 0.0412 | 应 ≤0.15 |
| V_ph_peak [V] | 2.134 | 2.252 | 2.113 | `phase_peak_v_from_line` |
| 三线独立幅值离散 [%] | 25.08 | 25.25 | 22.60 | `line_peak_spread_pct`，应 ≤35% |
| n 实测（频率反推）[rpm] | 537.26 | 564.17 | 547.71 | `rpm_measured_from_frequency` |
| 跟踪偏差 [%] | −7.69 | −3.06 | −5.89 | 诊断量，应 <10% |
| **ψ_SI 频率法 [mWb]** | **5.418** | **5.447** | **5.263** | **`flux_si_wb` × 1000，核心结果** |
| Ke(线RMS) | 4.864 | 4.890 | 4.725 | 均值 4.826 |
| Ke(相RMS) | 2.808 | 2.823 | 2.728 | 均值 2.786 |
| measurement_validity | `ok` | `ok` | `ok` | 三次均通过 |
| SC_KE 口径 | `ambiguous` | `ambiguous` | `ambiguous` | 不再用 SC_KE 转换 |
| **三次 ψ 极差/均值** | — | — | **3.41%** | **PASS（门限 <10%）** |

#### 5.3 判定结论

| 问题 | 结论 | 依据 |
|---|---|---|
| SC_KE 口径 | **ambiguous，不采用 SC_KE 换算** | 三窗均未通过口径门；Profiler 快照自身极差/均值 43.2% |
| 实测磁链 [Wb]（SI 口径） | **0.005375687** | 三次 `flux_si_wb` 均值 |
| 基线磁链 0.005529 Wb 判定 | **名义分压精度内一致，均值偏差 −2.77%** | 三次极差/均值 3.41% |
| Ke 是否可采用 | **直接 BEMF 值可进未审批候选；不可据此审批** | 三次重复性过门，但逐板分压标定/负载与温升证据仍缺 |

#### 5.4 后续动作

- 已用三次均值生成 `profiles/candidates/rev7-gbm2804h-rsl-flux-screening.json`；它是
  ABI V11 的**未审批离线候选**，不是固件默认值。
- PC 仿真已完成：正/反转空载功能通过，但轻载和死区补偿工况结果混合；rev7 性能
  晋级门为 `inconclusive`，**暂不安排实机 A/B**。
- 先补 `Ls(I)`：1.114 mH 是 1 kHz 小信号值，不能视为 0.8 A 工作点值；工程目前
  没有经过评审的 MCU 脉冲辨识入口，不临时拼接功率脉冲上板。
- A19 逐板增益完成后，用新 `uV/count` 对三个原始 CSV 重分析并更新候选 revision/CRC。
- `motor_profiler_convert.py` 的磁链是 MCSDK 口径（= p × SI）；rev7 直接使用
  `flux_si_wb`，没有使用 `SC_KE` 或 convert 输出。

---

## 5. 证据归档要求

- [x] 两个无效 1000 rpm 窗与三个有效 582 rpm 窗的 CSV/log/metadata/analysis 均已保存
- [x] 全部放入 `profiles/identification/evidence/bemf-20260927/`
- [x] 三次 582 rpm 磁链极差/均值 3.41%，重复性门通过
- [x] `docs/工程操作日志.md` 已补 `LOG-20260927-006`
- [x] 已同步更新 Motor Profiler README 的历史磁链警告
- [x] rev6/rev7 PC A/B 已归档；rev7 性能晋级门仍为 `inconclusive`

## 6. 边界与不宣称

- 分压模型是名义值（未逐板标定），**本次磁链数值的绝对精度 ~3-6%**，只用于口径判定（√3 差异）与量级确认；最终磁链值要等 A19 逐板标定后精修。
- 频率法消除了转速基准误差，但**不提升绝对精度**——幅值链（分压模型）仍是主导误差项。
- 本实验只产生 rev7 **未审批离线候选**；没有改动 Rust/C 默认参数、Production Profile
  或板上固件，不能把候选生成误读为参数批准。
- 频率与幅值来自同一窗口，但每窗只有约 1.4 个电周期；三次重复已约束随机波动，
  仍不能替代逐板幅值标定、负载、温升和独立真值证据。

---

## 附录 A：Rs 交叉验证 → 当前温度值已取得

EXP-B2 已完成 LT1 交流测量与万用表 DC 交叉检查：

- 万用表线间 10.0/10.0/10.1 Ω，表笔短接 0.1 Ω；按星形模型得到当前温度 **`Rs=4.9667 Ω`**；
- profiler 4.875 Ω 与该值只差约 1.9%，旧版“低读 −4.7%”结论已撤销；
- `Ls` 的 Run B/C 复测平均为 **1.114 mH（1 kHz 小信号）**；
- 尚需记录测量温度，才能决定是否换算到 20℃参考值。

完整数据、判据与后续动作见 [EXP-B2_LCR实测与Rs定案.md](EXP-B2_LCR实测与Rs定案.md) v7。本实验不再重复电阻测量，只补记环境/绕组温度。

## 附录 B：命令速查

```text
串口手动操作（备用，脚本失败时人工复现）：
  foc_cfg startup 582   # 手工复现时必须同步开环终速
  foc_start 582         # 强拖到 582 rpm（对齐+升速+保持）
  foc_stop               # 关栅极，BEMF 出现在端子
  foc_phase_capture start on    # arm 分压 ON 固定窗（256 拍自动完成）
  foc_phase_capture status      # 确认 complete 256/256
  foc_phase_capture dump        # 打印 256 样本
  foc_status                    # 状态/故障检查

采集脚本（流程与频率法/转速法无关，两者共用）：
  python simulation\capture_bemf_measurement.py --port COM6 --rpm 582 --output <csv> --allow-motor-run
分析脚本（v5 平衡三线联合拟合）：
  python simulation\analyze_bemf_measurement.py <csv> --rpm 582 --sc-ke 3.75248
  # --rpm 仅作诊断基准；磁链/Ke 由波形频率解出，不受其影响
```

## 附录 C：频率法公式速查卡

```text
输入（同一 256 拍窗口，12 kHz，分压 ON，4469 uV/count）：
  V_uv[i] = (phase_u_raw[i] - phase_v_raw[i]) × 4469 uV
  V_vw[i] = (phase_v_raw[i] - phase_w_raw[i]) × 4469 uV
  V_wu[i] = (phase_w_raw[i] - phase_u_raw[i]) × 4469 uV

频率/幅值（三线联合拟合）：
  V_uv = A·sin(ωt+30°+φ)+C_uv
  V_vw = A·sin(ωt-90°+φ)+C_vw
  V_wu = A·sin(ωt+150°+φ)+C_wu
  在 10..200 Hz 搜索使三线总归一化残差最小的 f_e=ω/(2π)

磁链（SI 口径 = 固件 params.rs 口径）：
  V_ll_pk = A                                                   [V]
  V_ph_pk = V_ll_pk / √3                                         [V]
  ψ_SI    = V_ph_pk / (2π × f_e)                                 [Wb]

Ke（口径判定用）：
  n        = 60 × f_e / p                                        [rpm]
  Ke(相RMS) = (V_ph_pk/√2) / (n/1000)
  Ke(线RMS) = (V_ph_pk/√2 × √3) / (n/1000)
  SC_KE 口径门：最佳 ratio 与 1 的距离 ≤0.15，且两候选距离相差 ≥0.10；
  否则输出 ambiguous，不生成 convert 命令

口径换算（勿混用）：
  ψ_MCSDK = p × ψ_SI          （convert 工具输出的是 MCSDK 口径）
  Ke(线RMS) = √3 × Ke(相RMS)
```
