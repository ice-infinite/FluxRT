# 借鉴 ST MCSDK 与 VESC 的 FOC 工程修正路线

## 1. 文档目的

本文把 ST MCSDK 参考工程、VESC 7.01 开发源码和本工程现状转换成一份可执行的
改进路线。目标不是把本工程改造成 VESC，也不是机械复制某个参考项目，而是：

1. 保留当前 C 硬件层、Rust 控制层和纯算法层的可移植边界；
2. 先补齐测量、时序和参数证据，再增加补偿与高级算法；
3. 所有新功能都可关闭、可配置、可在 PC 上测试、可在实机上回退；
4. 最终让仿真、目标固件和实机使用同一套参数定义与控制公式；
5. 为以后更换国产 MCU 保留清晰的硬件适配层。

本文是实施路线，不代表所列功能已经完成。每个阶段只有通过对应验收门，才能进入
下一阶段或把功能改为默认开启。

每项整改的模块归属、目标目录、依赖规则和实施工作包见
[FOC 整改工程架构分配与代码落位规范](REMEDIATION_ARCHITECTURE_ALLOCATION.md)。

## 2. 当前经过核实的基线

| 项目 | 当前事实 | 不能误读为 |
|---|---|---|
| 控制器 | STM32G431RBT6，170 MHz，128 KiB Flash | 只要换算法就有充足资源 |
| PWM/控制环 | 当前均约 12 kHz | 已经实现高频 PWM、低频控制环解耦 |
| 闭环 WCET | 已记录最大约 10,083 cycles；软件截止 12,500 cycles | `pre/control/post` 三个独立最大值可以相加 |
| Flash | 当前重建后 `text + data = 127,052 B`，剩余约 4,020 B，即 3.07% | 只剩 989 B，或只改 `opt-level` 就一定解决 |
| 电机参数 | Rs=5.29 Ω、Ld=Lq=1.058 mH、磁链来自 Workbench | 参数已经由当前实物辨识 |
| 无感闭环 | 两轮 5 秒短时接管成功，上电默认仍关闭 | 已完成全工况或量产验证 |
| 死区补偿 | 仅 Host/Rust/Matlab 仿真有实验实现 | MCU 目标固件已经补偿 |
| ST 角度补偿 | MCSDK 有接口，但本参考工程两个补偿系数均为 0 | 当前 ST 例程实际使用了角度超前 |
| VESC 频率 | 通用默认 25 kHz PWM、通常约 12.5 kHz 完整控制；VESC6 常见 30/15 kHz | VESC 默认每 25 kHz 都执行完整浮点 FOC |
| PI Ki | 当前先恢复 ST 的连续时间 Ki，再乘本工程实际 `ts` | 存在必须把 30 kHz 改成 12 kHz 的 2.5 倍 bug |

基线资料：

- [工程架构与安全边界](ARCHITECTURE.md)
- [开环到无感闭环接管](CLOSED_LOOP_HANDOFF.md)
- [仿真与实机相关性验证](SIMULATION_HARDWARE_CORRELATION.md)
- [ST MCSDK 参考参数与 PC 仿真](ST_MCSDK_REFERENCE_AND_SIMULATION.md)
- [硬件数学加速与 CPU 回退](HARDWARE_MATH_ACCELERATION.md)

## 3. 借鉴边界

### 3.1 从 ST MCSDK 借鉴什么

- 与当前 NUCLEO-G431RB、IHM16M1 和 GBM2804H 套件一致的引脚、保护和启动流程；
- 三分流同步采样、定点量纲、状态机和可靠性检查方法；
- 成熟的故障关断顺序和硬件触发关系；
- 将参考参数作为可运行起点，而不是把数据库值当作最终整定值。

### 3.2 从 VESC 借鉴什么

- PWM 载波频率与完整 FOC 计算频率可以分开设计；
- 快环只做必要工作，缓存 sin/cos、Clarke 和预计算参数；
- 控制用原始调制，观测器用经过逆变器模型修正后的电压；
- 用滤波电流极性估计死区电压误差；
- 自动完成 Rs、Ls、Ld-Lq 等调试期参数辨识；
- 解耦前馈、温度补偿、HFI、MTPA 和弱磁全部做成可选配置。

### 3.3 不直接复制 VESC 源码

VESC `mcpwm_foc.c` 文件声明为 GPLv3-or-later。本工程根目录目前没有明确的统一许可
文件。除非以后明确选择兼容许可并完成合规检查，否则执行以下规则：

- 不粘贴或逐行翻译 VESC 函数；
- 只借鉴公开的控制理论、数据流和工程策略；
- 根据本工程量纲、相序和接口独立实现；
- 用本工程自己的测试向量、仿真和实机数据验证；
- 文档记录参考来源和参考提交，不把参考代码混入供应商或项目源码。

这条规则同样避免以后替换 MCU 时把 VESC 的硬件宏和 ChibiOS 依赖带进 Rust 算法层。

## 4. 目标架构

```text
                    RT-Thread 管理线程
          参数管理 / Shell / 标定 / 数据落盘 / 故障记录
                              │
                    只在停机状态修改配置
                              ▼
PWM 定时器 ──→ ADC 同步采样 ISR ──→ C 平台快照
  24/30 kHz       12/15/24 kHz          │
       │          独立可配置             ▼
       │                         Rust realtime bridge
       │                                │
       │              ┌─────────────────┼─────────────────┐
       │              ▼                 ▼                 ▼
       │        电流预处理一次      电压估计/补偿       观察器与接管
       │        Clarke + LPF       command/measured     SMO/BEMF/HFI
       │              │                 │                 │
       │              └──────────────┬──┴─────────────────┘
       │                             ▼
       │                  Id/Iq PI + 可选解耦前馈
       │                             │
       │                  可选圆限幅 / d轴优先限幅
       │                             │
       └──── CCR/SVPWM ←──── 逆 Park + SVPWM

PC/Matlab：使用相同 Rust bridge、相同配置结构和相同补偿算法
其他 MCU：只替换 board/ 与 foc/platform/<mcu>/
```

目标中的三个时间概念必须分开：

| 名称 | 含义 | 影响 |
|---|---|---|
| `pwm_frequency_hz` | 功率器件实际开关/载波频率 | 可闻噪声、死区归一化、开关损耗 |
| `control_frequency_hz` | Id/Iq 电流环和状态机更新频率 | PI `ts`、快环 WCET、控制带宽 |
| `observer_frequency_hz` | 观察器实际更新频率 | 观察器 `dt`、收敛与 CPU 负载 |

当前 `pwm_frequency_hz` 同时承担了前两种含义。实施频率解耦时必须拆开，不能只修改
TIM1 ARR，否则 PI、观察器、Rev-Up、trace 分频和死区补偿都会使用错误的时间基准。

## 5. 缺口与改进矩阵

| 优先级 | 当前缺口 | 改进内容 | 主要位置 | 完成证据 |
|---|---|---|---|---|
| P0 | 时序字段是独立最大值 | 增加同一拍完整 ISR WCET 和分段记录 | C 平台层 | DWT 与 GPIO 示波器结果一致 |
| P0 | Flash 仅余约 3.07% | 对比 `O3/s/z`，裁剪格式化、libm 和调试功能 | 构建系统 | size/map 报告和 WCET 同时通过 |
| P0 | 没有独立轴端真值 | 接编码器或外部测速仪 | 平台层/测试台 | 可计算真实角度和速度误差 |
| P1 | PWM 与控制环绑定 12 kHz | 评估 24 kHz PWM + 12 kHz 控制 | TIM1/ADC/ABI | 无漏采样、无超时、声学和波形改善 |
| P1 | 重复电流变换 | 每拍只做一次 Clarke 并共享 αβ | Rust control/bridge | 数值对拍一致，WCET 降低 |
| P1 | CORDIC 适配偏重 | 对比当前、打包 Q15、CPU 快速近似 | 数学后端 | 误差、平均时间、WCET 三项报告 |
| P1 | trace 与生产路径混合 | 生产构建可编译关闭，诊断构建保留 | Kconfig/C 平台 | 两类构建均可复现 |
| P2 | 目标固件无逆变器补偿 | 加入可关闭的死区/管压降电压模型 | Rust control/bridge | Host、MATLAB、低压实机逐级通过 |
| P2 | 观测器只用命令占空比 | 增加 `CommandModel/Measured/Hybrid` 电压源 | Rust observer/平台 | 切换无角度跳变，失效可回退 |
| P2 | 参数来自数据库 | 自动辨识 Rs、平均 L、Ld-Lq、磁链 | 标定状态机 | 重复测量误差和温升条件有记录 |
| P3 | 未使用板载 BEMF 网络 | 接入 PC0/PC1/PC3 对应 BEMF ADC | STM32G431 平台 | 示波器、ADC 和重构电压对拍 |
| P3 | 无交叉/BEMF 前馈 | 增加可选 dq 解耦 | Rust control | 仿真和实机扰动指标改善 |
| P3 | 限幅策略固定 | 保留圆限幅并增加可选 d 轴优先模式 | Rust control | 饱和区稳定且 anti-windup 正确 |
| P4 | 无延迟补偿 | 根据实测 ADC→PWM 延迟增加角度补偿 | Rust control/config | 编码器真值下相位误差下降 |
| P4 | 无绕组温度模型 | 温度有效时修正 Rs 和电流环 Ki | 管理层/Rust | 冷热机带宽和角度误差可比较 |
| P5 | 零速无感依赖 Rev-Up | 在实测凸极性充分后研究 HFI | Rust algorithm/control | 零速角度真值和温升测试通过 |
| P5 | MTPA/弱磁算法未接主链 | 按电机与工况需要逐项接入 | Rust control | 有明确收益且不破坏保护 |

## 6. 分阶段实施方案

### 阶段 0：冻结和补全基线证据

在改算法之前先保存可回退基线：

- [x] 保存当前 ELF/BIN/HEX/map、Git 提交和配置摘要；
- [x] 分别记录 trace 关闭、10 Hz、50 Hz 时的完整 ISR WCET；
- [x] WCET 从中断入口测到 ADC 标志清理后，包含正常路径诊断计数；
- [x] 每拍保存一组关联的 `total/pre/control/post`，禁止相加四个独立最大值；
- [x] 记录 `text/data/bss`，不使用 ELF 文件本身体积代替 Flash load；
- [x] 保留当前 12 kHz、`closed_loop_enable=0` 和全部安全阈值；
- [x] 用现有 PC、Clippy、交叉构建和 C 平台测试作为回归门；
- [x] 接入独立速度/角度真值之前，不提高默认闭环权限。

本轮 DWT、size/map 和实机结果见
[`performance/2026-09-23-a0-correlated-wcet-baseline.md`](performance/2026-09-23-a0-correlated-wcet-baseline.md)。
PA5 示波器对拍与独立轴端真值仍未完成，因此阶段 0 只完成软件和板端 DWT 基线，不能
标记为完整硬件验收。

建议新增自动生成的基线报告：

```text
docs/performance/<date>-<commit>-baseline.md
```

报告至少包含构建配置、固件大小、完整 ISR 最大值、控制路径最大值、运行场景、母线、
限流、温度、trace 状态和结果文件路径。

### 阶段 1：先优化现有 12 kHz 快环

这一阶段不改变控制公式和实机默认行为。

#### 1.1 构建配置 A/B

至少比较：

| Rust 配置 | 目的 |
|---|---|
| `opt-level=3, lto=true` | 当前性能基线 |
| `opt-level=s, lto=true` | 优先回收 Flash |
| `opt-level=z, lto=true` | 仅作为体积下界，不默认采用 |

每组必须同时记录 Flash 和 WCET。若只缩小 Flash、但使完整 ISR 超过安全预算，则不能
作为目标配置。

同时从 map 检查：

- 浮点 `printf`/`scanf` 是否被调试输出拉入；
- `libm` 中哪些函数由目标快环实际引用；
- 未启用的算法是否被 LTO/`--gc-sections` 删除；
- trace buffer、Shell 和测试入口能否通过 Kconfig 在生产配置裁剪；
- C 和 Rust 是否出现同功能的重复数学实现。

建议的阶段目标，不是当前已经达成的指标：

- bring-up 构建至少保留 8 KiB Flash；
- 后续生产候选构建至少保留约 10% Flash；
- trace 关闭时完整 ISR WCET 不高于 12 kHz 物理周期的 70%；
- 所有安全测试、仿真和短时实机基线不退化。

构建配置 A/B、map 审计和 Diagnostic/Production 分档已完成，结果见
[`performance/2026-09-23-a1-build-profile-matrix.md`](performance/2026-09-23-a1-build-profile-matrix.md)。
当前默认 Rust `s`；Production 候选剩余 29.04% Flash，且 trace 关闭时完整 ISR 为
8,160 cycles。阶段 1 尚未完成共享 Clarke、中间量复用和 CORDIC 事务优化，不能把本次
构建分档标记为整个阶段 1 完成。

#### 1.2 数据流复用

目标快环数据流：

```text
Ia/Ib/Ic
  → Clarke 一次
      ├→ 观察器
      ├→ Park 电流反馈
      └→ 电流极性滤波/死区模型
```

不要把另一个观察器后端的源码存在误计为运行时调用。后端枚举每拍只执行一个实现。
桥接层为接管初始化计算的附加 Clarke 只在状态切换时需要，不应强行并入稳态快环。

#### 1.3 数学后端基准

建立三种可替换后端：

1. 当前 G431 CORDIC；
2. 减少事务和换算的打包 Q15/Q31 CORDIC；
3. 可移植 CPU 快速近似，作为没有 CORDIC 的 MCU 基线。

比较项目：最大角度误差、最大模长误差、平均 cycles、最坏 cycles、超时行为和故障
计数。不能仅因为 ST 或 VESC 不轮询，就删除当前超时保护；若采用固定延迟读取，应先
证明寄存器时序，并保留启动自检或健康计数。

### 阶段 2：分离 PWM、控制环和观察器频率

建议优先评估：

```text
PWM：     24 kHz
电流环：  12 kHz
观察器：  12 kHz 起步
速度环：   1 kHz
```

这不是简单把 TIM1 改成 24 kHz。可评估两种调度：

#### 方案 A：每两个 PWM 周期触发一次完整 ADC/控制

- PWM CCR 保持两个载波周期；
- 用 TIM1 repetition counter、更新事件或专用触发关系生成 12 kHz 控制采样；
- 完整控制仍有约 83.3 μs 周期预算；
- 必须建模“采样到新占空比实际生效”的额外延迟。

这是当前约 10,083 cycles 快环更现实的起点。

#### 方案 B：24 kHz 都采样，每两拍执行一次完整控制

- 非控制拍只完成极短的采样、保护和状态保存；
- 完整控制拍仍必须在下一个 24 kHz 边界前结束；
- 170 MHz 下单个 PWM 周期只有约 7,083 cycles，当前实现尚不满足。

只有阶段 1 将完整控制路径降到足够范围后，才考虑方案 B。

无论使用哪种方案，都必须：

- 将 Rust 配置中的控制频率与 PWM 频率拆开；
- PI `ts` 使用控制频率；
- 观察器 `dt` 使用观察器实际更新频率；
- Rev-Up、接管超时和可靠性窗口使用真实时间，不按错误拍数推算；
- 死区归一化使用 PWM 频率，而不是控制频率；
- trace 分频根据实际生产采样源定义；
- PC plant 和 MATLAB 使用相同延迟与保持模型。

24 kHz 载波有机会降低当前 12 kHz 可闻音，但 12 kHz 占空比更新仍可能产生边带，
因此必须用示波器和声学实测确认，不能只凭载波频率判定噪声消失。

### 阶段 3：建立统一的逆变器电压估计层

当前控制器输出 `PwmCommand`，观察器再从上一拍 PWM 和母线电压重构 αβ 电压。目标
是增加独立的电压估计对象：

```rust
pub enum ObserverVoltageSource {
    CommandModel,
    PhaseVoltage,
    Hybrid,
}

pub struct InverterModelConfig {
    pub enabled: bool,
    pub dead_time_s: f32,
    pub compensation_gain: f32,
    pub current_zero_band_a: f32,
    pub current_sign_filter_alpha: f32,
    pub device_drop_v: f32,
    pub source: ObserverVoltageSource,
}
```

名称只是设计建议，实施时应遵守现有 ABI 命名和版本策略。

数据流必须保持：

```text
PI/SVPWM 输出 ───────────────→ 实际 PWM
      │
      └→ 逆变器模型/相电压融合 ─→ 观察器电压
```

实施顺序：

1. 把当前 Host-only 补偿移动到纯 Rust、可同时供 Host 与 target 编译的模块；
2. 默认 `enabled=false`，不改变现有目标行为；
3. 用低通后的 αβ/dq 电流构造三相极性，同时保留软零带；
4. 明确 duty、αβ 调制和实际伏特三种量纲，不能直接照搬 VESC 系数；
5. 在 PC 和 MATLAB 扫描死区、零带、滤波系数和补偿增益；
6. 增加管压降、母线纹波、ADC 量化和延迟模型；
7. 先只修正观察器电压，再单独评估 PWM 前馈；
8. 目标固件首次启用时限制为低压、限流、空载、短时间、可立即停机。

验收不能只看稳态转速偏差。至少比较：

- Id/Iq RMSE；
- 观察角对独立编码器角误差；
- 速度平均误差和标准差；
- 电流过零附近的抖动；
- 启动成功率；
- 补偿开启/关闭时的 WCET；
- 正反转一致性。

### 阶段 4：增加安全的自动参数辨识

辨识必须作为独立 `Calibration/Identification` 状态，不能与普通运行状态混用。

#### 4.1 Rs

- 转子锁定或保持定向；
- 使用两个或多个低电流平台，降低固定器件压降的影响；
- 记录母线、相电流、绕组/环境温度和持续时间；
- 达到限流、超时、偏差或驱动故障立即关断；
- 输出测量报告，不自动覆盖正式参数。

#### 4.2 平均 L、Ld 和 Lq

- 在受控角度下注入小幅高频电压；
- 由电流纹波估算电感；
- 扫描多个电角度并拟合 Ld/Lq；
- 先在 PC plant 加入已知 Ld≠Lq 的测试，验证辨识误差；
- 只有重复测量一致，才允许用于 HFI、MTPA 或观察器模型。

#### 4.3 磁链/Ke

- 可自由旋转且机械安全时，使用受控转速平台；
- 记录实际母线、电流、速度真值和温度；
- 由稳态电压方程和独立速度真值估算磁链；
- 禁止使用观察器自己的速度同时作为待验证真值。

建议参数记录包含：

```text
参数版本、板号、电机号、固件提交、日期、温度、母线、限流、
Rs、Ld、Lq、磁链、拟合误差、重复次数、是否人工批准
```

### 阶段 5：接入 IHM16M1 相电压/BEMF 采样

IHM16M1 原理图包含 `BEMF1/2/3` 分压与保护网络。当前平台只配置了电流、母线、
温度和电位器 ADC，尚未使用这些信号。

实施前先完成：

- [ ] 按原理图和 NUCLEO-G431RB 映射确认 PC0、PC1、PC3 对应关系；
- [ ] 测量分压比、二极管压降、输入 RC 和 ADC 源阻抗；
- [ ] 确认 ADC1/ADC2 的通道冲突、注入组/规则组资源和采样窗口；
- [ ] 示波器同时观察相端、分压端、PWM 和 ADC 触发；
- [ ] 先只采集 trace，不参与控制；
- [ ] 与 `duty × Vbus`、死区模型和三相重构结果离线对比；
- [ ] 增加开路、饱和、越界和不一致检测；
- [ ] 失效时自动回退 `CommandModel`，而不是继续使用坏数据。

目标采用 VESC 类似的混合策略，而不是无条件信任开关相电压：

```text
低速且相电压质量有效：命令方向 + 实测幅值，或经验证的直接重构
高速/采样质量不足：   补偿后的命令电压模型
停机：                 可使用实测 BEMF 做诊断，不作为运行控制证明
```

### 阶段 6：按需增加控制质量功能

#### 6.1 dq 解耦和 BEMF 前馈

提供枚举配置：

```text
Disabled / Cross / Bemf / CrossAndBemf
```

使用本工程现有 Park 符号和电机方程独立推导，不能直接复制 VESC 的正负号。先在
仿真中分别验证正反转、加减速和 Ld≠Lq，再进入低功率实机。

#### 6.2 电压限幅策略

保留现有“按比例缩放 vd/vq”的圆限幅，并增加可选的 d 轴优先模式。二者用途不同：

- 圆限幅保持电压矢量方向，适合当前 `Id≈0` 的普通运行；
- d 轴优先适合弱磁或需要保留磁链控制时；
- 两种模式都必须有正确 anti-windup；
- d 轴优先仍包含平方根，不能未经测量声称一定更省 cycles。

#### 6.3 角度延迟补偿

ST 参考工程当前补偿系数为 0，因此该功能应根据本工程实际延迟独立实现：

```text
补偿角 ≈ 电角速度 × ADC采样到Park使用/占空比生效的实际延迟
```

Park 与逆 Park 可使用不同延迟，但必须由 GPIO/DWT/定时器事件测得，并用编码器角度
验证。不得用观察器自身输出证明补偿正确。

#### 6.4 温度补偿

只有取得可信的电机绕组温度或经过验证的温度估计时才启用：

```text
Rs(T) = Rs(T0) × [1 + αCu × (T - T0)]
```

可相应调整观察器 Rs 和电流环 Ki 以保持设计带宽。IHM16M1 板载温度不等于电机绕组
温度，不能直接替代。稳态 Vq 随绕组温升增加是正常物理现象，补偿目标不是强行让
Vq 曲线保持不变。

### 阶段 7：HFI、MTPA 和弱磁

#### HFI 前置门

只有同时满足以下条件才开始：

- Ld/Lq 已在多个角度和电流下实测；
- 凸极差足以稳定区分转子方向；
- 有编码器角度作为开发真值；
- 注入电压、电流、声噪和温升上限已定义；
- 失败时可回退 Rev-Up 或安全停机；
- PC 模型能复现凸极和高频注入响应。

GBM2804H 是外转子云台电机并不能自动证明适合 HFI。若 `Ld≈Lq`，继续优化现有
Rev-Up + BEMF/SMO 比强行加入 HFI 更合理。

#### MTPA 与弱磁

当前低压、低速云台场景不急于使用弱磁。只有目标转速接近电压限制或实测 Ld/Lq
支持明显磁阻转矩时才接入，并继续保持默认关闭。

### 阶段 8：产品化验证

功能完成不等于可正常使用。最终至少补齐：

- Break2/驱动故障真实注入；
- 软件过流、ADC 丢采样、ADC 饱和、母线过欠压；
- CORDIC 超时或后端失败；
- 观察器启动失败和运行失锁；
- deadline miss、trace 溢出和配置版本错误；
- 正反转、冷启动、热启动、低压、高压、阶跃负载；
- 多台电机/多块板的一致性；
- 30 分钟以上温升与长期运行；
- 编码器/测速仪真值下的速度、角度、效率和电流 THD；
- 掉电参数一致性、CRC、版本迁移和恢复默认值；
- 更换 MCU 后的数学后端对拍和 WCET 重测。

在全部生产门通过前继续保持：

```text
closed_loop_enable = 0
inverter_compensation_enable = 0
phase_voltage_feedback_enable = 0
decoupling_mode = Disabled
hfi_enable = 0
```

这些默认值是安全发布基线，不妨碍测试时在停机状态显式打开单个功能。

## 7. 配置与接口设计

### 7.1 编译期配置

适合放入 Kconfig：

- MCU 数学后端；
- 是否有相电压硬件；
- 是否编译 HFI/MTPA/弱磁等大模块；
- trace 缓冲容量或是否完全裁剪；
- bring-up 与 production 构建类型。

### 7.2 运行时配置

适合加入 `foc_runtime_config_t`，且只能停机修改：

- PWM、控制、观察器和速度环频率；
- 电机 Rs/Ld/Lq/磁链和参数版本；
- 逆变器模型参数和电压源；
- 解耦与限幅模式；
- 角度延迟补偿；
- 温度补偿；
- HFI 参数和启用门。

每次扩展 ABI 必须：

1. 同时修改 Rust `#[repr(C)]` 和 C 头文件；
2. 增加 ABI/config version；
3. 保留 `struct_size` 校验；
4. 更新 `_Static_assert`、Rust layout 测试和 Host C ABI 测试；
5. 拒绝未知版本，而不是按旧布局继续运行；
6. 更新 trace/CSV schema 版本和 MATLAB 导入脚本。

### 7.3 参数来源和优先级

```text
安全编译默认值
    ↓
板卡配置
    ↓
人工批准的电机辨识记录
    ↓
停机状态临时调试覆盖
```

运行过程中禁止 Shell 直接改变快环参数。辨识结果默认只生成报告，只有人工确认后才
写入正式配置。

## 8. 代码落点

| 目录/文件 | 后续责任 |
|---|---|
| `foc/platform/stm32g431/` | PWM/控制分频、ADC 触发、相电压采样、DWT/GPIO 时序、硬件关断 |
| `foc/include/foc_platform.h` | 平台能力、关联时序记录、相电压有效性和诊断字段 |
| `foc/include/foc_rust_bridge.h` | 稳定且版本化的 C ABI，不放 STM32 类型 |
| `rust/crates/foc-rt-bridge/` | 运行配置、状态机、补偿选择、C ABI 校验和目标/Host 公共路径 |
| `rust/crates/foc-control/` | 共享 Clarke、观察器电压源、dq 解耦、限幅策略、角度补偿 |
| `rust/crates/foc-algorithm/` | 纯数学补偿、辨识、HFI/MTPA/弱磁，不依赖 RTOS/HAL |
| `rust/crates/foc-sim/` | 逆变器非理想、频率解耦、延迟、相电压传感器和参数辨识 plant |
| `simulation/` | 批量回归、实机采集、同字段比较和自动报告 |
| `simulink/` | 独立模型、参数扫描、图表和实机相关性验证 |
| `applications/main.c` | 显式启停、只在停机修改配置、标定命令和低频状态输出 |

硬件相关寄存器必须继续留在 C 平台层。补偿、辨识和控制公式放 Rust，才能在 PC 上
使用相同实现测试，并在更换国产 MCU 时复用。

## 9. 每项功能的最低验收表

| 功能 | 单元测试 | PC/Matlab | 目标静态 | 低功率实机 | 独立真值 |
|---|---|---|---|---|---|
| 编译优化 | 测试全过 | 数值一致 | size/map/WCET | 5 s 基线 | 不要求 |
| Clarke 复用 | 旧/新对拍 | CSV 一致 | WCET 降低 | 电流误差不退化 | 建议 |
| PWM/控制分频 | 时间基准测试 | 零阶保持/延迟模型 | ADC/PWM 波形 | 无漏拍、无超时 | 建议 |
| 死区补偿 | 正反转/过零/量纲 | 参数扫描 | 默认关闭可构建 | 限流短时 A/B | 编码器强烈建议 |
| 相电压反馈 | 有效性/回退 | 传感器模型 | ADC 与示波器对拍 | Command/Hybrid A/B | 编码器强烈建议 |
| 参数辨识 | 已知 plant 回归 | 噪声/偏差扫描 | 超时/关断测试 | 重复测量 | 外部仪器抽检 |
| dq 解耦 | 符号和反转测试 | 负载阶跃 | 默认关闭 | 扰动 A/B | 速度真值 |
| 角度补偿 | 正负速度测试 | 延迟扫描 | 时间测量 | 角误差 A/B | 编码器必须 |
| HFI | 凸极 plant | 注入扫描 | 故障回退 | 低压温升/声噪 | 编码器必须 |

任何一列失败，都不能仅凭“电机能转”把功能标为完成。

## 10. 推荐实际执行顺序

1. 完整 ISR 时序、Flash 和测试基线；
2. `O3/s/z` 实测，不先决定结果；
3. 裁剪无关格式化/调试体积，建立 production 配置；
4. 共享 Clarke、缓存/预计算并优化 CORDIC 事务；
5. 在保持 12 kHz 控制环不变时重新测 WCET；
6. 仿真实现 PWM/控制频率分离和额外延迟；
7. 实机评估 24 kHz PWM + 12 kHz 控制；
8. 在 Host/Matlab 比较当前软零带与滤波电流极性死区补偿；
9. 补偿默认关闭接入目标固件，低功率 A/B；
10. 完成 Rs/Ld/Lq/磁链辨识流程；
11. 接入板载 BEMF ADC 并先只记录；
12. 增加 Hybrid 观察器电压、dq 解耦和角度延迟补偿；
13. 有独立角度真值、凸极证据后再研究 HFI；
14. 最后评估 MTPA、弱磁、温度补偿和产品化默认值。

## 11. 明确禁止的捷径

- 不把 `MCSDK_REFERENCE_PWM_HZ` 直接改成 `CONTROL_PWM_HZ` 当作 Ki 修复；
- 不把独立分段最大值相加成一次 ISR 最大值；
- 不用 ELF 文件大小代替 Flash `text + data`；
- 不因为 VESC 使用 float，就直接把控制环升到 25/30 kHz；
- 不在没有实测 Ld/Lq 时宣称电机适合 HFI；
- 不把板载温度当作绕组温度；
- 不把相电压 ADC 原始值未经有效性判断直接送入观察器；
- 不同时启用多个新补偿后只看“能不能转”；
- 不使用观察器自己的速度证明观察器准确；
- 不直接复制 VESC GPL 源码到当前工程；
- 不在功率级运行时修改快环配置；
- 不在没有故障回退和默认关闭开关时合入高风险功能。

## 12. 参考版本

- ST 参考工程：`P-IHM03-Potentiometer`，MCSDK 6.4.1；
- VESC 官方仓库：<https://github.com/vedderb/bldc>；
- 本次 VESC 核对提交：`4fd8279ea45a17c0d69357438ae2f7237a32514f`；
- VESC 频率与采样：`motor/mcconf_default.h`、`motor/mcpwm_foc.c`、
  `motor/foc_math.c`；
- VESC 逆变器模型：`motor/mcpwm_foc.c::update_valpha_vbeta()`；
- VESC 参数辨识：`mcpwm_foc_measure_resistance()`、
  `mcpwm_foc_measure_inductance()`、`mcpwm_foc_measure_res_ind()`；
- IHM16M1 官方资料：<https://www.st.com/en/evaluation-tools/x-nucleo-ihm16m1.html>。
