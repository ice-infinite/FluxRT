# FOC 整改工程架构分配与代码落位规范

## 1. 文档定位

[ST/VESC 工程修正路线](ST_VESC_ENGINEERING_IMPROVEMENT_ROADMAP.md)回答“后续要改什么”，
本文回答以下问题：

1. 每一项整改应该放在哪个目录和模块；
2. C、Rust、RT-Thread、硬件平台和 PC/MATLAB 各自负责什么；
3. 模块之间允许怎样依赖和传递数据；
4. 新功能以什么顺序落入架构，避免继续扩大现有聚合文件；
5. 更换 MCU 时，哪些代码应保留，哪些代码必须替换。

本文是后续整改的结构基线。新增功能应先在本文的“工作包分配表”中找到归属，再实现；
如果确实需要改变边界，应先修改本文并说明原因，而不是直接把代码塞进最方便的文件。

本文列出的“规划新增”文件尚未全部创建，不能把目录规划误读为功能已经实现。

## 2. 当前结构需要解决的问题

当前 C/Rust 分层方向是正确的，但继续增加补偿、辨识和相电压采样之前，需要先控制
模块膨胀：

| 当前位置 | 当前规模 | 已混合的责任 | 整改方向 |
|---|---:|---|---|
| `applications/main.c` | 约 526 行 | 初始化、Shell、配置、状态打印、trace 导出 | 拆成入口、应用服务、Shell、参数服务 |
| `foc_platform_stm32g431.c` | 约 1209 行 | PWM、ADC、保护、ISR、Rust 调用、trace、诊断 | 拆分硬件驱动，并抽出通用实时胶水 |
| `foc-rt-bridge/src/lib.rs` | 约 1810 行 | ABI、上下文、配置、状态机、快环、遥测、测试 | 拆成 ABI、配置、实时执行、管理状态和遥测模块 |
| `foc-control/src/observer.rs` | 约 527 行 | 多种观测器、可靠性与后端选择 | 按公共接口、实现、可靠性和电压源拆分 |

拆分必须保持行为不变并逐步进行。不能一边大规模搬文件，一边同时改 PI、观测器、
PWM 频率和硬件采样，否则出现问题时无法定位来源。

## 3. 不可破坏的架构原则

### 3.1 单向依赖

```text
RT-Thread 应用管理（C）
          ├──────────────→ 参数存储/命令/报告（C）
          ├──────────────→ MCU 平台管理接口（C）
          └──────────────→ Rust 管理 ABI

MCU IRQ/平台驱动（C） ───→ 通用实时监督层（C）
  │                            │
  └→ HAL/LL/CMSIS/寄存器       │ 固定 C ABI
                               ▼
                         Rust ABI bridge
          ▼
Rust 控制组合 foc-control
          ▼
Rust 纯算法 foc-algorithm

Rust target_math ──窄数学端口──→ MCU CORDIC/DSP 适配器

PC 仿真 foc-sim ─────────→ 同一个 bridge/control/algorithm
MATLAB/Simulink ─────────→ 同参数、同场景、同结果字段
```

允许的依赖方向：

- `applications` 可以调用平台管理接口和 Rust 管理 ABI，不进入中断快环；
- MCU 中断入口把硬件快照交给通用实时层；
- 通用实时层只调用 Rust 实时 ABI，不反向读取寄存器或写 PWM；
- MCU 平台实现可以使用 HAL/LL/CMSIS，但不能了解 Rust 内部结构；
- `foc-rt-bridge` 可以依赖 `foc-control` 和 `foc-algorithm`；
- `foc-control` 可以依赖 `foc-algorithm`；
- `foc-sim` 可以依赖 bridge/control/algorithm；
- `foc-algorithm` 不得反向依赖任何上层。

唯一窄例外是 `target_math` 通过固定 C 符号调用数学加速端口；该端口只接收数值并返回
数值，不暴露 HAL 类型、寄存器或控制器状态。Rust panic 紧急关断钩子也是只关断、不
恢复的安全出口，不能扩展成通用硬件调用接口。

禁止的依赖：

- Rust 算法层调用 RT-Thread、HAL、寄存器或文件系统；
- C 应用层直接修改 Rust 控制器内部状态；
- `foc-algorithm` 读取板卡引脚、ADC 标定值或 Shell 配置；
- 仿真单独复制一套与目标固件不同的补偿和控制公式；
- 中断通过动态函数表、trait object、堆分配或锁选择后端；
- 为每个算法分别暴露一个 C 接口，导致 ABI 跟随算法库无限增长。

### 3.2 单一职责判定

新增代码按下面的判定顺序放置：

| 代码特征 | 唯一归属 |
|---|---|
| 读取或写入 MCU 寄存器、HAL 句柄、ADC/PWM/DMA/GPIO | `foc/platform/<mcu>/` |
| 处理中断一拍的调用顺序、deadline、输出二次检查 | `foc/runtime/` |
| RT-Thread、Shell、通信、文件/Flash、参数审批 | `applications/` |
| C 指针、`#[repr(C)]`、ABI 尺寸和版本检查 | `foc-rt-bridge` |
| 组合启动、观测器、电流环、速度环和模式切换 | `foc-control` |
| 与板卡无关、输入相同必得相同输出的数学算法 | `foc-algorithm` |
| 电机 plant、噪声、死区、量化和测试场景 | `foc-sim` / `simulation` |
| 参数扫描、独立模型和图表 | `simulink` |

同一公式只能有一个正式实现。例如死区电压修正的正式公式放在 Rust 纯算法层；目标
固件和 PC 仿真都调用它。MATLAB 可以保留独立公式用于交叉核对，但必须由测试证明
两者数值一致。

## 4. 目标目录与文件分配

标记说明：`现有` 表示保留，`规划新增` 表示整改时建立，`逐步拆分` 表示先迁移再缩小
原文件。

```text
FluxRT/
├─ applications/
│  ├─ main.c                         现有，最终只保留启动和服务注册
│  ├─ foc_app_service.c/.h           规划新增，应用模式和启停协调
│  ├─ foc_shell.c/.h                 规划新增，Shell 参数解析和输出
│  ├─ foc_parameter_service.c/.h     规划新增，参数校验/待应用/持久化
│  └─ foc_calibration_service.c/.h   规划新增，辨识命令和人工批准流程
│
├─ foc/
│  ├─ include/
│  │  ├─ foc_types.h                 现有，最小跨语言定宽数据类型
│  │  ├─ foc_rust_bridge.h           现有，唯一 Rust C ABI
│  │  ├─ foc_platform.h              现有，MCU 平台公共契约
│  │  └─ foc_realtime.h              规划新增，通用实时监督层接口
│  ├─ runtime/
│  │  ├─ foc_realtime.c              规划新增，一拍调用与输出复核
│  │  ├─ foc_realtime_timing.c       规划新增，关联 WCET 和 deadline
│  │  └─ foc_trace_buffer.c          规划新增，ISR/线程 SPSC trace
│  └─ platform/stm32g431/
│     ├─ foc_platform_stm32g431.c    逐步拆分，生命周期与公共接口汇总
│     ├─ foc_stm32g431_internal.h     规划新增，仅本平台内部可见
│     ├─ foc_stm32g431_pwm.c          规划新增，TIM1/CCR/触发/载波
│     ├─ foc_stm32g431_adc.c          规划新增，三分流与监控 ADC
│     ├─ foc_stm32g431_phase_voltage.c 规划新增，BEMF1/2/3 采样
│     ├─ foc_stm32g431_safety.c       规划新增，Break/COMP/关断
│     ├─ foc_stm32g431_irq.c          规划新增，中断入口和标志清理
│     └─ foc_math_accel_stm32g431.c   现有，CORDIC 数学后端
│
├─ rust/crates/
│  ├─ foc-algorithm/src/
│  │  ├─ transform.rs                 现有，Clarke/Park 等纯变换
│  │  ├─ filter.rs                    现有，纯滤波器
│  │  ├─ controller.rs                现有，PI/PID 等基础控制器
│  │  ├─ inverter.rs                  规划新增，死区/管压降纯模型
│  │  ├─ identification/              规划新增，Rs/Ld/Lq/磁链估计公式
│  │  └─ ...                          现有高级算法，按 feature 选择
│  ├─ foc-control/src/
│  │  ├─ lib.rs                       现有，最终只声明模块和重导出
│  │  ├─ config.rs                    规划新增，板级无关配置模型
│  │  ├─ timing.rs                    规划新增，多速率计划和真实 dt
│  │  ├─ signal.rs                    规划新增，一次 Clarke 与信号复用
│  │  ├─ current_loop.rs              由 controller.rs 逐步拆分
│  │  ├─ speed_loop.rs                由 controller.rs 逐步拆分
│  │  ├─ voltage.rs                   规划新增，电压源/限幅/解耦/补偿组合
│  │  ├─ startup.rs                   现有，Rev-Up 和接管
│  │  ├─ observer/                    由 observer.rs 逐步拆分
│  │  ├─ identification.rs            规划新增，安全辨识步骤状态机
│  │  ├─ state.rs                     规划新增，控制状态与故障迁移
│  │  └─ telemetry.rs                 规划新增，板级无关遥测快照
│  ├─ foc-rt-bridge/src/
│  │  ├─ lib.rs                       逐步拆分，最终只做模块装配
│  │  ├─ abi.rs                       规划新增，所有 extern C 入口
│  │  ├─ context.rs                   规划新增，原位上下文与生命周期
│  │  ├─ config.rs                    规划新增，C/Rust 配置转换和校验
│  │  ├─ realtime.rs                  规划新增，快环执行器
│  │  ├─ management.rs                规划新增，启动/停止/故障请求
│  │  ├─ telemetry.rs                 规划新增，ABI 遥测转换
│  │  └─ target_math.rs               规划新增，CORDIC/CPU 后端选择
│  └─ foc-sim/src/
│     ├─ plant.rs                     从 lib.rs 拆分，PMSM plant
│     ├─ inverter.rs                  从 lib.rs 拆分，物理逆变器非理想
│     ├─ sensor.rs                    规划新增，ADC/角度/相电压模型
│     ├─ scheduler.rs                 规划新增，PWM/控制/观察器多速率
│     ├─ scenario.rs                  规划新增，统一测试工况
│     └─ report.rs                    规划新增，结果指标和 CSV schema
│
├─ simulation/
│  ├─ scenarios/                     规划新增，版本化场景定义
│  ├─ results/                       现有，机器生成结果
│  └─ scripts/                       现有/扩展，回归与实机对比
└─ simulink/
   ├─ 模型与初始化脚本               现有
   └─ data/                          现有，参数扫描和结果
```

`foc/SConscript` 负责选择通用 runtime 和唯一 MCU 平台实现。换芯片时只能链接一个
`foc/platform/<mcu>`；不允许通过运行时判断同时编入两套硬件驱动。

## 5. 七个区域的职责和边界

### 5.1 RT-Thread 应用管理层：`applications/`

负责：

- 系统启动、服务注册和 Shell/通信命令；
- `Off/Run/Identify/Diagnostic` 应用模式；
- 参数的读取、暂存、人工批准、CRC、版本迁移和落盘；
- 只在停机状态提交新的活动配置；
- 从只读遥测快照打印状态，不直接读取 Rust 上下文；
- 标定报告、运行记录和故障历史。

不负责：

- FOC 数学和观察器更新；
- 在命令线程直接写 TIM/ADC；
- 在电机运行时原地修改快环配置；
- 将辨识结果自动覆盖正式参数。

`main.c` 最终只保留“初始化顺序”，业务命令移入 `foc_shell.c`，应用协调移入
`foc_app_service.c`。

### 5.2 通用实时监督层：`foc/runtime/`

这一层是当前平台文件中应抽出的板级无关胶水。它接收平台已经整理好的物理量快照，
负责单个控制拍：

```text
接收平台快照
  → 检查序号和有效位
  → 检查硬件故障镜像
  → 调用 foc_rust_realtime_step()
  → 验证状态码、NaN、duty 范围和变化率
  → 返回“接受输出”或“拒绝并关断”的确定性结果
  → 记录同一拍 total/pre/control/post
  → 平台根据结果写入预装载 CCR，或立即关断并锁存故障
```

它不调用平台驱动、不访问具体寄存器，也不实现控制算法。平台写 CCR 前仍进行最后
一道范围和硬件状态复核，因此通用检查不能代替硬件侧安全门。

`foc_realtime_timing.c` 保存同一拍的关联时序记录，避免把不同拍的四个最大值相加。
`foc_trace_buffer.c` 只做固定容量 SPSC 环形缓冲；ISR 是唯一生产者，管理线程是唯一
消费者，满时丢样并计数，绝不阻塞 ISR。

### 5.3 MCU 平台层：`foc/platform/<mcu>/`

负责硬件事实：

- 实际 PWM 载波、ADC 触发和控制节拍；
- ADC 原始值到 A/V/℃ 的标定与有效性；
- PWM CCR 预装载、MOE、栅极使能和中性占空比；
- Break、COMP、驱动故障和硬件紧急关断；
- DWT/GPIO 时间测量；
- CORDIC/DSP 等硬件数学适配；
- 相电压、编码器和其他可选传感器能力。

平台层必须报告“实际应用值”，而不是仅保存“请求值”。例如 TIM1 分频计算后，平台
返回实际 `pwm_frequency_hz` 和 `control_frequency_hz`，Rust 只能使用这个快照计算
`ts`，防止 C 与 Rust 各自认为频率不同。

### 5.4 Rust ABI 层：`foc-rt-bridge`

负责：

- 裸指针和 `#[repr(C)]` 的唯一边界；
- ABI/config 版本、`struct_size`、枚举值和对齐校验；
- 在 C 提供的固定内存中创建和销毁上下文；
- 将 C 快照转换为 Rust 领域类型；
- 管理请求与 ISR 快环入口分离；
- 将 Rust 状态转换为稳定 C 遥测。

不负责：

- 新控制公式；
- STM32 寄存器；
- RT-Thread 服务；
- 为算法库每个函数建立 C 包装。

`lib.rs` 最终只是模块声明、必要重导出和目标 panic 入口。算法和状态机不得继续直接
堆在 `lib.rs`。

### 5.5 Rust 控制组合层：`foc-control`

负责把纯算法组成可运行的电机控制流程：

- 多速率调度和每个子环的真实 `dt`；
- 电流快照预处理和 Clarke 结果复用；
- 电流环、速度环、Rev-Up、观察器和接管状态机；
- 观察器电压源 `CommandModel/PhaseVoltage/Hybrid`；
- 逆变器模型、解耦、限幅和角度补偿的组合顺序；
- 观察器可靠性、超时和回退；
- 辨识动作序列，但不直接允许栅极；
- 板级无关遥测。

这一层允许使用 Rust trait 表达测试替身和数学后端，但目标快环采用静态分派或枚举
匹配，不能在 ISR 中使用堆上的 trait object。

### 5.6 Rust 纯算法层：`foc-algorithm`

负责无硬件、无 RTOS、无全局状态的可复用算法：

- 变换、滤波、PI/PID、调制；
- 死区/管压降模型公式；
- Rs/Ld/Lq/磁链估计公式；
- SMO/BEMF/HFI 等算法构件；
- MTPA、弱磁和其他高级控制构件。

输入相同必须得到相同输出。需要安全授权、硬件时序或模式切换的代码不属于这里。
大模块通过 Cargo feature 和上层 Kconfig 双重控制，避免未使用算法占用目标 Flash。

### 5.7 仿真与验证层

`foc-sim` 运行与目标相同的 Rust 控制链，只替换 plant 和传感器。它不允许复制一个
“简化版控制器”。`simulation` 负责批量运行、指标和报告；`simulink` 作为独立模型
验证物理假设和画图。

统一场景至少定义：

- 母线、负载、初始角度、目标速度和运行时间；
- PWM、控制环、观察器和速度环频率；
- 死区、管压降、延迟、ADC 量化和噪声；
- 电机参数版本和温度；
- 预期状态迁移与保护限制。

Rust、MATLAB 和实机报告使用相同字段名、单位和 schema 版本。

### 5.8 当前聚合代码的迁移映射

| 当前内容 | 目标位置 | 迁移要求 |
|---|---|---|
| `main.c` 中 `foc_start/foc_stop/foc_cfg/foc_trace` | `foc_shell.c` + `foc_app_service.c` | 命令名和默认安全行为先保持不变 |
| `main.c` 中活动配置和打印 | `foc_parameter_service.c` | 建立 pending/active，运行时只读 |
| `foc_platform_bind_controller()` | `foc/runtime/foc_realtime.c` 的上下文绑定 | 平台最终不保存 Rust context 指针 |
| ADC IRQ 内 `foc_rust_realtime_step()` 调用 | `foc_realtime_step()` | 平台只提供快照并消费接受/拒绝结果 |
| ADC IRQ 内 duty 通用合法性检查 | `foc/runtime` 首次检查 + 平台写入前最终检查 | 两道门均保留 |
| 平台文件中的 trace 环形缓冲 | `foc/runtime/foc_trace_buffer.c` | ISR 单生产者、线程单消费者 |
| TIM1/CCR/MOE 操作 | `foc_stm32g431_pwm.c` | 不进入通用 runtime |
| ADC 配置/标定/原始值换算 | `foc_stm32g431_adc.c` | 输出带有效位的物理量快照 |
| Break/COMP/驱动故障 | `foc_stm32g431_safety.c` | 保持最高优先级立即关断 |
| bridge `lib.rs` 中 extern C 函数 | `bridge/abi.rs` | 函数签名不在结构拆分阶段改变 |
| bridge `lib.rs` 中控制器状态 | `context.rs` + `management.rs` + `realtime.rs` | ISR 单写者规则不变 |
| bridge 中 Host-only 死区补偿 | `algorithm/inverter.rs` + `control/voltage.rs` | 变成 Host/target 共用、默认关闭 |
| `observer.rs` 中后端和可靠性 | `control/observer/` 子模块 | 枚举值、默认后端和数值先不变 |

只有迁移后的所有回归证据通过，才删除旧实现；迁移期间禁止长期保留两套都会被调用的
实现。

## 6. 状态所有权，避免 C/Rust 双状态机冲突

状态按三个维度分别拥有：

| 状态 | 唯一拥有者 | 示例 | 其他层如何获取 |
|---|---|---|---|
| 应用模式 | C 应用层 | Off/Run/Identify/Diagnostic | 只读服务状态 |
| 硬件安全状态 | C 平台层 | Safe/Ready/Armed/FaultLatched | 平台诊断快照 |
| 控制状态 | Rust control | Alignment/OpenLoop/Transition/ClosedLoop/Fault | ABI 遥测快照 |

允许功率输出必须同时满足：

```text
应用模式允许运行
AND 平台自检完成且没有锁存故障
AND Rust 配置有效且控制状态允许输出
AND 当前一拍输入有效
AND 当前一拍输出通过 C 二次检查
```

任一层拒绝都进入安全关断。C 平台先断硬件，再把故障通知 Rust；不能等待 Rust 返回
之后才关 MOE。

辨识模式不新增一套绕过保护的 PWM 通道。它仍通过相同实时监督层和相同输出安全门，
只是 Rust 使用受限的 `Identification` 控制状态与更低的电流/时间上限。

## 7. 配置、参数和 ABI 分配

### 7.1 配置分组

继续扩展当前扁平 `foc_runtime_config_t` 会让边界难以维护。下一次确实需要增加字段时，
应按下面的领域分组设计 C/Rust 对称的定宽结构：

| 配置组 | 内容 | 最终校验者 |
|---|---|---|
| `TimingConfig` | PWM/控制/观察器/速度环频率 | 平台应用硬件值，Rust 校验整除与 dt |
| `MotorConfig` | 极对数、Rs、Ld、Lq、磁链、电流/转速 | Rust |
| `CurrentControlConfig` | Id/Iq PI、限幅、解耦 | Rust |
| `ObserverConfig` | 后端、SMO/PLL、可靠性、超时 | Rust |
| `InverterConfig` | 死区、管压降、零带、滤波、电压源 | Rust + 平台能力 |
| `StartupConfig` | 对齐、Rev-Up、接管与回退 | Rust |
| `ProtectionConfig` | 过流、母线、duty、deadline | C 平台，不由 Rust 放宽 |
| `FeatureConfig` | 补偿/HFI/MTPA/弱磁开关 | Rust，且受编译期能力限制 |

保护配置与控制配置分开：Rust 不能通过运行配置提高硬件保护上限。应用层可以请求更
保守的限制，但放宽上限必须由板级安全配置决定。

### 7.2 参数的四种形态

```text
编译安全默认值
    ↓
持久化参数记录（带版本、CRC、板号、电机号）
    ↓ 启动时校验
待应用配置 pending
    ↓ 仅在 Safe/Disabled 状态原子提交
活动配置 active（整个运行周期只读）
```

Shell 只修改 `pending`。运行中禁止直接写 `active` 或 Rust context。需要调参时先停机，
提交完整快照，重新配置，再重新 arm。

辨识结果先形成候选记录和报告，人工确认后才能转成新的持久化参数。失败或掉电时继续
使用上一份已批准参数。

### 7.3 ABI 变更规则

下一次扩展快速输入或配置时必须同时完成：

1. 修改 C 定义和 Rust `#[repr(C)]` 定义；
2. 提升 ABI/config version；
3. 保留 `struct_size` 和枚举合法性检查；
4. 添加 C `_Static_assert`、Rust size/alignment/offset 测试；
5. Host C ABI 测试覆盖正常、旧版本、错误尺寸和未知枚举；
6. 更新 trace/CSV schema 版本；
7. 目标交叉构建和 map/size/WCET 重新验收。

不要为每个小功能增加一次 ABI 调用。C 每拍提供完整的输入快照，Rust 每拍返回完整的
输出和必要遥测；内部算法如何组合不暴露给 C。

## 8. 快环数据契约和执行链

### 8.1 目标输入快照

未来替换当前 `foc_feedback_t` 时，快速输入至少要能表达：

- 样本序号和有效位；
- 三相电流与母线电压；
- 可选三相相电压及其有效位；
- 可选编码器/霍尔角度及其有效位；
- 温度及传感器来源；
- 当前控制拍真实 `dt`；
- 平台硬件故障镜像。

可选值必须使用 `valid_flags`，不能用 `0.0` 同时表示“真实为零”和“没有传感器”。
快速输入只含已经标定到物理单位的值；原始 ADC 计数留在平台诊断中。

### 8.2 目标输出

实时输出继续保持窄接口：三相 duty 和最少的输出有效标志。`vd/vq`、观察角、估速等
放入遥测，不要让 C 根据 Rust 内部中间量重新计算控制结果。

平台写入 PWM 前必须检查：

- Rust 返回 OK；
- duty 全部有限且在硬件允许范围；
- 没有新硬件故障；
- 样本没有过期或重复；
- 本拍没有 deadline miss；
- 输出状态与应用/平台/Rust 三层状态一致。

### 8.3 多速率执行

```text
PWM 硬件事件              24 kHz 候选
  └─ 平台采样/控制触发     12 kHz 候选
       └─ Rust 电流环       12 kHz
            ├─ 观察器       12 kHz 或验证后的分频
            └─ 速度环        1 kHz
```

平台层拥有硬件触发和实际频率，Rust `TimingPlan` 拥有软件子环分频。所有分频必须在
停机配置时验证为整数关系；不支持的组合直接拒绝，不能在 ISR 里用浮点时间累加碰运气。

死区归一化使用实际 PWM 周期；PI 使用实际控制周期；观察器使用自己的实际更新周期。
Rev-Up、接管和超时使用真实秒数转换后的确定性计数。

## 9. 并发和数据交换

- Rust controller context 只有 ISR 写；管理线程不得直接访问；
- 启停/配置通过停机状态的管理入口完成，不与 ISR 并发；
- 管理线程读取复制后的 telemetry snapshot，不持有 ISR 使用的锁；
- trace 使用固定容量 SPSC 环形缓冲；
- 参数落盘、CSV 输出和格式化全部在线程上下文；
- 硬件故障 ISR 可随时先关断功率级，并原子设置锁存标志；
- 清除故障必须在功率级关闭后，由显式命令和重新自检完成。

如果以后确实需要运行中改变速度目标，单独设计固定宽度 mailbox，并在控制拍边界消费；
不要因此开放整个运行配置的并发写权限。

## 10. 整改工作包分配表

| 工作包 | 目标 | 主要落点 | 配套验证 | 前置条件 |
|---|---|---|---|---|
| A0 基线证据 | 关联 WCET、size/map、真值接口 | `foc/runtime/*timing*`、平台 DWT/GPIO、`simulation/scripts` | 同拍时序报告、size 报告 | 无 |
| A1 结构拆分 | 拆 main/platform/bridge，行为不变 | 第 4 节规划文件 | 全测试、交叉构建、二进制行为对拍 | A0 |
| A2 多速率 | 24 kHz PWM + 12 kHz 控制候选 | 平台 PWM/ADC、`control/timing.rs`、`sim/scheduler.rs` | 无漏采样、延迟模型、WCET | A1 |
| A3 快环优化 | Clarke 复用、数学后端、trace 裁剪 | `control/signal.rs`、`bridge/target_math.rs`、Kconfig | 数值误差、平均/最坏 cycles | A1 |
| A4 逆变器模型 | 死区、管压降和观察器电压源 | `algorithm/inverter.rs`、`control/voltage.rs`、`sim/inverter.rs` | Rust/MATLAB/实机 A-B | A1、A3 |
| A5 参数辨识 | Rs/Ld/Lq/磁链和审批流程 | `algorithm/identification`、`control/identification.rs`、应用 calibration/parameter service | 已知 plant 误差、重复性、限流 | A1 |
| A6 相电压 | PC0/PC1/PC3 BEMF 采样与混合电压源 | 平台 phase_voltage、快速输入 ABI、`control/voltage.rs` | 示波器/ADC/模型对拍、失效回退 | A2、A4 |
| A7 控制质量 | dq 解耦、限幅、角度/温度补偿 | `algorithm` 纯公式、`control/voltage.rs` 和 config | 正反转、冷热机、饱和区 | A4、独立真值 |
| A8 高级控制 | HFI、MTPA、弱磁 | `foc-algorithm` + `foc-control`，feature 默认关闭 | 凸极、效率、电压限幅、温升 | 参数辨识、编码器真值 |
| A9 产品化 | 故障注入、长测、参数恢复、换 MCU | 平台 safety、应用 parameter service、CI/报告 | 完整生产验收矩阵 | 前述稳定 |

一个工作包可以跨多个层，但每一层只实现自己的责任。例如 A4 中：

```text
algorithm：死区和管压降的数学关系
control：决定给 PWM 前馈还是观察器电压，以及组合顺序
bridge：把配置和遥测跨 ABI 转换
platform：报告实际 PWM 周期和硬件能力
sim：模拟真实逆变器造成的电压损失
application：停机状态下启用/关闭并记录参数
```

## 11. 第一轮只做结构整改的顺序

第一轮不得同时改变电机控制行为，按以下小步进行：

1. 保存当前测试、目标构建、map/size 和实机安全基线；
2. 把 `main.c` 的 Shell 命令移入 `foc_shell.c`，保持命令名和输出不变；
3. 把参数暂存与提交移入 `foc_parameter_service.c`；
4. 在 STM32G431 平台目录建立内部头文件，先拆 safety，再拆 PWM、ADC 和 IRQ；
5. 建立 `foc/runtime/foc_realtime.c`，迁移通用的 Rust 调用、输出复核和关联计时；
6. 将 bridge `lib.rs` 按 ABI/context/config/realtime/management/telemetry 拆模块；
7. 将 `observer.rs` 拆成目录，但保持公开 API 与数值输出不变；
8. 拆分 `foc-sim` 的 plant/inverter/scheduler/report；
9. 每一步分别运行格式、Clippy、Host 测试、C 平台测试、交叉构建和 size 对比；
10. 结构整改通过后，才从 A2/A3 开始增加新能力。

拆文件时不应顺便重命名全部公共 API。先保持旧符号工作，再在单独的 ABI 变更中升级
接口，能显著降低实机回归风险。

## 12. 每次填充功能的完成定义

功能只有同时满足以下条件才算填入架构完成：

- 代码位于本文规定的责任层，没有新增反向依赖；
- 默认关闭时与整改前行为一致；
- 配置有范围检查、版本和安全默认值；
- 快环无堆、无阻塞、无日志、无动态分派；
- 纯算法有边界值和异常输入单元测试；
- PC 仿真使用目标端同一份 Rust 实现；
- MATLAB 独立结果与 Rust 在容差内对拍；
- 目标能交叉构建，Flash/RAM 有记录；
- 开启与关闭时的完整 ISR WCET 都有记录；
- 实机先低压、限流、空载、短时，并有立即关断手段；
- trace/schema/参数版本和相关文档同步更新；
- 未取得独立真值时，不宣称观察器或补偿已最终验证。

## 13. 换 MCU 时的替换范围

合理落位后，更换国产 MCU 的预期范围为：

```text
必须替换：board/、foc/platform/<new_mcu>/、链接/启动/构建选择
按能力替换：硬件数学后端、ADC/PWM 触发、相电压/编码器采样
原则上复用：foc/runtime、foc-rt-bridge、foc-control、foc-algorithm
继续复用：foc-sim、simulation 场景、MATLAB 对拍和验收指标
```

移植不是只做到编译通过。新平台必须重新证明 ADC-PWM 同步、相序、标定、关断延迟、
deadline、数学后端数值误差和完整实机保护。

## 14. 文档之间的关系

- [项目操作记录](PROJECT_OPERATION_LOG.md)：记录每次修改、验证、Git 关联、风险和下一步；
- 本文：决定模块、目录、所有权、依赖和工作包落点；
- [整改路线](ST_VESC_ENGINEERING_IMPROVEMENT_ROADMAP.md)：决定优先级、功能内容和验收门；
- [现有 C/Rust 混合架构](RUST_C_FOC_ARCHITECTURE.md)：说明当前已经接通的调用链；
- [架构与安全边界](ARCHITECTURE.md)：记录当前必须保持的实机安全不变量；
- [硬件数学加速](HARDWARE_MATH_ACCELERATION.md)：约束 CORDIC 和 CPU 回退；
- [仿真与实机相关性](SIMULATION_HARDWARE_CORRELATION.md)：定义模型与实机证据如何对齐。

后续实施以“本文的代码落点 + 整改路线的验收门”为共同准则：只完成其中一半，不能
视为整改完成。
