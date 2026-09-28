# FluxRT 通用 FOC 产品架构与双仿真路线图

## 1. 文档定位

本文定义 FluxRT 从当前 STM32G431 单板 FOC 工程演进为类似 VESC / ODrive 的通用电机
控制平台时，应当具备的产品架构、模块边界、功能路线、双仿真体系和阶段验收门。

本文回答五个问题：

1. “通用 FOC 项目”具体包含哪些层次，而不是只列算法名称；
2. C、Rust、RT-Thread、板级代码和上位机分别负责什么；
3. 转矩、速度、位置、传感器、通信、配置和保护如何统一进入同一个电机轴；
4. Rust PC 与 MATLAB/Simulink 两套仿真怎样互相独立、又使用同一份场景和参数；
5. 后续按什么顺序实现，达到什么证据才允许进入下一阶段。

本文是**目标架构和长期路线**，不是当前能力声明。当前实机、仿真和测试结论仍以
[工程操作日志](工程操作日志.md)、[后续任务阶段计划](后续任务阶段计划.md)及对应性能
报告为准。

---

## 2. 总体结论

FluxRT 不应变成“把 VESC 和 ODrive 源码拼在一起”的工程，而应形成自己的三层产品：

```text
第一层：可移植电机控制内核
  FOC、启动、观察器、传感器融合、转矩/速度/位置控制、轨迹与限制器

第二层：电机控制设备平台
  Axis、MotorService、配置、标定、故障、命令仲裁、遥测、通信和升级

第三层：用户与验证生态
  上位机、CLI、配置向导、Rust PC 仿真、MATLAB/Simulink 仿真、HIL 和板卡包
```

当前工程在第一层已经有较好基础，在第二层只有部分安全、配置和 Shell 骨架，在第三层
已有双仿真原型但尚未形成统一产品契约。下一阶段的重点应当是先建立第二层，而不是继续
孤立增加高级算法。

---

## 3. 当前基线与证据边界

### 3.1 已有基础

当前已经具备：

- Clarke/Park、逆变换、PI、电压圆限幅、SVPWM；
- Rev-Up、开环保持、接管过渡、BEMF/SMO-PLL 和失锁保护；
- C 平台层、Rust ABI、Rust 控制组合层、Rust 纯算法层；
- `FeedbackPort`、`PwmPort`、`SafetyPort` 和 PC fake；
- 版本化实时输入 ABI、运行配置 CRC、候选 profile 与审批信息；
- Diagnostic、Calibration、Identification、Production 四种构建档；
- Rust PC 闭环 plant、MATLAB/Simulink 独立模型、实机 trace 对比；
- 实时 WCET、deadline、fault latch、功率级关断和显式 fault clear 基础。

### 3.2 当前仍不是产品能力的部分

当前公开操作仍以 `foc_start <rpm>`、`foc_stop`、`foc_cfg` 和 `foc_status` 为中心，尚未
形成统一的转矩、速度、位置、制动和输入模式 API。当前状态机主要服务无感启动，不等于
完整的 Axis 产品状态机。

截至 2026-09-28，审计整改镜像只完成三轮 582 rpm、每轮 5 s、trace-off、空载开环
受限实机验证；没有完成闭环最终复验、trace-on WCET、Break/driver/UV/OV/deadline 故障
注入和长时间带载。因此：

- Host 测试通过不等于目标 MCU 实时性通过；
- Rust 与 MATLAB 仿真一致不等于 plant 正确；
- 短时开环实机通过不等于闭环、带载或产品安全通过；
- 当前文档中的参数候选不自动成为生产 profile。

---

## 4. 借鉴 VESC 与 ODrive 的边界

### 4.1 从 VESC 借鉴

- 所有应用、通信和输入最终进入统一电机控制门面；
- 占空比、电流、制动、速度、位置、开环和释放共享一致的安全语义；
- 高频 ADC/PWM ISR 与慢速故障、统计、通信和配置任务分离；
- 统一故障汇聚、故障历史、通信超时和安全停机；
- 板卡硬件配置、应用配置和电机配置分离；
- CAN 多控制器、上位机配置和 Bootloader 形成完整生态。

### 4.2 从 ODrive 借鉴

- 每台电机以 `Axis` 为产品对象；
- `AxisState`、`ControlMode`、`InputMode` 分开表达；
- 转矩、速度、位置按串级控制器组合，而不是复制多套 FOC；
- 标定、Index Search、Homing、Closed Loop 是显式请求状态；
- 配置与实时状态可查询，参数写入与保存分离；
- 轨迹规划、前馈、软硬限制和反馈源属于 Axis 配置。

### 4.3 FluxRT 保留的自身特点

- RT-Thread 负责设备、线程、Shell 和通信驱动；
- 厂商寄存器、ISR、PWM/ADC/Break 和立即硬件关断保留在 C；
- 可移植控制状态、策略和算法优先放在 `no_std` Rust；
- 同一 Rust 控制实现可在 MCU 与 PC 仿真复用；
- MATLAB/Simulink 保持独立公式实现，用来发现两边共同复制同一个错误的风险；
- 不直接复制 VESC GPLv3 源码，架构借鉴与代码复用必须分开处理。

---

## 5. 目标系统架构

### 5.1 数据与控制主线

```text
CAN / UART / USB / PWM / ADC / Step-Dir / Shell / Internal App
                              │
                              v
                    Input Adapter Layer
               单位换算、合法性、时间戳、来源
                              │
                              v
                      Command Router
              优先级、租约、超时、急停、所有权
                              │
                              v
                    MotorService / Axis
       AxisState + ControlMode + InputMode + Limits + Faults
                              │
             ┌────────────────┼────────────────┐
             v                v                v
        Trajectory       Position/Speed     Calibration
         Planner          /Torque Loop       Manager
             └────────────────┼────────────────┘
                              v
                   Rust FOC Realtime Core
              Current Loop + Observer + SVPWM
                              │
                              v
                   C Realtime Supervisor
       ADC snapshot -> Rust step -> validate -> CCR preload
                              │
                              v
               PWM / Gate Driver / Motor / Sensors

横向服务：ConfigStore、FaultManager、Telemetry、EventLog、Timebase
```

### 5.2 依赖方向

```text
applications / protocol / input adapters
                  ↓
        motor service / runtime policy
                  ↓
       C ABI / Rust control composition
                  ↓
             pure algorithms

board/platform 只通过端口向上提供能力，不允许算法层反向 include MCU 头文件。
```

禁止出现以下路径：

- CAN 收包函数直接写 TIM CCR；
- Shell 直接改 Rust 控制器内部字段；
- Rust 纯算法读取 STM32 寄存器；
- MATLAB 参数手工更新而没有 profile revision；
- 故障发生后某个输入源绕过 fault latch 再次 arm；
- 配置 Flash 写入期间保持电机输出有效。

---

## 6. 核心领域模型

### 6.1 Device、Axis 与 Motor 的区别

| 对象 | 含义 | 示例职责 |
|---|---|---|
| `Device` | 一块控制器硬件 | 通信、全局配置、母线、升级、一个或多个 Axis |
| `Axis` | 一条可控制机械轴 | 状态、模式、命令、反馈、标定、限制、故障 |
| `Motor` | 电磁参数与执行对象 | 极对数、Rs/Ld/Lq、磁链、温度、额定值 |
| `Inverter` | 功率级 | 电流采样、母线、死区、管压降、驱动器和热限制 |
| `Feedback` | 位置/速度来源 | Sensorless、Hall、ABZ、SPI、Resolver、Fusion |

第一版只实现单 Device、单 Axis，但 API、配置键和状态快照从一开始保留 `axis_id`，避免
以后双电机时重新破坏协议。

### 6.2 AxisState

P1.1 已将目标状态冻结为产品契约 V1（数值、结构和完整拒绝规则见
[产品公共契约 V1](产品公共契约V1.md)）：

```text
Uninitialized
    -> Disabled
    -> Calibration
    -> Startup
    -> ClosedLoop
    -> Stopping
    -> Disabled

任意不安全状态 -> FaultLatched
FaultLatched --显式清除且前置检查通过--> Disabled
```

V1 的独立 `AxisRequest` 为：

- `CurrentOffsetCalibration`；
- `MotorIdentification`；
- `EncoderIndexSearch`；
- `EncoderOffsetCalibration`；
- `HallCalibration`；
- `Homing`；
- `OpenLoopTest`；
- `ClosedLoopControl`。

AxisState 负责“现在处于什么生命周期”，不负责表达目标控制量。

### 6.3 ControlMode

```rust
pub enum ControlMode {
    Inactive,
    Voltage,
    Duty,
    Current,
    Torque,
    Velocity,
    Position,
}
```

- `Inactive` 只用于非 Setpoint 命令，不能拿零目标冒充“无控制”；
- `Voltage/Duty` 只用于明确授权的调试或特殊应用；
- `Current` 使用 `Id/Iq[A]`，与使用 `N*m` 的 `Torque` 明确分离；
- `Torque` 使用最内层电流/转矩控制器；
- `Velocity` 在转矩环外增加速度环；
- `Position` 在速度与转矩环外增加位置环；
- 制动不是简单负速度，必须有独立 `BrakeMode` 和母线能量策略。

### 6.4 InputMode

```rust
pub enum InputMode {
    Inactive,
    Passthrough,
    TorqueRamp,
    VelocityRamp,
    PositionFilter,
    TrapezoidalTrajectory,
    ExternalSynchronized,
}
```

`ControlMode` 决定闭合哪些控制环，`InputMode` 决定如何处理外部目标。二者不能混成一个
大枚举，否则每增加一种轨迹就会复制全部控制模式。P1.1 已冻结允许组合；未知或不兼容
组合全部 fail-closed。

### 6.5 FeedbackMode

```rust
pub enum FeedbackMode {
    Sensorless,
    Hall,
    IncrementalEncoder,
    AbsoluteEncoder,
    Resolver,
    Fused,
}
```

统一反馈快照至少包含：

- 机械位置与速度；
- 电角度与电角速度；
- 多圈位置；
- 时间戳和样本年龄；
- 有效位、质量等级和故障原因；
- 方向、零位和极对数 revision；
- 当前主反馈和备用反馈来源。

---

## 7. 统一 MotorService 接口

### 7.1 命令接口

目标接口应覆盖：

```text
request_axis_state(axis, state)
set_control_mode(axis, control_mode, input_mode)
set_duty(axis, duty)
set_voltage(axis, voltage_v)
set_torque(axis, torque_nm)
set_current(axis, iq_a, id_a)
set_velocity(axis, velocity_rad_s, torque_ff_nm)
set_position(axis, position_rad, velocity_ff, torque_ff)
set_brake_current(axis, current_a)
release(axis)
emergency_stop(axis, reason)
clear_fault(axis)
```

每个接口只提交请求，不直接在调用线程执行硬件动作。实际目标通过固定容量 mailbox 或
无锁快照交给控制任务；ISR 只消费已经校验和仲裁的最新目标。

### 7.2 查询接口

```text
get_axis_state()
get_active_command()
get_limits()
get_feedback()
get_fast_telemetry()
get_slow_telemetry()
get_fault_summary()
get_fault_history()
get_config_identity()
get_firmware_identity()
```

遥测必须区分：参考值、测量值、估算值和最终采用值，禁止只用一个 `speed` 字段同时表示
命令速度、观察器速度和编码器真值。

---

## 8. 命令来源、仲裁与超时

### 8.1 标准命令信封

建议所有输入适配器生成同一种命令：

```text
axis_id
source_id
sequence
created_at
valid_until
control_mode
input_mode
setpoint
feed_forward
limits_override
flags
```

### 8.2 来源优先级

优先级不是写死在各驱动里，而是由版本化 `AppConfig` 配置。默认原则：

1. 硬件 Break、急停、驱动器故障拥有最高优先级；
2. 安全管理器可以撤销任何运行命令；
3. 标定/辨识会话获得临时独占所有权；
4. 正常输入源按显式优先级和租约仲裁；
5. Shell 默认只能在 Disabled 下接管；
6. 命令过期后执行配置的 `coast/brake/hold` 动作。

### 8.3 必测竞争场景

- CAN 控制时 Shell 请求启动；
- UART 丢线而 PWM 输入仍存在；
- 标定过程中收到普通速度命令；
- sequence 重复、跳变和回绕；
- 两个来源同优先级同时刷新；
- 急停与新命令同拍发生；
- 超时后旧帧延迟到达；
- fault clear 后没有新的控制租约。

P1.2 已把本节落为 `foc-control::command` 纯逻辑：固定来源槽、来源权限、每来源全局
sequence、排他时间窗/lease、同拍有界batch、Setpoint/AxisRequest分槽以及独立EStop通道。
完整冻结语义见[命令仲裁与租约契约 V1](命令仲裁与租约契约V1.md)。当前仍未接目标管理
任务或协议，实际 source policy 要由 P3 的版本化 `AppConfig` 提供。

---

## 9. 配置、标定和持久化

### 9.1 配置分组

| 配置 | 主要内容 | 是否跟随板卡/电机 |
|---|---|---|
| `BoardConfig` | ADC/PWM、采样拓扑、分压、电流增益、驱动器、能力位 | 板卡 |
| `InverterConfig` | 电流/电压/温度限制、死区、管压降、制动电阻 | 功率板 |
| `MotorConfig` | 极对数、Rs、Ld/Lq、磁链、额定电流、热参数 | 电机 |
| `AxisConfig` | 反馈源、环路增益、轨迹、软限位、方向 | 机械轴 |
| `AppConfig` | CAN ID、UART、PWM/ADC 输入、优先级和超时 | 应用 |
| `CalibrationData` | 电流零偏、相电压、编码器/Hall 校准 | 板卡+电机组合 |

### 9.2 持久化要求

- schema version、结构长度、CRC 和生成工具版本；
- A/B 双副本或日志式提交；
- 写前验证、写后回读；
- 掉电中断后仍能选出最后一份完整记录；
- 不兼容版本拒绝或显式迁移；
- 工厂默认配置不可被普通保存覆盖；
- 运行中禁止直接写 Flash，必须先释放电机并确认输出关闭；
- candidate、approved、active 三种状态分离；
- 导出文件携带 board/motor ID、revision 和证据引用。

当前 `profiles/schema/fluxrt-production-profile-v1.schema.json` 可作为起点，后续应扩展为上述
多对象配置，而不是再建立一套互不兼容格式。

---

## 10. 传感器与标定路线

### 10.1 推荐实现顺序

1. 保留并稳定当前 Sensorless；
2. ABZ/QEI 增量编码器；
3. Hall；
4. SPI 绝对编码器；
5. BiSS-C/SSI；
6. Resolver；
7. 双反馈融合和冗余诊断。

### 10.2 标定状态机

标定必须是可取消、可超时、失败保持输出关闭的显式会话：

```text
Preflight -> AcquireExclusiveLease -> ArmLimited
          -> Excitation/Rotation -> Collect
          -> Validate -> Candidate
          -> StopAndDisarm -> UserApproval -> Persist
```

需要逐步补齐：

- ADC 电流零偏和噪声；
- 电流增益与极性；
- Rs、Ld、Lq 和磁链；
- 相序和电机方向；
- 极对数；
- 编码器方向、Index、机械零位和电角度偏移；
- Hall 极性、顺序和扇区表；
- 相电压增益/偏置；
- 母线电压与温度通道；
- 转矩常数和机械惯量候选。

任何辨识值必须先成为 candidate，通过仿真、边界检查和受限实机审批后才能进入 active
profile。

---

## 11. 控制功能路线

### 11.1 通用基础能力

- 转矩/电流控制；
- 速度 PI 和转矩前馈；
- 位置—速度—转矩串级；
- 转矩、速度和位置斜坡；
- 梯形轨迹，之后再增加 S 曲线；
- 多圈与圆周位置；
- Homing、Index Search、软限位和硬限位；
- 受限电压/占空比开环测试；
- 制动电流、保持制动和自由停车。

### 11.2 控制质量能力

- dq 解耦和 BEMF 前馈；
- 电压饱和与 anti-windup；
- 采样到生效的角度延迟补偿；
- 死区与器件压降补偿；
- 过调制；
- 在线 Rs/磁链温漂修正；
- 负载与惯量估计。

### 11.3 高级能力

- MTPA；
- 弱磁；
- HFI 低速/零速估算；
- Flying Start；
- Anti-cogging；
- Sensorless/Encoder/Hall 平滑切换；
- 位置与速度前馈；
- 自动整定与频响辅助。

高级能力只能在对应基础传感器、限制器、故障回退和仿真场景已经完成后接入运行路径。

---

## 12. 故障、热与能量管理

### 12.1 故障分层

| 层级 | 示例 | 处理要求 |
|---|---|---|
| 硬件立即关断 | Break、比较器过流、驱动器 fault | 不等待 Rust 或线程，立即禁能 |
| 快环故障 | 非法采样、输出非数、deadline、严重失锁 | 本拍零输出并锁存 |
| 慢速保护 | 温度、失速、连续采样异常、通信超时 | 降额、受控停止或锁存 |
| 配置/会话错误 | CRC、版本、错误板卡、标定失败 | 禁止 arm，保留诊断 |

### 12.2 故障管理器

统一故障记录至少包括：

- fault code、严重度和来源；
- axis/device；
- 首次和最近时间；
- 发生次数；
- 当时的状态、模式、目标和反馈；
- Iabc、Vbus、速度、温度、duty；
- fault epoch、固件和配置 identity；
- 是否自动恢复、恢复次数和最终动作。

故障清除必须检查：输出已关、硬件故障已释放、母线/温度安全、命令租约已撤销，并要求
显式 clear。不能在下一次 start 中顺便清除。

### 12.3 热降额

- MOS/驱动板温度；
- 电机热敏电阻；
- MCU/板内温度作为辅助；
- 低阈值开始线性或曲线降额；
- 高阈值停止并锁存；
- 传感器开路/短路本身也是故障；
- 仿真中同时验证温升模型和传感器故障，不只测一个常数阈值。

### 12.4 再生制动

再生制动必须知道能量去向：

- 电源是否允许吸收负电流；
- 是否有电池/BMS；
- 是否有制动电阻或外部 regen clamp；
- 母线过压阈值和斜坡；
- 制动电阻功率、温度和占空限制；
- 过压时是减小负转矩、投入电阻还是停止。

没有能量路径配置时，不得把负转矩命令简单等同于安全制动。

---

## 13. 通信、遥测与上位机

### 13.1 协议优先级

1. CAN 经典帧：控制、状态、心跳、参数和故障；
2. UART/USB 二进制协议：配置、遥测和升级；
3. PWM/ADC/Step-Dir：简单设备输入；
4. 后续按产品需要选择 CAN-FD、CANopen CiA 402 或 DroneCAN；
5. Shell 始终只作为开发诊断接口。

第一版不要同时实现所有工业协议。先冻结 FluxRT 自有协议语义，再做映射层。

### 13.2 协议必须包含

- 协议版本和设备发现；
- board/device/axis/firmware identity；
- 心跳和在线状态；
- 带 sequence 与租约的控制命令；
- ControlMode/InputMode/AxisState；
- 配置读取、候选写入、校验、保存和恢复默认；
- 标定会话和进度；
- 固定速率遥测订阅；
- 故障历史和冻结帧；
- Bootloader 进入、升级、校验和回滚状态。

### 13.3 上位机最小版本

- 设备发现与连接；
- 安全状态、母线和故障总览；
- 电机/板卡/反馈源配置；
- 标定向导；
- 转矩/速度/位置试运行；
- 实时曲线与 CSV 导出；
- 配置 diff、导入、导出和审批；
- 固件升级；
- 明确显示当前证据等级和未验证能力。

上位机正式命名为 **FluxRT Studio**。其 PySide6/QML 分层、参数事务、实时 Scope、
控制租约、二进制协议、仿真接入和实施阶段见
[FluxRT Studio 上位机架构与调参可视化设计](FluxRTStudio上位机架构与调参可视化设计.md)。

---

## 14. 双仿真总体设计

P1.3 已先完成本章的 D0/D1 公共基础：canonical bundle/schema/profile/scenario/trace/
comparison、524 rpm SI 迁移、Rust/MATLAB 独立 lifecycle engine 和严格比较器均已落地，见
[双仿真共同契约 V1](双仿真共同契约V1.md)。连续波形和 fault 数值对拍仍属于 D2～D4，
不能因 D0/D1 PASS 提前标记完成。

FluxRT 正式保留两套仿真，而不是把 MATLAB 只当作 Rust 输出的画图工具。

| 仿真 | 目录 | 核心目的 | 特点 |
|---|---|---|---|
| Simulation-R | `rust/crates/foc-sim/` | 复用真实 Rust 控制代码做快速回归 | 快、确定性、适合每次提交和 CI |
| Simulation-M | `simulink/` | 用独立 MATLAB/Simulink 公式验证系统行为 | 独立实现、易扫描、易绘图和扩展 plant |

`simulation/matlab/` 和 `simulation/run_matlab_*.ps1` 是两套仿真的编排、绘图、比较和报告
工具，不定义第三套控制器。

### 14.1 为什么必须保留两套

如果 MATLAB 每一步直接调用 Rust 控制器，只能证明 Rust 在另一个 plant 中可以运行；Rust
算法里的单位、符号或状态错误也会被原样带入，无法形成独立复核。

如果 MATLAB 完全脱离 Rust，又允许手工维护参数和场景，两边很快会漂移，比较结果失去
意义。

因此采用：

```text
共享：参数 profile、场景 manifest、单位、输入波形、trace schema、验收公式
独立：控制器公式实现、plant 实现和状态更新代码
比较：事件、状态、故障、波形和统计指标
```

### 14.2 Simulation-R：Rust PC 快速回归

职责：

- 直接复用 `foc-control`、`foc-algorithm` 和必要的 `foc-rt-bridge`；
- 用 fake feedback/PWM/safety/clock 执行与目标一致的控制调用顺序；
- 支持平均逆变器、PMSM、负载、死区、传感器误差和故障注入；
- 检查无堆/固定容量/确定性路径；
- 运行大量参数组合、边界和随机序列；
- 生成机器可读 CSV/JSON，默认不依赖 MATLAB。

适合：

- 每次代码提交；
- 单元与属性测试；
- 状态机和协议 fuzz；
- 数千次冷启动、丢样、超时和故障序列；
- 参数包络筛查；
- PC 上的接口 Mock 测试。

不负责证明：

- MCU ISR WCET；
- 寄存器、ADC/PWM 相位和 Break 行为；
- 功率器件开关纹波、EMI、声学和真实温升；
- plant 参数与实机完全一致。

### 14.3 Simulation-M：MATLAB/Simulink 独立系统仿真

职责：

- `controller_core.m` 独立实现目标控制链；
- `pmsm_plant.m` 独立实现电机和机械 plant；
- `foc_bringup.slx` 表达离散执行拓扑和多速率关系；
- 扫描 PI、观察器、死区、负载、采样频率和延迟；
- 生成曲线、频域/统计结果和仿真实机叠图；
- 后续可接入 Simscape Electrical 或更高保真功率级，但不替代现有快速模型。

适合：

- 系统级趋势和可视化；
- 多速率、延迟和零阶保持审查；
- 观察器收敛、接管和失锁分析；
- 死区、母线、机械负载和热模型研究；
- 与实机 trace 同工况对齐；
- 独立重建关键公式，发现 Rust 侧共同错误。

初期不使用 MATLAB Coder 生成目标固件。固件真值仍是 Rust/C 实现；MATLAB 代码生成若
以后启用，必须作为第三种候选后端单独审批，不能悄悄替换当前控制器。

### 14.4 两套仿真的共同输入

后续新增：

```text
simulation/
  scenarios/
    schema/fluxrt-scenario-v1.schema.json
    nominal_start_582rpm.json
    load_step.json
    observer_loss.json
    command_timeout.json
  contracts/
    trace-schema-v1.json
    comparison-gates-v1.json
  results/                 # 默认忽略，不进 Git
    rust/
    matlab/
    comparison/
```

场景 manifest 至少描述：

- scenario/version/seed；
- board、motor、inverter 和 profile identity；
- PWM、控制环、速度环频率；
- 仿真时长和记录降采样；
- 初始转速、角度、温度和母线；
- 目标命令时间线；
- 负载时间线；
- 传感器噪声、偏置、量化、延迟和故障；
- 电源/母线/制动电阻模型；
- 期望状态事件、禁止事件和比较门。

两边不得各自硬编码一个“相同名字但不同内容”的场景。

### 14.5 参数唯一来源

参数按以下优先级进入仿真：

```text
approved production profile
    > explicitly selected candidate profile
    > checked-in reference profile
    > test-local override
```

每次仿真结果必须记录：

- profile 路径、revision 和 SHA-256；
- runtime config CRC；
- scenario 路径、version 和 SHA-256；
- Rust commit；
- MATLAB 版本和模型 revision；
- 是否启用 candidate 或 test-local override。

`simulink/init_foc_params.m` 最终应从 profile 导入或由生成工具产生，而不是长期手工复制
Rust 默认参数。生成后的 MATLAB 参数快照可以缓存，但必须带来源 hash。

### 14.6 统一 trace schema

每个样本至少包含：

| 类别 | 字段 |
|---|---|
| 身份 | scenario/profile/config/model revision |
| 时间 | time、control tick、PWM tick、sample age |
| 状态 | AxisState、ControlMode、InputMode、active source |
| 目标 | position/velocity/torque/current/duty reference |
| 电流 | Ia/Ib/Ic、Id/Iq、Id/Iq reference |
| 电压 | Vbus、Vd/Vq、三相 command/applied voltage |
| 调制 | duty U/V/W、饱和和限幅标志 |
| 角度 | true、forced、estimated、selected angle |
| 速度 | true、estimated、selected、reference speed |
| 观察器 | BEMF、PLL、quality、reliable、variance |
| 机械 | torque、load、position、inertia |
| 热/能量 | device/motor temperature、bus power、regen power |
| 安全 | fault flags、derating、timeout、output enabled |

没有真值的实机字段必须为空或显式标为 `estimated`，不能把观察器值复制到 `true_speed`。

### 14.7 双仿真比较层级

| Gate | 比较内容 | 规则 |
|---|---|---|
| D0 | schema、单位、参数和场景 identity | 必须完全一致 |
| D1 | 状态、fault、输出启停和事件顺序 | 枚举完全一致，事件允许的 tick 偏差由场景定义 |
| D2 | 单步数学向量 | 固定输入下按每个信号的绝对/相对容差比较 |
| D3 | 完整时序波形 | RMSE、峰值、过冲、稳态误差、接管时间 |
| D4 | 参数和故障矩阵 | 两边必须得到同类成功/故障分类 |
| D5 | 与实机相关性 | 趋势、事件和已校准测量量比较，不要求未建模纹波一致 |

容差必须写入版本化 `comparison-gates`，不能在比较脚本中临时放宽。浮点差异可以使用
容差，状态、故障位、输出关闭和限幅违反属于离散不变量，原则上不允许用 RMSE 掩盖。

### 14.8 必备场景矩阵

#### 正常控制

- 对齐、升速、保持、接管和稳态；
- 正反转与零速附近；
- 速度阶跃、斜坡和负载阶跃；
- 转矩、速度和位置模式切换；
- 饱和、anti-windup 和解除饱和；
- 多速率和 actuation delay。

#### 反馈与观察器

- Sensorless 不收敛、瞬时抖动和持续失锁；
- Hall 合法序列、非法状态和掉线；
- 编码器 Index、方向错误、跳变和延迟；
- 主/备用反馈切换；
- Flying Start 和反向旋转捕获。

#### 电气与传感器

- 电流偏置、增益、噪声、量化和卡值；
- 相电压丢样、stale、饱和、开路；
- 母线跌落、过压和纹波；
- 死区、器件压降和补偿增益扫描；
- Rs/Ld/Lq/磁链误差和温漂。

#### 安全与产品行为

- 命令 timeout；
- 两个输入源竞争；
- 硬件 fault 同拍到达；
- deadline miss；
- 配置损坏与不兼容版本；
- 热降额到停机；
- 再生制动、不可吸收电源和制动电阻饱和。

### 14.9 双仿真执行频率

| 时机 | Rust PC | MATLAB/Simulink |
|---|---|---|
| 每次小提交 | 全部单元测试 + 快速核心场景 | 不强制 |
| 合并前 | 完整场景矩阵 | 核心 D0～D3 场景 |
| 参数/控制器变更 | 完整参数包络 | 必须运行对应扫描和对拍 |
| 发布候选 | 全部 D0～D4 | 全部 D0～D4，生成报告 |
| 实机相关性更新 | 重放同一 scenario | 同工况运行并与实机叠图 |

MATLAB 许可证或运行环境暂时不可用时，只能把 MATLAB gate 标成 `NOT_RUN`，不能用 Rust
PASS 替代。核心开发不能因此完全阻塞，但发布候选不能绕过规定的 MATLAB gate。

### 14.10 仿真到实机的相关性闭环

```text
同一 profile + scenario
        │
        ├── Rust PC trace
        ├── MATLAB/Simulink trace
        └── Hardware trace
                 │
                 v
          单位/时间轴/事件对齐
                 │
                 v
        误差归因与模型 revision
```

差异必须归入以下一种：

1. 控制实现差异；
2. 调度/延迟差异；
3. 参数差异；
4. 传感器与量化差异；
5. 逆变器非理想；
6. 机械负载与结构差异；
7. 实机缺少独立真值，暂不可判定。

禁止仅通过调 MATLAB 参数把曲线“调得像”而不更新 profile、证据和模型 revision。

---

## 15. C、Rust 与仿真的职责

| 区域 | C | Rust | MATLAB/Simulink |
|---|---|---|---|
| MCU/RTOS | 启动、RT-Thread、驱动、ISR | 不依赖 MCU | 不涉及 |
| 快速硬件安全 | Break、gate、CCR 清零 | 输出合法性与故障请求 | 独立故障场景 |
| 控制算法 | 数学硬件适配 | 正式实现 | 独立复核实现 |
| Axis 策略 | 线程/传输适配 | 状态、模式、仲裁策略 | 独立状态模型 |
| 配置 | Flash 原语 | schema 校验、迁移策略 | 导入同一 profile |
| 传感器 | 外设采集 | 质量、融合、选择 | 传感器/故障模型 |
| plant | 不实现 | 快速平均模型 | 独立系统模型/高保真扩展 |
| 遥测 | 非阻塞采集/传输 | 快照定义 | 分析和绘图 |

为了让 PC 纯软件测试覆盖产品骨架，Rust 侧后续还需要补充：

- `ClockPort`；
- `ConfigStorePort`；
- `CommandSourcePort` 或纯数据 mailbox；
- `PositionFeedbackPort`；
- `ThermalPort`；
- `BusEnergyPort`；
- `EventSinkPort`。

端口用于隔离硬件边界，不要把每个纯数学函数都抽象成 trait。

---

## 16. 分阶段实施路线

编号表示能力域，不再强制按数字顺序实施。当前执行优先级冻结为：

```text
U0 → U1 → U2 → U3 → U4 → U5 → U7 → U8-S/U8-T → U6 → U8-H → U9～U11
```

也就是先完成基础和高级FOC的软件/双仿真/目标关闭态门，再开发FluxRT Studio。U8-H是
高级功能powered实机门，可以与U6只读上位机并行，但U8-S/U8-T未完成前不启动上位机代码。

### U0：关闭当前基线缺口

目标：先证明当前板卡的已实现能力，不把旧风险带入通用化。

内容：

- 完成 FRT-001/002/003/005/006/007 目标端剩余验证；
- trace-on、闭环路径重新测量完整 ISR WCET；
- 观察器失锁根因与默认关闭状态保持一致；
- 固化当前 profile、固件 identity 和恢复点；
- 将未提交审计整改形成可回退 Git 提交。

完成门：当前硬件路径和证据边界明确，不能要求所有闭环参数已达到最终性能。

### U1：冻结产品领域模型

目标：建立不依赖 CAN、Shell 或某块 MCU 的 Axis/MotorService 契约。

内容：

- `AxisState`、`ControlMode`、`InputMode`、`FeedbackMode`；
- 统一命令、遥测、限制器和故障快照；
- C ABI 版本化；
- PC 上完成状态和非法组合测试。

完成门：现有速度启动可以通过新接口表达，默认行为不变。

### U2：命令仲裁与超时

目标：所有输入源经过相同的安全、租约和超时语义。

内容：

- 命令 mailbox；
- source/sequence/timestamp/lease；
- 优先级与独占会话；
- timeout 后 coast/brake/hold 策略；
- Shell 迁移到普通输入适配器。

完成门：Rust PC 两输入源竞争和 fault 同拍场景通过；目标默认仍不新增外部控制权限。

### U3：配置持久化

目标：从编译期常量和临时 Shell 配置升级为可恢复配置系统。

内容：

- Board/Motor/Inverter/Axis/App/Calibration 分组；
- 双副本、CRC、迁移、回滚和 factory defaults；
- candidate/approved/active；
- Rust PC 断电时刻穷举测试；
- MATLAB 自动导入 profile。

完成门：任意模拟掉电不会产生半份可用配置，错误板卡/电机组合拒绝 arm。

### U4：反馈源与基础标定

目标：在同一 Axis 下支持 Sensorless、ABZ 和 Hall。

内容：

- 通用反馈快照和质量门；
- 编码器方向/Index/offset；
- Hall 极性/顺序/扇区；
- 反馈选择和故障回退；
- 两套仿真增加传感器模型。

完成门：相同速度/位置控制器可切换反馈源，失效时进入定义好的安全状态。

### U5：转矩、速度、位置与轨迹

目标：形成通用伺服/ESC 控制接口。

内容：

- Torque/Velocity/Position；
- ramp/filter/trapezoidal trajectory；
- feed-forward、软限位、Homing；
- 模式切换无突变策略；
- 单位统一为 SI，协议层按需要显示 rpm/turn。

完成门：两套仿真 D0～D4 通过；实机按转矩、速度、位置分别建立受限验收。

### U6：通信协议与上位机最小闭环

目标：脱离 Shell 也能完成设备发现、配置、标定、控制和诊断。

实施前置：U0～U5、U7以及U8-S/U8-T完成。此前只保留架构文档和仿真trace schema，
不创建FluxRT Studio页面、FakeDevice或正式设备协议实现。

内容：

- CAN 控制/状态/心跳；
- UART/USB 配置与遥测；
- 协议版本和兼容策略；
- Python/Qt 上位机 MVP；
- 命令 timeout、批量参数和 trace 下载。

完成门：断线、乱序、重复帧和非法参数不能绕过安全层。
FluxRT Studio 先按只读链路、参数事务、受控试运行的顺序开放权限，不允许第一版 GUI
直接绕过 MotorService 操作硬件。

### U7：热、功率与再生管理

目标：将电流限制升级为随温度、母线和电源能力变化的动态包络。

内容：

- MOS/电机温度和传感器故障；
- 热降额；
- DC source/sink 能力；
- 再生电流与母线反馈；
- 制动电阻或外部 clamp 策略；
- 故障冻结帧。

完成门：双仿真过压/热场景通过；实机需要合适功率硬件和测温手段后再验收。

### U8：高级运行范围

目标：在基础闭环稳定后扩展效率、转速和低速能力。

内容：MTPA、弱磁、HFI、Flying Start、过调制和更完整补偿。

U8拆成三个不能混写证据的子门：

- **U8-S 软件/双仿真门**：Rust控制实现与MATLAB独立模型均完成，逐项有适用条件、独立
  开关、正常/边界/失效场景、对照组、图和CSV；
- **U8-T 目标关闭态门**：功能默认关闭时旧路径逐拍等价，四档交叉构建、size/map、完整ISR
  WCET和故障回退通过，不要求带电启用；
- **U8-H powered实机门**：按电机凸极性、转速区间、母线和测量真值逐项受限验证。

U8-S和U8-T是启动U6上位机实现的前置；U8-H可以在上位机只读能力开始后继续。只有三个
子门都通过，才能把对应高级功能标为“实机完成”。GBM2804不适用的MTPA/HFI工况允许使用
独立IPMSM仿真profile证明软件能力，但不能据此宣称该实物电机已验证。

### U9：Bootloader 与升级

目标：形成可维护设备。

内容：

- 固件 manifest、board compatibility、CRC/签名；
- USB/UART/CAN 升级；
- 掉电恢复与回滚；
- 配置迁移；
- SWD 救援路径。

完成门：升级断电和错误镜像不能破坏最后可启动版本。

### U10：多板卡与多轴

目标：证明架构不绑定 STM32G431/IHM16M1。

内容：

- `BoardCapabilities`；
- 1/2/3 shunt、不同 ADC/PWM 拓扑；
- 国产 MCU 平台端口；
- 双 Axis 与共享母线/功率预算；
- per-axis 命令、fault 和 telemetry。

完成门：复用 Rust 控制和双仿真场景，只替换平台与能力配置即可通过基础门。

### U11：产品化验证

目标：从开发工程进入可发布控制器。

内容：

- 长时间、温升、冷/热启动、负载和多样本；
- HIL、故障注入和电源扰动；
- 协议 fuzz、配置掉电、升级回滚；
- EMI/EMC 和硬件保护时延；
- 发布包、兼容矩阵、版本说明和恢复手册。

完成门：按明确产品规格验收，不能用开发阶段阈值代替产品指标。

---

## 17. 建议的版本里程碑

| 里程碑 | 范围 | 明确不包含 |
|---|---|---|
| `0.2 Generic Core` | U0～U2，统一 Axis/模式/命令 | 持久化、上位机、位置实机 |
| `0.3 Configurable Drive` | U3～U4，配置、ABZ/Hall | 高级算法、升级 |
| `0.4 Motion Control` | U5，转矩/速度/位置/轨迹 | 多轴、量产 |
| `0.5 Extended FOC Core` | U7 + U8-S/U8-T，热/母线/再生与高级FOC双仿真、目标关闭态 | 高级功能powered实机结论、上位机 |
| `0.6 Connected Drive` | U6，CAN/UART/USB + 上位机 MVP | U8-H全部实机结论、多板量产 |
| `0.7 Advanced Hardware` | U8-H，高级FOC逐项受限实机证据 | 全平台认证 |
| `0.8 Maintainable Device` | U9，升级和回滚 | 多轴 |
| `0.9 Portable/Multi-Axis` | U10 | 产品级全工况声明 |
| `1.0` | U11 与明确硬件规格验收 | 未列入规格的应用 |

版本号是能力门，不是时间承诺。

---

## 18. 第一批实际落地建议

完成当前 U0 安全缺口后，第一批代码只建立骨架，不改变当前默认运行行为：

```text
rust/crates/foc-control/src/
  axis.rs              # AxisState 与请求/实际状态
  mode.rs              # ControlMode/InputMode/FeedbackMode
  command.rs           # 标准命令、租约和仲裁纯逻辑
  limits.rs            # 单位统一的限制器

foc/include/
  foc_motor_service.h  # C 侧稳定服务 API

foc/runtime/
  foc_motor_service.c  # RT-Thread/平台适配，不包含控制公式
  foc_command_router.c # 输入 mailbox 与来源状态

simulation/scenarios/
  schema/...
  legacy_speed_start.json
  command_timeout.json
```

第一批完成条件：

1. 旧 `foc_start 582` 被适配为 Axis 的 Velocity/Passthrough 命令；
2. 旧控制链逐拍数值不变；
3. 未获得租约、命令过期、非法模式组合均输出关闭；
4. Rust PC 与 MATLAB 都能读取 `legacy_speed_start` 场景；
5. 两套仿真记录相同 identity 和统一 trace 字段；
6. 默认闭环、参数和保护阈值不因架构改造而开放。

---

## 19. 每项功能的完成定义

任何新功能必须同时满足：

- 写明所在层、所有者和依赖方向；
- 有配置版本、范围检查和安全默认值；
- 有命令权限、超时和故障回退；
- 快环无堆、无阻塞、无日志和无不可控动态分派；
- Rust 单元/Host 测试覆盖正常、边界和故障；
- Simulation-R 使用正式 Rust 实现通过；
- Simulation-M 用独立公式在相同场景通过；
- 双仿真 D0～D4 的结果和差异有记录；
- 目标交叉构建、ROM/RAM 和完整 ISR WCET 有记录；
- 需要实机时按低压、限流、短时、可立即断电逐级验证；
- trace、profile、scenario、ABI 和协议 schema 同步升级；
- 更新专项报告和[工程操作日志](工程操作日志.md)；
- 没有独立真值时明确写“功能/趋势验证”，不写成精度证明。

---

## 20. 当前暂缓项

以下内容有价值，但不应早于通用骨架：

- 同时支持 BLDC 六步、PMSM FOC、ACIM 和步进电机；
- 一次性支持 CANopen、DroneCAN、VESC CAN 和自有协议；
- 双 Axis；
- Resolver/BiSS-C；
- 直接由 MATLAB Coder 生成目标 FOC；
- 全功能图形上位机；
- 自动调参、神经网络或复杂自适应控制。

第一版产品范围建议锁定为：单 Axis、三相 PMSM/BLDC FOC、Sensorless + ABZ + Hall、
Torque/Velocity/Position、CAN + UART/USB、配置持久化、基础热/母线保护、双仿真和升级恢复。

---

## 21. 文档关系

- [整改架构与职责分配](整改架构与职责分配.md)：当前源码各层落点和不可破坏的依赖；
- [ST 与 VESC 工程改进路线](ST与VESC工程改进路线图.md)：当前控制性能和参数路线；
- [后续任务阶段计划](后续任务阶段计划.md)：A0～A28 当前开发板验证；
- 本文：U0～U11 通用产品化路线和双仿真正式契约；
- [FluxRT Studio 上位机设计](FluxRTStudio上位机架构与调参可视化设计.md)：细化在线
  调参、状态可视化、协议、控制租约、trace 和仿真工作区；
- [MATLAB 联合仿真](MATLAB联合仿真.md)：Rust trace 的 MATLAB 绘图/比较入口；
- [Simulink 仿真工程说明](../simulink/Simulink仿真工程说明.md)：独立 MATLAB/Simulink 模型操作；
- [仿真实机相关性验证](仿真实机相关性验证.md)：仿真与硬件证据的对齐方式；
- [架构与安全边界](架构与安全边界.md)：当前实机不可绕过的安全约束；
- [工程操作日志](工程操作日志.md)：每次修改、验证、Git 和实机状态。

执行顺序是：先完成 A/FRT 当前安全门，再按 U 路线建立产品骨架。A 路线回答“当前板卡
和当前控制链是否被证据证明”，U 路线回答“如何把它扩展为通用产品”；二者不能互相替代。
