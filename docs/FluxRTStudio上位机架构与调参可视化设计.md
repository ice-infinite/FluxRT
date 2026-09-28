# FluxRT Studio 上位机架构与调参可视化设计

## 1. 文档定位

本文定义 FluxRT 自有上位机 **FluxRT Studio** 的产品目标、技术路线、界面结构、通信协议、
在线调参安全规则、实时可视化、仿真接入和实施阶段。

上位机不是串口 Shell 的图形包装，也不是把安全逻辑搬到电脑。它是 FluxRT 设备平台的
正式组成部分，负责：

- 发现和连接设备；
- 可视化电机、逆变器、观察器和实时性能状态；
- 读取、编辑、校验、临时应用和保存参数；
- 执行受控标定、参数辨识和试运行；
- 记录波形、事件、故障和配置版本；
- 对比 Rust PC、MATLAB/Simulink 与实机数据；
- 后续执行固件升级和恢复。

固件始终是安全和配置合法性的最终权威。FluxRT Studio 可以提出请求，但不能绕过板端
硬限、状态机、命令租约、fault latch 或 Flash 提交规则。

实施顺序约束：本文先作为设计冻结保留，不代表立即开始上位机代码。只有
`U0～U5 + U7 + U8-S/U8-T`达到[通用FOC产品路线](FluxRT通用FOC产品架构与双仿真路线图.md)
规定的软件、双仿真和目标关闭态完成门后，才创建正式协议、FakeDevice和FluxRT Studio
实现。此前观察高级算法数据使用Rust PC输出和MATLAB/Simulink图表，不以GUI替代仿真证据。

---

## 2. 产品目标

### 2.1 第一版必须完成

1. USB/UART 设备发现和稳定连接；
2. 显示 Device、Axis、固件、板卡和电机 profile identity；
3. 显示 Disabled/Startup/ClosedLoop/Fault 等状态；
4. 显示转速、电流、电压、占空比、角度、观察器、母线、温度和 WCET；
5. 实时波形选择、触发、暂停、缩放、游标和 CSV 导出；
6. 参数读取、分组编辑、单位提示、本地校验和设备端校验；
7. `Apply Volatile`、`Save Persistent`、`Rollback` 三种操作明确分离；
8. 故障总览、历史、冻结帧和显式清除；
9. 受控速度/转矩/位置试运行界面；
10. Rust PC 和 MATLAB/Simulink 结果导入与叠图。

### 2.2 后续扩展

- CAN/CAN-FD 多节点；
- 标定和电机辨识向导；
- 参数扫描和自动整定助手；
- Bootloader、固件升级、校验和回滚；
- 多 Axis 工作区；
- 制动电阻、再生和热模型页面；
- HIL 自动测试和报告；
- 插件化设备/板卡页面。

### 2.3 不属于上位机的职责

- 在连接丢失后继续维持电机控制；
- 代替 MCU 的过流、Break、欠压、过压和 deadline 保护；
- 直接写 MCU 寄存器或 PWM CCR；
- 在 GUI 中复制一套实际运行 FOC；
- 将观察器估算速度冒充独立机械真值；
- 自动把仿真参数保存为生产参数；
- 在设备运行时强制写入必须停机的参数。

---

## 3. 技术路线

### 3.1 第一版技术栈

推荐第一版采用：

```text
Python 3 + PySide6 + Qt Quick/QML
```

理由：

- 当前主要开发环境是 Windows，PySide6 打包和串口/CAN 工具成熟；
- QML 适合仪表、波形、状态卡片和响应式桌面界面；
- Python 适合协议迭代、数据处理、MATLAB 文件和测试工具集成；
- 现有项目已经有大量 Python 采集、分析和 profile 工具，可逐步复用；
- UI 只接收降采样遥测和 trace 块，不在 Python 中执行 12 kHz 控制环。

如果性能分析证明 Python 成为瓶颈，再把协议解析、环形缓冲或数据压缩迁到 Rust 扩展；
不要在没有数据前先引入 Qt/C++/Rust 三语言 GUI 绑定复杂度。

### 3.2 长期边界

QML 和 Python 只属于 PC 工具，不进入 MCU 固件。设备协议使用语言无关 schema，未来可以
增加 Rust/C++/C#/Web 客户端，而不要求修改固件控制逻辑。

### 3.3 拟议目录

```text
host/
  fluxrt-studio/
    pyproject.toml
    README.md
    src/fluxrt_studio/
      app.py
      paths.py
      settings.py
      controllers/
      services/
      adapters/
      core/
      models/
      qml/
        MainWindow.qml
        pages/
        components/
        charts/
        styles/
    tests/
    tools/

protocol/
  schema/
  golden/
  generated/
```

`protocol/` 位于 GUI 之外，作为固件、上位机、测试和第三方 SDK 的共同契约。

---

## 4. 上位机分层架构

```text
┌──────────────────────────────────────────────────┐
│ QML View                                         │
│ 页面、图表、表单、对话框、主题、用户交互           │
└──────────────────────┬───────────────────────────┘
                       │ Property / Signal / Slot
┌──────────────────────▼───────────────────────────┐
│ Controllers                                      │
│ 页面状态、命令入口、选择状态、错误提示              │
└──────────────────────┬───────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────┐
│ Services                                         │
│ 连接、设备会话、参数事务、调参、trace、升级编排      │
└───────────────┬────────────────┬─────────────────┘
                │                │
┌───────────────▼──────┐ ┌──────▼─────────────────┐
│ Core / Models        │ │ Adapters               │
│ 协议对象、参数、状态   │ │ Serial/CAN/Replay/Sim │
│ 缓冲、比较、单位       │ │ MATLAB result/import  │
└──────────────────────┘ └────────────────────────┘
```

### 4.1 QML 层

只负责：

- 显示属性；
- 发出用户意图；
- 页面导航、布局和视觉状态；
- 图表选择、游标和缩放。

禁止：

- 直接打开串口；
- 解析二进制包；
- 计算配置 CRC；
- 决定参数是否安全；
- 在 JavaScript 中保存设备配置。

### 4.2 Controller 层

每个功能域一个 `QObject` Controller，通过 `Property/Signal/Slot` 与 QML 连接：

- `ConnectionController`；
- `DashboardController`；
- `ControlController`；
- `ScopeController`；
- `ParameterController`；
- `CalibrationController`；
- `FaultController`；
- `FirmwareController`；
- `SimulationController`。

Controller 管理页面状态和错误呈现，不包含协议状态机或复杂业务逻辑。

### 4.3 Service 层

- `ConnectionService`：发现、连接、握手、重连；
- `DeviceSessionService`：identity、capabilities、心跳和租约；
- `ParameterService`：读取、diff、校验、volatile apply、commit、rollback；
- `ControlSessionService`：受控试运行和 deadman keepalive；
- `TelemetryService`：订阅、解码、限速和分发；
- `TraceService`：触发、批量读取、保存和标注；
- `CalibrationService`：向导和步骤编排；
- `FaultService`：故障历史与冻结帧；
- `FirmwareService`：镜像检查、升级和恢复；
- `SimulationService`：Rust/MATLAB/实机数据统一导入和比较。

### 4.4 Adapter 层

统一传输接口：

```text
open()
close()
write_frame()
frame_received
connection_state_changed
statistics()
```

计划适配器：

- `SerialTransport`：USB CDC、ST-LINK VCP、UART；
- `CanTransport`：USB-CAN/SocketCAN/厂商适配器；
- `ReplayTransport`：从录制文件重放；
- `RustSimTransport`：连接 `foc-sim`；
- `MatlabResultAdapter`：导入 `.mat/.csv/.json`；
- `FakeDeviceTransport`：UI 和协议测试。

上层页面不应知道当前连接是实机、重放还是仿真。

---

## 5. 页面与信息架构

### 5.1 总览 Dashboard

固定显示：

- 连接方式、设备序列号、板卡、固件和协议版本；
- Axis 状态、ControlMode、InputMode、反馈源；
- Disabled/Armed/Fault 明显状态条；
- 目标/估算/测量转速；
- Iq/Id、母线电压、相电流峰值；
- MOS/电机温度；
- 当前故障和降额原因；
- 命令来源、剩余租约和通信延迟；
- 控制频率、ISR worst/average 和 deadline miss；
- active profile 名称、revision、CRC 和审批状态。

状态颜色不能是唯一信息来源，同时显示文字和图标；Fault 必须包含原因和下一步，不只显示
红灯。

### 5.2 控制 Control

提供：

- AxisState 请求；
- Torque/Velocity/Position/Duty 模式选择；
- Passthrough/Ramp/Filter/Trajectory 输入模式；
- 目标值、斜坡、限制和前馈；
- Acquire Control、Enable、Stop、Emergency Stop；
- deadman keepalive 和 session countdown；
- 输出关闭后的最终状态确认。

第一版默认：

- 连接设备不自动获取控制权；
- 切换页面不保持未确认的运行命令；
- GUI 关闭、连接中断或租约过期由固件执行配置的安全动作；
- Emergency Stop 不需要参数页面保存或确认对话框；
- 清除故障与重新启动是两个独立动作。

### 5.3 实时波形 Scope

至少支持：

- 多通道添加/移除；
- 单位分组和独立 Y 轴；
- 设备时间轴；
- 暂停、缩放、平移和双游标；
- 边沿/阈值/状态/故障触发；
- pre-trigger、post-trigger；
- 采样率、decimation 和 dropped count；
- CSV/MAT/JSON 导出；
- 保存当前通道布局；
- Rust/MATLAB/Hardware 三组叠图。

推荐通道分组：

| 分组 | 通道 |
|---|---|
| State | AxisState、模式、source、fault、output enabled |
| Speed/Position | reference、estimated、selected、true/encoder、position |
| Current | Ia/Ib/Ic、Id/Iq、Id/Iq reference |
| Voltage | Vbus、Vd/Vq、phase command/applied |
| PWM | duty U/V/W、limiter、sector |
| Observer | angle、BEMF、PLL error、variance、reliability |
| Power/Thermal | torque、bus current/power、motor/MOS temperature |
| Realtime | ISR total/pre/control/post、miss、trace dropped |

当前固件已有 72 B 定长 trace 样本，可作为第一版高速 trace 的基础，但协议必须把
trace schema/version/单位一起返回，GUI 不能依赖某个固件的固定列号。

### 5.4 参数 Parameters

分组页面：

- Board / Inverter；
- Motor；
- Current Loop；
- Speed Loop；
- Position Loop；
- Startup / Rev-Up；
- Observer / Feedback；
- Modulation / Compensation；
- Limits / Protection；
- Thermal / Regeneration；
- Communication / App；
- Calibration Data。

每个参数显示：

- 名称、说明、单位；
- 当前 active 值；
- 设备上次保存值；
- 本地 draft 值；
- 默认值；
- 最小/最大和枚举；
- 来源 `[HW]/[ST]/[FW]/[ID]/[USER]`；
- 修改权限和应用条件；
- 是否影响实时性、重新初始化或校准；
- profile revision 和最后修改时间。

### 5.5 标定与辨识 Calibration

向导必须逐步显示：

- 物理接线和机械条件；
- 当前电源、限流和安全确认；
- 固件 build profile 与 capability；
- 每一步会不会 arm、转动或锁定转子；
- 当前步骤、剩余时间和取消动作；
- 原始数据、拟合结果、不确定度和 reject 原因；
- candidate 与 active profile 的差异；
- 用户审批和保存。

上位机不得因为拟合成功就自动写入生产 profile。

### 5.6 故障 Faults

- 当前 active fault；
- first/last fault、count、epoch；
- 故障发生时的冻结帧；
- 建议排查步骤；
- clear 的前置条件和拒绝原因；
- 导出诊断包；
- 不允许把 `Clear Fault` 和 `Start` 合并为一个按钮。

### 5.7 仿真与对比 Simulation

- 启动或导入 Rust PC 结果；
- 启动或导入 MATLAB/Simulink 结果；
- 导入 Hardware trace；
- 检查 profile/scenario/schema identity；
- 对齐状态事件和时间轴；
- 显示 RMSE、峰值、稳态误差、过冲和接管时间；
- 标注哪些通道是真值、测量值或估算值；
- 生成差异报告。

---

## 6. 在线调参安全模型

### 6.1 参数安全等级

| 等级 | 含义 | 示例 | 应用条件 |
|---|---|---|---|
| P0 | 只读/硬件事实 | MCU、引脚、shunt 拓扑、硬件硬限 | 不允许 GUI 修改 |
| P1 | 非控制运行参数 | 遥测速率、界面名称、日志级别 | 可运行中修改 |
| P2 | 控制参数 | PI、observer、ramp、soft limits | 第一版必须 Disabled |
| P3 | 标定数据 | ADC offset、phase map、encoder offset | 仅对应标定会话生成 |
| P4 | 构建/板卡参数 | PWM 拓扑、ADC 触发、驱动类型 | 必须重新构建或更换板卡包 |

后续若开放运行中调 PI，也必须进入受限 Tuning Session：

- 用户显式获得控制租约；
- 固件强制低电流、低速度和最大持续时间；
- 每次只改变允许字段；
- 超时或断线自动恢复 last-known-good；
- 实时监控过流、振荡、饱和和观察器质量；
- 保存 Flash 前必须停机并再次确认。

### 6.2 参数事务

```text
Read Active + Saved
        ↓
Create Local Draft
        ↓
Local Schema Validation
        ↓
CONFIG_BEGIN(device revision, expected CRC)
        ↓
CONFIG_SET_BATCH(changes)
        ↓
Firmware Validation
        ↓
Apply Volatile ──监控──> Accept or Rollback
        ↓
Stop / Disarm / Preflight
        ↓
Commit Persistent + Readback + CRC
```

需要的设备操作：

- `CONFIG_READ_SCHEMA`；
- `CONFIG_READ_ACTIVE`；
- `CONFIG_READ_SAVED`；
- `CONFIG_BEGIN`；
- `CONFIG_SET_BATCH`；
- `CONFIG_VALIDATE`；
- `CONFIG_APPLY_VOLATILE`；
- `CONFIG_ROLLBACK`；
- `CONFIG_COMMIT`；
- `CONFIG_FACTORY_DEFAULTS`。

### 6.3 并发与版本冲突

每次事务携带：

- device/axis；
- schema version；
- base profile revision；
- base CRC；
- transaction ID；
- expected AxisState；
- 修改字段集合。

如果设备配置已被另一客户端或板端会话修改，commit 必须返回 conflict，GUI 重新读取并
显示 diff，不能覆盖新配置。

### 6.4 单位规则

协议和配置以 SI 为规范：A、V、Ω、H、Wb、N·m、rad、rad/s、s、°C。

GUI 可以显示 rpm、turn、mA、mH 等工程单位，但必须：

- 清楚显示单位；
- 输入时立即换算并显示规范值；
- 导出文件记录显示单位与协议单位；
- 禁止同一字段在不同页面使用不同隐含单位；
- Ke、磁链、线/相参数等易歧义量必须显示口径说明。

---

## 7. 通信协议设计

### 7.1 当前差距

当前 `foc_status`、`FSTAT`、`FTIMING`、`FCFG` 和 trace 文本适合开发与脚本，不适合作为
长期 GUI 协议：

- 文本解析容易受格式变化影响；
- 请求和响应缺少统一 correlation ID；
- 不能可靠描述 schema、capabilities 和字段权限；
- 参数事务、批量读写、重放和兼容能力不足；
- 高频遥测占用和 dropped 行为难以协商。

Shell 保留，但 FluxRT Studio 使用版本化二进制协议。

### 7.2 逻辑帧

```text
magic
protocol_version
message_type
flags
sequence
device_id
axis_id
payload_length
payload
crc
```

UART/USB 可以采用 COBS 或等价定界编码避免帧边界歧义；CAN 使用 CAN 帧承载同一消息
语义，不要求逐字节复用串口帧格式。

### 7.3 消息域

| 域 | 消息 |
|---|---|
| Discovery | HELLO、IDENTITY、CAPABILITIES、SCHEMA_LIST |
| Session | HEARTBEAT、TIME_SYNC、ACQUIRE_LEASE、RELEASE_LEASE |
| Control | SET_MODE、SET_SETPOINT、STOP、EMERGENCY_STOP |
| State | AXIS_STATE、FEEDBACK、LIMITS、POWER_STATE |
| Config | READ、BEGIN、SET_BATCH、VALIDATE、APPLY、ROLLBACK、COMMIT |
| Telemetry | CHANNEL_LIST、SUBSCRIBE、SAMPLE_BLOCK、UNSUBSCRIBE |
| Trace | CONFIG、ARM、TRIGGER、STATUS、READ_BLOCK、STOP |
| Fault | SUMMARY、HISTORY、FREEZE_FRAME、CLEAR |
| Calibration | START、STEP、PROGRESS、RESULT、CANCEL、APPROVE |
| Firmware | MANIFEST、BEGIN、BLOCK、VERIFY、ACTIVATE、ROLLBACK |

### 7.4 请求语义

- 每个修改请求都有 sequence/transaction ID；
- 响应包含明确 status code 和拒绝原因；
- 可重试请求必须幂等；
- 设备报告 capabilities，GUI 不显示不支持的控制项；
- 未知消息和未知字段安全拒绝；
- 协议版本不兼容时只允许 identity、诊断和升级；
- 禁止任意内存读写和任意函数调用协议。

---

## 8. 遥测、Trace 与性能

### 8.1 三种数据级别

| 级别 | 用途 | 典型速率 |
|---|---|---:|
| Status | Dashboard、连接健康 | 5～20 Hz |
| Live Telemetry | 实时曲线 | 50～500 Hz，按链路能力协商 |
| Trace Burst | ISR/接管/故障细节 | 板端高频采集，分块读取 |

不建议通过串口永久发送所有 12 kHz 通道。高速数据由板端固定容量环形缓冲记录，再按
触发、降采样和通道选择分块传输。

### 8.2 时间与同步

每个数据块记录：

- device monotonic tick；
- control sequence；
- sample period；
- block sequence；
- host receive timestamp；
- dropped/overrun count。

绘图优先使用设备时间；主机时间只用于连接和日志关联。多设备同步在 CAN time-sync 或
外部同步能力完成前不能宣称同相位。

### 8.3 UI 性能

- 传输和解析在工作线程；
- 原始数据进入固定上限 ring buffer；
- 记录线程保存原始块；
- QML 只接收适合屏幕刷新率的降采样点；
- 图表刷新与数据采集解耦；
- 窗口不可见时降低绘图率但不悄悄停止明确开启的记录；
- 显示 receive、decode、render 和 dropped 统计。

---

## 9. 控制与连接安全

### 9.1 控制租约

```text
Connect != Control Authority
Acquire Lease -> Keepalive -> Commands
Disconnect/Timeout/Release -> Firmware Safe Action
```

上位机必须显示：

- 当前控制来源；
- 是否持有租约；
- 租约剩余时间；
- 超时动作；
- 最后一条已确认命令；
- 设备是否实际接受。

### 9.2 UI 防误操作

- Emergency Stop 固定可见；
- Start 和 Save Persistent 不能放在相邻无区分位置；
- 高风险动作显示实际目标、限制和 Axis；
- 切换 Torque/Velocity/Position 前显示单位和当前限制；
- 参数越界在本地提示，设备仍必须再次拒绝；
- Fault 状态禁止普通 Start；
- 清除故障后保持 Disabled；
- 不使用仅靠颜色的 Armed 状态。

### 9.3 断线与应用崩溃

安全结果只能由固件 timeout 保证。上位机退出钩子可以主动发送 Stop/Release，但不能把它
当作唯一保护，因为进程崩溃、USB 拔出和电脑掉电时钩子不会运行。

---

## 10. 与双仿真的集成

FluxRT Studio 使用同一页面连接三类来源：

```text
Live Device
Rust PC Simulation
MATLAB/Simulink Result or Session
Replay File
```

### 10.1 Rust PC

- 通过子进程/本地 socket 或 trace 文件连接；
- 加载同一 profile 和 scenario；
- Dashboard、Scope 和 Fault 页面复用；
- UI 功能可以在无电机、无开发板时自动测试。

### 10.2 MATLAB/Simulink

第一阶段导入 `.mat/.csv/.json`；后续可由 `SimulationService` 启动 MATLAB batch 并读取
结果。MATLAB 控制器保持独立实现，不在 GUI 中改写其公式。

### 10.3 三方对比

比较前必须检查：

- profile revision/CRC；
- scenario hash；
- trace schema；
- 控制/PWM频率；
- 单位和角度方向；
- 信号是真值、测量值还是估算值。

不一致时显示“不可比较”及原因，不自动缩放到看起来相似。

---

## 11. 测试体系

### 11.1 纯软件测试

- 协议编码/解码 golden vectors；
- 分包、粘包、CRC 错误、长度错误和未知消息；
- 参数 schema、单位换算和 diff；
- 配置事务、版本冲突和 rollback；
- 控制租约、timeout 和重连；
- 遥测 ring buffer、掉包和乱序；
- Replay/FakeDevice；
- Controller/Service 单元测试；
- QML 页面加载和核心交互 smoke test。

### 11.2 仿真集成

- RustSimTransport 驱动完整 Dashboard/Scope/Control；
- MATLAB 结果导入和字段映射；
- 同场景三方叠图；
- 自动生成 D0～D5 比较报告。

### 11.3 目标板/HIL

- 只连接 USB、功率断开时完成协议和配置负例；
- 受限条件下验证租约、Stop 和断线超时；
- Trace burst 不造成 ISR deadline miss；
- 配置 commit 前输出已关闭；
- 故障历史和冻结帧一致；
- 升级掉电和错误镜像回滚。

Host UI PASS 不能代替上述目标证据。

---

## 12. 实施阶段

### S0：协议和 UI 契约冻结

- 定义 identity、capabilities、status code；
- 定义参数 metadata 和安全等级；
- 定义遥测 channel schema；
- 定义连接、配置和控制状态机；
- 建立 FakeDevice 和 golden vectors。

完成门：没有真实设备也能运行页面原型和协议测试。

### S1：只读上位机

- SerialTransport；
- 握手和设备信息；
- Dashboard；
- Status/Telemetry/Trace 只读显示；
- CSV/MAT 导出；
- 不包含 arm 和参数修改。

完成门：接入当前目标板读取状态，不改变设备配置和输出。

### S2：安全参数工作流

- 参数 schema；
- Read/Compare/Draft；
- P1/P2 权限；
- Validate/Apply Volatile/Rollback；
- Disabled 下 Commit 和回读 CRC。

完成门：断线、拒绝、版本冲突和 Flash 失败均不会留下不明配置。

### S3：受控试运行

- 控制租约和 keepalive；
- Torque/Velocity/Position 页面；
- Stop/Emergency Stop；
- 超时安全动作；
- 低功率试运行流程。

完成门：关闭 GUI、拔线和停止 keepalive 后，固件按定义进入安全状态。

### S4：标定与调参

- Current offset、相序、Rs/L、BEMF；
- Encoder/Hall 标定；
- PI/observer 参数候选；
- candidate、approval 和 profile commit；
- 受限 tuning session。

完成门：所有结果可追溯，失败不会写入 active profile。

### S5：仿真与实机统一工作区

- RustSimTransport；
- MATLAB 导入/运行；
- 三方叠图；
- 比较报告；
- scenario/profile 一致性检查。

### S6：CAN、多节点和升级

- CAN transport 和多节点；
- 多 Axis；
- Bootloader；
- 固件 manifest、升级和回滚；
- 发布打包和兼容矩阵。

---

## 13. 第一批代码建议

第一批不要同时实现控制、调参和升级，先建立只读链路：

```text
host/fluxrt-studio/
  app + QML shell
  ConnectionController
  DashboardController
  ScopeController
  ConnectionService
  TelemetryService
  SerialTransport
  FakeDeviceTransport

protocol/
  identity-v1
  status-v1
  telemetry-channel-v1
  trace-block-v1
```

固件端第一批只增加：

- binary protocol parser；
- identity/capabilities；
- 只读状态快照；
- 只读 channel list；
- trace block 读取；
- 心跳统计。

暂不增加：

- GUI arm；
- 参数写入；
- Flash 保存；
- 标定；
- 固件升级。

只读链路通过目标板和 FakeDevice 验证后，再开放 S2 参数事务。

---

## 14. 完成定义

每项上位机功能必须：

- QML、Controller、Service、Core、Adapter 职责不倒置；
- 不在 UI 线程做阻塞串口、CAN、MATLAB 或文件操作；
- 使用版本化协议/schema；
- 对未知设备能力做安全隐藏或拒绝；
- 修改操作具备请求、设备确认和最终回读；
- 高风险操作受 AxisState、租约和权限限制；
- 断线和应用崩溃由固件 timeout 收敛；
- 具备 FakeDevice/Replay 测试；
- 记录设备、固件、profile、协议和 trace identity；
- 清楚区分仿真、Host、目标板和实机证据；
- 更新专项文档和[工程操作日志](工程操作日志.md)。

---

## 15. 与其他文档的关系

- [通用 FOC 产品架构与双仿真路线图](FluxRT通用FOC产品架构与双仿真路线图.md)：定义
  Device/Axis/MotorService、U0～U11 和双仿真总架构；
- 本文：细化 U6 上位机、协议、调参和可视化；
- [整改架构与职责分配](整改架构与职责分配.md)：定义固件 C/Rust 各层落点；
- [仿真实机相关性验证](仿真实机相关性验证.md)：定义数据与实机对齐边界；
- [构建档与优化等级](构建档与优化等级.md)：限制不同固件构建档开放的能力；
- [工程操作日志](工程操作日志.md)：记录实际实现、验证、固件身份和回退点。

FluxRT Studio 应与固件协议共同演进，但不能与某一 MCU 平台实现绑死。更换 MCU 或功率板
时，只要 identity、capabilities、参数 schema 和协议兼容，上位机的主体页面应继续复用。
