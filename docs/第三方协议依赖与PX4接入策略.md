# FluxRT 第三方协议依赖与 PX4 接入策略

## 1. 结论

FluxRT 可以对接 PX4，而且应把它作为正式外部控制目标纳入架构。首选路线是：

1. **DroneCAN ESC**：PX4 通过 CAN 下发电机命令，FluxRT 回报转速、电流、电压、温度和故障；
2. **Cyphal/UDRAL**：作为下一代 CAN 网络路线，与 DroneCAN 独立编译、独立配置；
3. **PWM pulse**：最简单、最通用的基础兼容路径；
4. **DShot**：只有取得可复用且许可清晰的官方/标准维护实现后才接入，不自写 wire codec；
5. **MAVLink**：用于配置、诊断、遥测和伴随计算机管理，不作为确定性实时电机命令总线。

所有第三方协议必须使用协议所有者的官方源码或官方生成器产物。FluxRT 只写平台 port、
最小 adapter、字段映射和产品安全接入；不自己实现第三方 wire codec、CRC、分片重组、节点
状态机或兼容栈。没有可复用官方实现时，对应 feature 保持关闭并延期。

## 2. 什么属于“可以自己写”

| 层 | 来源 | FluxRT 是否实现 |
|---|---|---|
| CAN/UART/SPI/定时器/DMA 驱动 | STM32 HAL/RT-Thread/板级代码 | 是，只负责原始帧和硬件状态 |
| 第三方 wire codec、CRC、传输分片、协议状态机 | 协议官方上游/官方生成器 | 否，不重写 |
| 官方栈平台 port | 官方栈定义的 callback/API | 是，保持薄层和可替换 |
| 官方消息到 `ProductCommand` 的映射 | FluxRT adapter | 是，有界、可测试、无硬件写入 |
| 来源权限、sequence、lease、timeout、抢占 | FluxRT `CommandArbiter` | 是，所有协议共用 |
| arm、Axis 状态、fault、限值、PWM 许可 | FluxRT `MotorService`/安全层 | 是，第三方栈不能绕过 |
| PX4 参数和 HIL 兼容测试 | PX4 官方工程 + FluxRT fixture | 是测试，不把 PX4 源码嵌入固件 |

“不自己实现协议”不等于把官方源码散落复制进工程。上游代码和 FluxRT adapter 必须能够
分别升级、构建、测试和回退。

## 3. 依赖隔离目录

```text
third_party/
  README.md                         上游引入与升级规则
  upstream/<component>/             Git submodule；官方源码，零本地修改
  generated/<protocol>/             官方生成器输出，不手改
  licenses/                         上游许可证副本与NOTICE
  upstreams.catalog.json            官方URL、候选tag/commit、用途、状态

protocol/
  ports/<protocol>/                 RT-Thread/HAL transport port
  adapters/<protocol>/              官方消息 <-> FluxRT产品命令/状态
  mappings/<protocol>/              字段、单位、状态、fault映射说明
  golden/<protocol>/                官方样本与FluxRT互操作向量
```

每个官方栈独立生成静态库，并使用私有 include path。`foc-control`、`foc-algorithm`、
MotorService 和平台 PWM 代码禁止直接 include 第三方头文件。跨边界只传固定容量的
`ProductCommand`、状态快照和原始 transport 统计。

如果确实发现必须修补上游，默认处理顺序是：向上游提交修复、等待新版本、暂时禁用；不得
直接魔改 submodule。只有安全问题且没有替代方案时，才允许在单独 `patches/` 保存最小补丁，
记录上游 issue、应用顺序和删除条件，并在每次升级时重新验收。

## 4. 上游选择与锁定

真正开始某协议的实现批次时执行以下流程：

1. 从官方仓库重新查询最新稳定 tag；滚动仓库选择当前经过评审的完整 commit；
2. 审计许可证、生成器和传递依赖，记录 URL、tag、commit、内容哈希和日期；
3. 以 submodule 或隔离的生成目录引入，禁止复制到 `applications/` 或 `rust/crates/`；
4. 保持 upstream 零修改，FluxRT 差异全部放在 port/adapter；
5. 使用官方测试 + FluxRT golden/fuzz/Host/目标构建验证；
6. 升级一次只改变一个上游引用，协议兼容、ROM/RAM、栈、WCET和总线预算均需重新检查。

当前观察到的官方候选及精确 commit 记录在
[`third_party/upstreams.catalog.json`](../third_party/upstreams.catalog.json)。它是 2026-10-01
的发现快照，不表示已经下载、许可批准或进入固件；开始集成时必须刷新。

### 4.1 RT-Thread 包目录审计结论

- 本机 RT-Thread 包目录中的 MAVLink 指向第三方镜像，不满足“协议所有者官方来源”规则；
- 包目录中的 CanFestival 不是本项目选定的官方 CANopenNode 路线；
- SOEM 是 EtherCAT **主站**，FluxRT 驱动器需要的是从站栈，不能错用；
- 因此不能只凭 RT-Thread 有 package 就直接采用，必须核对来源、角色、许可证和版本。

选定的候选路线是 DroneCAN 官方 libcanard/DSDL、OpenCyphal 官方 libcanard/regulated DSDL、
MAVLink 官方生成 C 库、CANopenNode 官方栈和 SOES 官方从站栈。各自仍需在实际集成批次完成
许可证和目标资源审计。

## 5. PX4 对接架构

```text
PX4 mixer / control allocation
        │
        ├─ PWM pulse ───────────────┐
        ├─ DShot (gated) ───────────┤
        ├─ DroneCAN RawCommand ─────┤
        ├─ Cyphal/UDRAL setpoint ───┤
        └─ MAVLink management ──┐   │
                                │   v
                         protocol/input adapter
                                │   │
                                │   v
                         ProductCommand + source_id
                                    │
                                    v
                     CommandArbiter → MotorService → FOC
                                    │
                                    v
                       status/fault/telemetry adapter
```

### 5.1 DroneCAN：PX4 第一优先 CAN 路线

PX4 官方支持 DroneCAN ESC，通过 `uavcan.equipment.esc.RawCommand` 下发带符号的归一化命令，
并读取 ESC 状态。FluxRT 的映射边界为：

| DroneCAN/PX4 语义 | FluxRT 处理 |
|---|---|
| ESC index/node ID | 静态配置到 Axis/source，不在运行中猜测 |
| `RawCommand` | adapter 校验长度/范围后转换为 Torque 或归一化命令 |
| 消息时间和 transfer ID | 转为 sequence/freshness，不能直接当设备单调时钟 |
| arm/disarm | 只形成外部许可请求；不能绕过本地 Disabled/Armed/Fault 状态机 |
| command timeout | 来源失效并 Release/受控停机，不复活旧命令 |
| ESC status | 回报 rpm/current/voltage/temperature/fault/health |

PX4 官方说明 ESC 消息可能占用较高 CAN 带宽，所以产品化时优先给电机控制使用独立 CAN 总线，
并在多节点、状态频率和总线错误场景下测量利用率。

### 5.2 Cyphal/UDRAL：第二优先现代 CAN 路线

PX4 当前已有 Cyphal 驱动和 UDRAL actuator 类型。FluxRT 使用 OpenCyphal 官方 libcanard 和
官方 DSDL 生成物，独立于 DroneCAN 编译。两条协议可以共享 CAN platform driver，但第一版
同一个 CAN 控制器在运行时只能选择其中一种控制协议，避免 filter、node ID、带宽和 owner
语义冲突。

### 5.3 PWM 与 DShot

PWM pulse 由 FluxRT 自己实现输入捕获与归一化，因为这是板级输入，不是复制第三方协议栈；
仍要经过 timeout、标定、`ProductCommand` 和本地 arm 门。

DShot 包含帧编码、校验、命令和可选遥测。按本项目规则，不在缺少合适官方/标准维护嵌入式
接收实现时临时自写。P2.5 只保留 capability、资源和测试接口；找到并审计上游后再启用。

### 5.4 MAVLink

MAVLink 适合 PX4/GCS/伴随计算机的配置、身份、诊断、遥测和日志，不承担 FOC 实时 setpoint。
设备端只采用 MAVLink 官方生成头文件，并对写操作增加签名/权限、事务、速率和状态限制。
即使 MAVLink 链路在线，也不自动获得 Axis 控制租约或 arm 权限。

## 6. VESC、CANopen 与 EtherCAT 决策

### 6.1 原生 VESC CAN

VESC 官方 CAN 行为主要存在于完整 GPLv3 固件的 `comm_can.c` 等模块中，没有当前已确认的、
边界干净且可直接升级的官方独立协议 SDK。FluxRT 不复制该文件，也不自写“兼容版”。

当前决定：

- 原生 VESC CAN feature 延期；
- PX4 与 VESC/FluxRT 的统一 ESC 网络优先使用 DroneCAN；
- 后续只有在出现官方独立 SDK，或完成明确许可证和隔离方案评审后，才重新开启任务。

### 6.2 CANopen/CiA 402

使用 CANopenNode 官方栈。FluxRT 只实现 STM32/RT-Thread port、对象字典生成输入、CiA 402 到
MotorService 的 adapter 和安全映射，不另写 CANopen 栈。

### 6.3 EtherCAT

驱动器是 EtherCAT 从站，因此候选为官方 SOES，而不是 SOEM 主站。SOES 的许可证和链接例外
必须先做发布审查；当前 STM32G431 板没有 EtherCAT Slave Controller，目标功能保持关闭，
等待带外部 ESC 或原生从站硬件的新平台。

## 7. 分阶段任务

| 阶段 | 内容 | 完成门 |
|---|---|---|
| P2.4C | 本文、上游目录规则、候选 catalog、PX4边界 | S0 文档/路径/JSON检查；不下载、不链接 |
| P2.5A | PWM/Analog/Step-Dir | Fake输入、timeout、资源冲突、默认关闭目标构建 |
| P2.5B | DShot上游选择门 | 找到许可清晰、可复用的官方/标准维护实现；否则继续延期 |
| P2.6 | FluxRT Native UART/USB/CAN | A帧层已完成Rust单一codec、schema/golden/CRC/resync；后续payload、Fake service、transport、bus-off和多节点；始终与第三方栈隔离 |
| P2.7A | PX4 DroneCAN ESC | 官方上游锁定、RawCommand/Status映射、PX4 SITL/HITL/HIL兼容测试 |
| P2.7B | PX4 Cyphal/UDRAL | 官方上游锁定、UDRAL readiness/setpoint/feedback映射 |
| P2.7C | PX4 MAVLink管理 | 官方生成头、只读优先、签名/权限、配置事务；不进快环 |
| P2.7D | CANopen/CiA 402 | CANopenNode、对象字典、状态机与MotorService映射 |
| P2.8 | EtherCAT从站 | SOES、ESC硬件、PDO/CoE/DC/watchdog；专用板验证 |
| P2.9 | 上位机所需设备消息 | 冻结capability/config/status/statistics消息；仍不开发GUI |

原生 VESC CAN 不进入当前关键路径。P2.7A/P2.7B 是 FluxRT 对接 PX4 的正式 CAN 任务，
上位机仍在最后的 P7.1 实现。

## 8. 证据边界

- P2.4C 只有 S0 架构与上游发现证据；
- submodule 下载和目标链接通过最多是 S1/S2，不等于与 PX4 互操作；
- PX4 SITL/Host 回放是 S3，不等于 CAN 物理层或功率级通过；
- 无功率板端 CAN 收发是 S4；
- 限流电机运行、多节点断线恢复和独立时序测量分别进入 S5/S6。

任何第三方协议失败都只关闭对应 feature，不能改变 FluxRT 基本 FOC、硬件 fault 和本地停机
路径。
