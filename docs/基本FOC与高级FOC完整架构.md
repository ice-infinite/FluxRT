# FluxRT 基本 FOC 与高级 FOC 完整架构

## 1. 结论

FluxRT 现在把 FOC 分成三个互不混淆的层次：

1. **基本 FOC 数据通路**：每个控制拍必经，负责把电流给定变成合法 PWM；
2. **高级 FOC 策略层**：根据工作区修改 `Id/Iq`、增加解耦前馈并选择调制策略；
3. **硬件能力与安全层**：决定某项高级策略是否允许接到真实 PWM/ADC。

基本 FOC 是正式实时主链；高级层已经有统一 supervisor、固定内存状态、版本化配置、
实时接线、Rust PC/MATLAB 双仿真和目标交叉构建。所有高级功能默认关闭，启用也不等于
已经完成当前电机的参数整定或实机批准。

## 2. 一拍控制链

```text
ADC/母线/故障快照（C平台）
        │
        v
Clarke + 观察器/启动/反馈选择（Rust control）
        │
        v
Torque/Velocity/Position → 基础 Id/Iq 给定
        │
        v
AdvancedFocSupervisor
  ├─ MTPA / 弱磁 / MTPV：修改 Id/Iq
  ├─ dq 解耦：生成 Vd/Vq 前馈
  ├─ HFI：生成 αβ 注入候选
  ├─ DPWM / 过调制：选择输出策略
  └─ 飞车启动：只生成被动捕获候选
        │ CurrentLoopPolicy
        v
Park → Id/Iq PI → 前馈 → 电压圆 → 逆Park → 注入 → 最终电压圆
        │
        v
SVPWM / DPWM → 占空比合法性 → 逆变器补偿 → C平台输出复核 → CCR
```

高级层不写寄存器、不认识 RT-Thread，也不能自行打开 Gate。C 平台仍独占 ADC、PWM、
Break、deadline 和立即关断；Rust 只返回数值决策。

## 3. 基本 FOC 能力

| 能力 | 实现位置 | 当前接线 |
|---|---|---|
| 三相电流 Clarke | `foc-algorithm` / `controller.rs` | 实时主链 |
| Park / 逆 Park | `controller.rs` | 与同拍角度、角度延迟补偿共用一次数学事务 |
| Id/Iq 双 PI | `controller.rs` | 每控制拍运行，分别保存积分状态 |
| PI 输出抗饱和 | `PiState` | 单轴限幅回算 |
| 矢量电压圆限幅 | `CurrentLoop` | 使用实测母线，默认 `Vbus/sqrt(3)` 线性区 |
| 矢量限幅回算 | `CurrentLoopPolicy` | 高级前馈启用时可回算两个 PI |
| SVPWM | `foc-algorithm` | 默认调制器 |
| 速度外环 | `SpeedLoop` | 分频运行，输出 `Iq` |
| Torque/Velocity/Position 串级 | `motion*` | 独立默认关闭候选；不复制 FOC 内环 |
| Rev-Up/无感接管 | `startup.rs` + `observer.rs` | 对齐、升速、保持、混合、闭环 |
| SMO/BEMF-PLL | `observer.rs` | 可靠性门、获取/保持门和失锁超时 |
| 死区/器件压降模型 | `voltage.rs` | 观察器电压修正与 PWM 前馈独立开关 |
| 最终输出安全检查 | bridge + C platform | 非有限/越界立即锁存故障，不下发 |

基本链的关键回归是：`CurrentLoopPolicy::default()` 与改造前入口逐位一致。高级候选即使
被编译，只要 `enabled_features=0`，也不会预计算额外 Park、不会改变给定和调制方式。

## 4. 高级 FOC 能力矩阵

| 功能 | 算法/状态 | 工作区管理 | 目标实时接线 | 当前限制 |
|---|---|---|---|---|
| MTPA | 已实现 | 电流门、搜索步数、电流圆 | 已接 | 当前 GBM2804H 按 `Ld=Lq` 时自然退化为 `Id=0` |
| 弱磁 FW | 已实现 | 电压进入/退出滞环、Id 斜率、电流圆 | 已接 | PI/阈值需按实测参数整定 |
| MTPV | 已实现 | 电角速度进入/退出滞环、离散搜索 | 已接 | 当前电压模型忽略 `Rs*i`，只能作为高速候选 |
| dq 解耦前馈 | 已实现 | 增益 0～1.5、仅闭环 | 已接 | 依赖 `Ld/Lq/磁链/速度` 准确度 |
| DPWM | 已实现 | 调制度进入/退出滞环 | 已接策略入口 | 目标配置必须声明三分流电流重构能力；当前板尚未批准 |
| 过调制 | 已实现 | 独立进入/退出滞环，电压比上限 `<=2/pi` | 已接策略入口 | 必须先证明最小脉宽和 ADC 采样窗；当前板尚未批准 |
| 旋转 HFI | 已实现 | 低速门、显式注入请求、HPF/解调/收敛门 | PC/MATLAB策略门 | 目标逐拍请求未开放；`Ld≈Lq` 电机配置会被拒绝 |
| 飞车启动 | 已实现 | 被动扫描、连续可靠样本、超时、捕获状态 | PC/MATLAB状态机 | 只给候选角/速度，不自行接管或加电；目标请求未开放 |

“已实现”表示源码、状态、配置校验和主机测试存在；“目标已批准”必须另外经过 S2 size、
S4 无功率时序、S5 受限实机和相应故障注入。本表刻意不把这两个概念合并。

## 5. 工作区所有权与切换顺序

`AdvancedFocSupervisor` 是高级策略的唯一 owner，组合顺序固定：

```text
基础 Id/Iq
  -> 电流圆
  -> MTPA
  -> 弱磁（带滞环和斜率）
  -> MTPV（带速度滞环，成功搜索才接管）
  -> 最终电流圆
  -> dq 解耦前馈
  -> 调制方式（过调制优先于 DPWM）
  -> HFI/飞车候选
```

这样避免 MTPA、弱磁和 MTPV 各自直接写 `Id`，也避免 DPWM 与过调制同时争夺调制器。
配置非法时采用事务语义：旧配置和状态保持不变；实时输入非法则锁存
`FOC_FAULT_ADVANCED_CONTROL` 并停止输出。

目标高级入口另有明确的故障回退：观察器暂时不可靠时，高级层停止修改工作区并退回基础
FOC输出；输入非有限、平台能力丢失、supervisor失败或PWM非法时，输出归零并返回原因码。
回退不清除锁存故障、不自行重新arm，也不绕过C平台的最终安全门。

## 6. 配置与 ABI

- 全局 bridge：V22 / `0x00160000`；运行配置V12 / 300 B；V19 的88 B逐拍输入布局不变；
- 高级子 ABI：V2 / `0x00020000`；
- `foc_advanced_algorithm_config_t`：128 B；
- `foc_advanced_runtime_config_t`：148 B，新增平台有效`minimum_duty/maximum_duty`窗口；
- `foc_advanced_telemetry_t`：68 B；
- 配置顺序：`foc_rust_configure()` 基础参数 →
  `foc_rust_default_advanced_config()` → 修改字段 →
  `foc_rust_configure_advanced()`；
- 只允许 `Disabled + fault=0 + 基础配置有效` 时应用；运行中热改一律拒绝；
- 重新应用基础电机/频率配置会自动把高级层恢复为默认关闭，防止旧高级参数套到新电机。

硬件相关功能还有第二道能力门：

- DPWM：`FOC_ADVANCED_CAP_DPWM_CURRENT_RECONSTRUCTION`；
- 过调制：`FOC_ADVANCED_CAP_OVERMOD_MIN_PULSE`；
- HFI：`FOC_ADVANCED_CAP_HFI_INJECTION`；
- 飞车启动：`FOC_ADVANCED_CAP_PASSIVE_FLYING_START`。

能力位表示平台实现具备相应机制，不表示参数已审批。当前目标 ABI 仍拒绝 HFI 和飞车启动，
因为尚无逐拍显式请求通道；这两项不能通过伪造 capability 绕过。

### 6.1 P5.4A 目标有界 owner

高级算法 supervisor 与“谁能打开功率输出”是两层不同所有权。P5.4A 增加
`foc_advanced_power_trial` 硬件中立状态机，STM32G431 平台负责把它绑定到真实启动、ISR和
关断寄存器：

```text
Shell精确token
  -> Advanced平台authority
  -> 私有闭环/582 rpm trial配置
  -> 正常startup + observer获取
  -> ClosedLoop且observer可靠
  -> 1200拍active窗
  -> ISR同拍关闭Gate/MOE/CCER
  -> 恢复Advanced disabled和Basic runtime
```

- Generic、Motion、Advanced 使用三个独立 start authority；Advanced Lab 的普通
  `foc_start` 不能 arm；
- Shell 当前只开放 `P54-BASIC-100MS`，不接受转速、电流、feature mask或时长参数；
- 状态机内部允许 feature=0 和单独 dq 解耦，但解耦 token 必须等基线 S5 通过后另行开放；
- startup 获取阶段允许 observer 暂未可靠；进入 active 后一旦回退立即停机；fault epoch、
  deadline miss、Advanced fault、超时和非法输出同样 fail-closed；
- 正常/非锁存退出恢复调用者 Basic 配置；若硬件或控制故障已经锁存，故障保持 sticky，
  owner 不会为了恢复配置而自动清故障，后续必须走显式安全恢复流程。

这套 owner 只建立了可测试的实机事务边界。S4 断主电源通过不代表 S5 powered、参数准确或
高级功能收益已经成立。

## 7. 文件落位

| 文件 | 责任 |
|---|---|
| `rust/crates/foc-algorithm/src/optimization.rs` | MTPA、MTPV、弱磁、V/f 叶算法 |
| `rust/crates/foc-algorithm/src/modulation.rs` | SVPWM、DPWM 等调制算法 |
| `rust/crates/foc-algorithm/src/observer/injection.rs` | HFI 注入/解调叶算法 |
| `rust/crates/foc-control/src/controller.rs` | 基本电流环与 `CurrentLoopPolicy` 最终组合 |
| `rust/crates/foc-control/src/advanced_foc.rs` | 高级工作区 supervisor、滞环、互锁、状态 |
| `rust/crates/foc-control/src/power_supervisor.rs` | 温度、母线、source/sink、再生与降额的纯策略监督器；默认关闭 |
| `rust/crates/foc-rt-bridge/src/advanced_abi.rs` | 高级配置/遥测 C ABI 和平台能力门 |
| `foc/include/foc_advanced_bridge.h` | C 侧定宽镜像和尺寸断言 |
| `foc/include/foc_build_profile.h` | Diagnostic高级候选能力派生；普通构建不携带候选 |
| `applications/main.c` | 候选启动ABI自检、默认关闭配置提交/回读；不提供自动启用 |
| `simulation/scenarios/advanced_foc_matrix_v1.json` | Rust/MATLAB 共用 9 场景输入 |
| `rust/crates/foc-sim/src/advanced_contract.rs` | Rust PC 场景执行器 |
| `rust/crates/foc-sim/src/dynamic_plant.rs` | SPMSM/IPMSM、DC link、source/sink和负载阶跃的RK4动态plant |
| `rust/crates/foc-sim/src/full_speed.rs` | 复用产品速度环、电流环和高级supervisor的全速域闭环场景 |
| `simulink/run_advanced_foc_contract.m` | 独立 MATLAB 公式实现 |
| `simulation/run_advanced_foc_gate.ps1` | 两引擎执行与结果对比入口 |
| `simulation/run_full_speed_closed_loop_gate.ps1` | Rust动态plant与独立MATLAB动态模型的全速域批量门 |

热/母线/再生监督通过独立V1 ABI接入，但应用层默认disabled。它只产生降额、允许source/sink、
再生限制和force-safe决策；真实温度采集、制动电阻驱动及板级能量能力仍由C平台提供，缺少
这些硬件时不能把软件策略标为产品可用。

## 8. 验证方式与当前结果

```powershell
# 纯控制与桥接
& "$env:USERPROFILE\.cargo\bin\cargo.exe" test `
  --manifest-path rust\Cargo.toml -p foc-control --features advanced-foc
& "$env:USERPROFILE\.cargo\bin\cargo.exe" test `
  --manifest-path rust\Cargo.toml -p foc-rt-bridge --features advanced-foc

# Rust PC + MATLAB 独立公式对拍
.\simulation\run_advanced_foc_gate.ps1

# 目标候选（只构建，不代表实机批准）
.\build.ps1 -Profile Diagnostic -RustOptLevel s -Regenerate
```

2026-10-01 批量冻结后的当前证据：

- Rust默认workspace：332/332 PASS；`foc-control` 147/147；
- bridge关键全feature组合：63/63 PASS，Cortex-M4F release archive通过；
- C Host：50/50 PASS（含高级/功率候选与Native Fake driver合同）；
- Rust/MATLAB 高级策略矩阵：9/9 场景逐字段对拍 PASS；
- Rust/MATLAB全速动态：6场景/1256行PASS；再生换向最终误差约7.34 rpm；
- 完整`foc-sim`：49/49 PASS；
- STM32G431 Diagnostic 高级候选：交叉构建 PASS，ROM 120,044 B、RAM 23,200 B；
  `foc_rust_advanced_abi_version/default/configure/get_telemetry`四个入口均保留在ELF；
  Advanced+Power也能链接，但运行堆只高于9 KiB底线24 B；全候选同镜像超Flash 6592 B。
  没有烧录、没有串口操作、没有转电机。

## 9. 后续调参与实机顺序

架构完成后按功能逐项批准，不能一次打开全部位：

1. 用实测 `Rs/Ld/Lq/磁链` 更新电机 profile；
2. 先只开 dq 解耦，做前馈开/关仿真和受限电流阶跃；
3. IPMSM 才评审 MTPA；SPMSM 保持 `Id=0` 基线；
4. 做母线跌落/升速矩阵后单独批准弱磁；
5. MTPV 先补含 `Rs*i` 的候选模型并做高速可行域对比；
6. 完成三分流重构、最小脉宽和 ADC 窗验证后再开放 DPWM/过调制 capability；
7. 换有足够凸极性的电机后才评审 HFI，并增加极性判定；
8. 飞车启动完成“捕获 → 无扰预装 → 接管”事务后再接目标逐拍请求；
9. 每一步都分别记录 S1/S2/S3/S4/S5，失败时只回退当前 feature bit。

目前不需要为了继续写架构而转电机；实机调试从上述第 1 项参数审批开始另行执行。
