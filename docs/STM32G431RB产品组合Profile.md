# STM32G431RB 产品组合 Profile

## 1. 为什么必须分 Profile

STM32G431RBT6 只有 128 KiB Flash。当前目标构建已经证明：

- `Advanced + Power` 虽能链接，但只比 9 KiB 运行堆底线多 24 B，不能作为产品组合；
- `Power + Native + Analog` 的历史构建只剩约 1.6 KiB Flash；
- `Advanced + Power + Native + Analog` 超出 Flash 约 6.6 KiB。

因此 G431RB 不再允许“把所有候选都勾上，等链接器最后报错”。工程在 Kconfig、CMake
和 C 预处理三个位置执行同一份互斥规则，在编译早期拒绝未经容量验证的组合。

Profile 只表示某类代码**允许编入**当前镜像，不表示运行时已经启用，更不表示电机可以
arm。运行时配置、平台能力、参数审批、故障状态和硬件保护仍要逐层通过。

## 2. 三个相互独立的选择轴

| 选择轴 | 解决的问题 | 配置位置 | 示例 |
|---|---|---|---|
| 主构建档 | 操作者/试验能力 | `build.ps1 -Profile`、CMake cache | Diagnostic、Calibration、Identification、Production |
| G431产品组合 | 128 KiB内允许编入哪一类大功能 | Kconfig / `.config` | Basic Drive、Advanced Lab、Connected Lab |
| 运行时配置 | 已编入能力当前是否启用 | 参数事务和平台能力门 | MTPA enable、transport enable、input enable |

这三层不能互相替代。例如选择 `Advanced Lab` 仍必须显式打开高级候选，候选编入后
`enabled_features` 仍默认为 0；选择 `Connected Lab` 也不会自动创建 UART/CAN 驱动。

## 3. G431RB 的五个互斥包络

| Profile | 允许的主要候选 | 主构建档限制 | 用途与边界 |
|---|---|---|---|
| `Basic Drive` | 不允许 Advanced/Motion/Power/External-I/O 大候选 | 四个主构建档均可 | 基础 FOC、安全核心和当前唯一 release-capable 组合；仍不等于量产通过 |
| `Advanced Lab` | `FOC_ADVANCED_CONTROL_CANDIDATE` | 仅 Diagnostic | MTPA、弱磁、MTPV、解耦、调制策略、HFI/飞启候选的独立测量镜像 |
| `Motion Lab` | `FOC_MOTION_CONTROL_CANDIDATE` | 仅 Diagnostic | 轨迹、级联控制、单写者 dispatcher 和 combined realtime 的独立测量镜像 |
| `Power Lab` | `FOC_POWER_MANAGEMENT_CANDIDATE` | 仅 Diagnostic | 热、母线、回馈与制动请求策略；不声明真实传感器或制动硬件 |
| `Connected Lab` | External-I/O、Native，以及最多一种 PWM/Analog/Step-Dir 输入 | 仅 Diagnostic | 通信和外部输入的软件/物理接口验证；不自动提供物理 transport |

没有 `Full Product` Profile。需要高级控制、功率管理和多协议同时常驻时，应迁移到更大
Flash/RAM 的 MCU，再为新目标建立独立容量、WCET 和硬件证据，不能放宽 G431RB 的门。

## 4. 机器门禁

### 4.1 Kconfig 可选性

`board/Kconfig` 先选择唯一 `FLUXRT_G431_PROFILE_*`。各大候选只有在对应包络下才显示，
默认仍为关闭。切换包络时，Kconfig 会清除不再满足依赖的旧候选。

### 4.2 CMake 配置门

`custom.cmake` 从生成的 `rtconfig.h` 读取 Profile 和候选，调用
`cmake/FluxRTG431ProductProfile.cmake`。以下情况在 Cargo/链接前直接失败：

- 没有选择或同时选择多个 G431 Profile；
- Lab Profile 配合 Calibration、Identification 或 Production；
- 候选逃出自己的允许包络；
- Native/简单输入没有 External-I/O 总开关；
- Connected Lab 同时编入两种或更多简单输入。

### 4.3 C/直接 SCons 门

`foc/include/foc_product_profile.h` 重复执行同一约束，防止绕过 CMake 的直接 SCons 或
手工编译产生非法目标镜像。通用 Host 能力穷举不模拟 G431 物理容量，因此只有检测到
`SOC_STM32G431RB` 或显式测试某个 G431 Profile 时才启用该板级互斥门。

## 5. 如何切换

在 RT-Thread Env 环境中：

```powershell
cd E:\File\RT-Thread\projects\FluxRT
scons --menuconfig
```

进入：

```text
Hardware Drivers Config
  -> STM32G431RB product composition
```

先选一个 Profile，再在对应候选菜单中选择需要编入的候选。保存后必须重新生成：

```powershell
.\build.ps1 -Regenerate -Profile Diagnostic -RustOptLevel s
```

不要直接修改 `rtconfig.h`，它是生成文件。切换 G431 Profile 或候选后也不要直接执行旧
构建目录的 `-BuildOnly`；必须先 `-Regenerate`，因为 SCons 还负责源文件集合。

若要回到日常安全基线，选择 `Basic Drive`，关闭所有大候选，再执行：

```powershell
.\build.ps1 -Regenerate -Profile Diagnostic -RustOptLevel s
.\build.ps1 -BuildOnly -Profile Diagnostic -RustOptLevel s
```

启动串口中保留原有 `FBOOT,p=<主构建档>,o=<优化等级>`，并新增：

```text
FBOOT,fp=basic-drive
```

Lab 镜像会分别打印 `advanced-lab`、`motion-lab`、`power-lab` 或 `connected-lab`。

## 6. 修改或新增 Profile 的规则

新增功能不能直接塞进现有包络。至少先完成：

1. Host/Rust 功能与负例测试；
2. Cortex-M4F 静态库和 STM32 目标链接；
3. ROM、RAM、运行堆和栈预算；
4. 快环相关功能的逐拍/WCET 测量；
5. 默认关闭、故障回退和运行时 capability 门；
6. 本文、任务表和工程操作日志同步更新。

更换 MCU 时保留 Profile 的语义，不复用 G431 的容量结论。新平台应新增自己的产品组合
约束；容量充足也只能在重新取得 S1～S4 证据后建立 Full 组合。

## 7. 当前验证结论

2026-10-03 的软件验证：

- Profile 正反例 Python：6/6 测试通过，覆盖五种合法包络及非法逃逸组合；
- Host C：51/51 通过，覆盖四个主构建档、Advanced/Motion/Power/Connected Profile ID；
- 统一 `test.ps1`：Profile 153、Simulation 47、Rust workspace/Clippy/ARM archive 全部通过；
- `Basic Drive` 四主构建档目标链接通过：Diagnostic 111,828 B、Calibration 104,172 B、
  Identification 117,012 B、Production 102,896 B ROM。

以上是 S1/S2 软件和交叉构建证据。本次没有烧录、没有板端启动、没有给功率级上电、
没有测量新的 WCET，也没有运行电机。
