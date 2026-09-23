# 独立 RT-Thread 空工程创建与复用手册

本文记录 `FluxRT` 是如何从 RT-Thread 官方源码和 STM32G431 BSP 基线整理成独立工程的，并给出下一次创建同类空工程时可以直接执行的步骤。

本文面向以下开发方式：

- Windows PowerShell。
- VS Code 作为日常编辑、编译和调试入口。
- RT-Thread 标准版，而不是 RT-Thread Nano。
- Env、Kconfig 和 SCons管理配置及源码选择。
- CMake 和 Ninja作为 VS Code 的日常构建层。
- Rust 控制核心与具体 MCU、HAL、RTOS 解耦，硬件平台保留在 C。

> “编译通过”只证明配置、编译和链接成立，不代表晶振、串口、ADC、PWM、保护电路或电机已经在实物上验证。

## 1. 当前工程最终形成了什么

当前工程位置：

```text
E:\File\RT-Thread\projects\FluxRT
```

它没有复制或修改 RT-Thread 上游源码，而是引用同一工作区中的：

```text
E:\File\RT-Thread\rt-thread
```

最终目录关系如下：

```text
E:\File\RT-Thread
├─ rt-thread\                       RT-Thread 上游源码，保持干净
└─ projects\
   └─ FluxRT\                       独立产品工程
      ├─ applications\              RT-Thread 应用入口
      ├─ board\                     当前板级启动、时钟、内存和 MSP
      ├─ foc\
      │  ├─ include\                C/Rust ABI 类型和稳定硬件接口
      │  └─ platform\stm32g431\     STM32G431 硬件适配层
      ├─ rust\
      │  ├─ crates\foc-algorithm\  no_std 纯 Rust 算法库
      │  └─ crates\foc-rt-bridge\  状态机、快环和 C ABI staticlib
      ├─ tests\host\                PC 主机测试
      ├─ docs\                      架构、移植和硬件文档
      ├─ .vscode\                   VS Code 任务和调试配置
      ├─ Kconfig / .config           RT-Thread 功能配置源
      ├─ SConstruct / SConscript     SCons 构建入口和源码清单
      ├─ rtconfig.py                 GCC、CPU、链接和产物配置
      ├─ build.ps1                   一键生成和构建脚本
      ├─ custom.cmake                Cargo 静态库构建依赖（不会被生成覆盖）
      └─ test.ps1                    一键主机测试脚本
```

构建后还会产生以下目录或文件，它们不属于手写源码：

```text
packages\                 Env 下载的 CMSIS/HAL 包
CMakeLists.txt            SCons 自动生成
cmake-build\              ARM CMake/Ninja 构建目录
build\host-tests\         PC 主机测试构建目录
build\rust-target\        Cortex-M4F Rust 静态库目录
build\rust-test-target\   Rust 主机测试目录
rtconfig.h                 根据 .config 生成
```

其中 `packages`、构建目录和生成的 `CMakeLists.txt` 已加入 `.gitignore`。

## 2. 当前工程采用的构建职责划分

不要把 SCons 和 CMake 当成两套互相独立的配置系统。当前工程中的职责是：

```text
Kconfig + .config
        ↓
SCons 解析 RT-Thread 组件、BSP 驱动和软件包
        ↓
生成 rtconfig.h 与 CMakeLists.txt
        ↓
CMake + Ninja 调用 Cargo 并编译 C
        ↓
C 对象 + libfoc_rt_bridge.a
        ↓
rtthread.elf + fluxrt.bin/.hex/.map
```

具体原则：

1. `Kconfig`、`.config`、`SConscript` 和 `rtconfig.py` 是配置与构建事实来源。
2. 根目录 `CMakeLists.txt` 是生成文件，不应手工长期维护。
3. 新增源文件、修改 Kconfig 或修改软件包后，必须重新生成 CMake。
4. 仅修改已进入工程的 `.c/.h` 文件时，可以直接增量编译。
5. VS Code 通过 `cmake-build/compile_commands.json` 获得准确的宏和头文件路径。
6. `custom.cmake` 是手写文件，专门补充生成器不知道的 Rust 构建依赖，不能删除。

## 3. 当前可复现基线

记录日期：2026-09-22。

### 3.1 硬件基线

| 项目 | 当前配置 |
|---|---|
| MCU | STM32G431RBT6，Cortex-M4F |
| Flash | 128 KiB |
| SRAM | 32 KiB |
| 初始控制板 | NUCLEO-G431RB |
| 初始功率板 | X-NUCLEO-IHM16M1，计划使用三分流 |
| 初始电机 | GIMBAL GBM2804H-100T |
| HSE | 板载 X3 24 MHz 无源晶体，`RCC_HSE_ON` |
| SYSCLK | `24 / 6 × 85 / 2 = 170 MHz` |
| LSE | 板载 X2 32.768 kHz 晶体 |
| RTC 时钟源 | LSE，RTC 设备驱动暂未启用 |
| 临时控制台 | LPUART1，PA2/TX、PA3/RX |

### 3.2 软件与工具基线

| 项目 | 已验证版本或修订 |
|---|---|
| RT-Thread 分支 | `master` |
| RT-Thread 提交 | `ed686ba95b5e8fb45910efc638544e43ffba89e2` |
| Arm GNU Toolchain | 15.3.1 |
| CMake | 4.4.3 |
| Ninja | 1.13.2 |
| SCons | 4.11.1 |
| Cargo | 1.98.1 |
| rustc | 1.98.1 |
| Rust target | `thumbv7em-none-eabihf` |
| CMSIS-Core 包 | `39d8e01f0be84b83a8f11d33756e82ce1ef07a84` |
| STM32G4 CMSIS 包 | `ac09c63bc8dc5f9e9ce9e54cba71c8b8f1d0f9ee` |
| STM32G4 HAL 包 | `d1d1f195c4e099e41ec9011e13b365234de83918` |

当前配置使用 `latest` 软件包选择项。以后重新执行 `pkgs --update` 时，上述包提交可能变化。如果要做可发布版本，应额外记录实际提交或改成固定版本。

## 4. 为什么没有直接修改官方 BSP

官方已经提供：

```text
rt-thread\bsp\stm32\stm32g431-st-nucleo
```

这个 BSP 很适合点亮开发板和确认 RT-Thread 基本运行，但产品工程还需要：

- 独立于上游源码进行版本管理。
- 避免更新 RT-Thread 时混入产品代码。
- 把控制算法和 STM32 HAL 分开。
- 给未来国产 MCU 留出新的平台适配目录。
- 增加主机测试，而不是所有测试都依赖开发板。
- 给 VS Code 提供稳定的一键构建入口。

因此当前工程引用上游 RT-Thread 和公共 STM32 HAL 驱动，只把产品自己的板级、应用和 FOC 结构放在 `projects/FluxRT` 中。

## 5. 新建工程前必须先确认的信息

不要先复制代码再猜硬件。至少记录以下内容：

```text
工程名称：
MCU 完整料号：
CPU 内核和 FPU：
Flash 起始地址和容量：
RAM 起始地址和容量：
HSE 类型：无源晶体 / 有源时钟 / ST-LINK MCO / 不使用
HSE 频率：
LSE 是否存在：
目标 SYSCLK：
控制台外设和引脚：
调试器：ST-LINK / J-Link / 其他
安全输出默认电平：
参考的官方 BSP：
```

必须区分：

- 无源晶体使用 `RCC_HSE_ON`。
- 外部有源时钟或 ST-LINK MCO 通常使用 `RCC_HSE_BYPASS`。
- 晶振频率相同，不代表两种模式可以混用。

## 6. 推荐方法：复制当前安全骨架

如果新项目也需要 RT-Thread、VS Code、CMake/Ninja、主机测试和可移植业务层，最快的方法是复制当前工程的手写部分。

### 6.1 设定源工程和目标工程

下面以创建 `NEW_MOTOR_PROJECT` 为例：

```powershell
$sourceProject = 'E:\File\RT-Thread\projects\FluxRT'
$targetProject = 'E:\File\RT-Thread\projects\NEW_MOTOR_PROJECT'

if (Test-Path -LiteralPath $targetProject) {
    throw "目标目录已经存在：$targetProject"
}
```

不要把目标设置成 `E:\File\RT-Thread`、`projects` 或其他已有工程根目录。

### 6.2 复制源码，排除生成目录

Windows 自带 `robocopy`，适合复制目录并排除构建产物：

```powershell
robocopy $sourceProject $targetProject /E `
    /XD build cmake-build packages __pycache__ `
    /XF CMakeLists.txt .sconsign.dblite .config.old *.elf *.bin *.hex *.map *.pyc

if ($LASTEXITCODE -ge 8) {
    throw "robocopy 失败，退出码：$LASTEXITCODE"
}
```

`robocopy` 的 0～7 通常属于成功或存在差异，8 及以上才视为失败。

复制后确认生成目录没有被带过去：

```powershell
$generated = @(
    "$targetProject\packages",
    "$targetProject\cmake-build",
    "$targetProject\build",
    "$targetProject\CMakeLists.txt"
)

$generated | Where-Object { Test-Path -LiteralPath $_ }
```

正常情况下不应输出任何路径。

### 6.3 搜索需要改名的标识

进入新工程：

```powershell
Set-Location $targetProject
rg -n "FluxRT|fluxrt|STM32G431|FOC:" . `
    -g '!packages/**' -g '!cmake-build/**' -g '!build/**'
```

至少检查并修改：

| 位置 | 需要修改的内容 |
|---|---|
| `SConstruct` | `TARGET`、缺失包列表、提示文字 |
| `Kconfig` | 主菜单、SoC 选择和板级标识 |
| 根 `SConscript` | MCU 宏、平台宏 |
| `rtconfig.py` | map/bin/hex 名称、CPU 参数和链接脚本 |
| `board/board.c` | 晶振、PLL、总线和外设时钟 |
| `board/board.h` | Flash、SRAM 和堆边界 |
| `board/linker_scripts/link.lds` | ROM/RAM 起始地址和容量 |
| `board/stm32g4xx_hal_conf.h` | HSE/LSE 值和 HAL 模块 |
| `board/stm32g4xx_hal_msp.c` | 控制台 GPIO、复用和时钟 |
| `foc/SConscript` | 当前平台源文件路径 |
| `.vscode/launch.json` | MCU 型号、ELF 路径和调试器 |
| `.vscode/tasks.json` | 任务显示名称，可选 |
| `README.md` 和 `docs` | 工程名称、板卡和安全边界 |

完成后再次运行同一条 `rg`，确认没有不应保留的旧工程名称。

### 6.4 重新确认 CPU 编译参数

当前 STM32G431 使用：

```text
-mcpu=cortex-m4
-mthumb
-mfpu=fpv4-sp-d16
-mfloat-abi=hard
```

这些参数位于 `rtconfig.py`。如果换成没有 FPU 的 MCU、Cortex-M0/M3/M33 或 RISC-V，不能继续照抄。

同时确认根 `SConscript` 中的设备宏。例如当前为：

```python
env.Append(CPPDEFINES = [
    'STM32G431xx',
    'FOC_TARGET_STM32G431',
])
```

宏必须与 CMSIS 设备头文件要求一致。

### 6.5 重做链接内存范围

当前链接脚本是：

```text
ROM：0x08000000，128 KiB
RAM：0x20000000，32 KiB
```

需要同时核对三个位置：

1. `board/linker_scripts/link.lds` 的 `MEMORY`。
2. `board/board.h` 的 Flash/SRAM 宏和堆结束地址。
3. MCU 数据手册中完整料号对应的存储容量。

不要只根据同一系列最大容量填写。例如 `STM32G431x6`、`x8` 和 `xB` 的 Flash 容量不同。

### 6.6 重做时钟配置

当前 NUCLEO-G431RB 使用：

```text
HSE = 24 MHz
PLLM = 6
PLLN = 85
PLLR = 2
SYSCLK = 24 / 6 × 85 / 2 = 170 MHz
```

对应代码位于 `board/board.c`，频率宏位于 `board/stm32g4xx_hal_conf.h`。

新项目需要检查：

1. HSE 是否实际焊接。
2. HSE 是晶体还是旁路输入。
3. PLL 输入频率、VCO 输出频率和 SYSCLK 是否都在数据手册范围内。
4. 电压档位和 Flash 等待周期是否匹配最高主频。
5. APB 分频是否会影响定时器实际时钟。
6. LSE 驱动能力是否适合所选 32.768 kHz 晶体。
7. 时钟起振失败时是否安全停止，而不是继续运行功率输出。

当前工程还加入了编译期检查：

```c
#if (HSE_VALUE != 24000000UL)
#error "FluxRT STM32G431 reference target requires a 24 MHz external HSE crystal"
#endif
```

新项目应把这个检查改成自己的实际频率，不能简单删除。

### 6.7 选择最小 bring-up 外设

空工程只开启验证启动所需的最小功能：

- GPIO。
- 一个控制台 UART。
- RT-Thread 设备框架。
- FinSH/MSH，方便执行 `ps` 等命令。

当前控制台配置为：

```text
设备名：lpuart1
TX：PA2
RX：PA3
```

它由以下文件共同决定：

```text
board/Kconfig
.config
board/stm32g4xx_hal_msp.c
```

只改其中一个位置通常不够。

FOC 空工程暂不启用 PWM、ADC、OPAMP、COMP、编码器和真实 DMA 控制链路，避免尚未确认引脚时误输出。

### 6.8 保留安全空实现

业务核心与硬件适配之间只通过 `foc_platform.h` 连接：

```c
foc_status_t foc_platform_init(void);
void foc_platform_emergency_stop(void);
foc_status_t foc_platform_read_feedback(foc_feedback_t *feedback);
foc_status_t foc_platform_apply_output(const foc_output_t *output);
```

新工程在真实硬件没有闭合前应保持：

- `foc_platform_init()` 返回 `FOC_STATUS_NOT_CONFIGURED`。
- C 平台未就绪时 `foc_rust_request_start()` 不进入运行状态。
- Rust 快环在错误返回前清零输出。
- 平台输出函数不访问 PWM 寄存器。
- `main()` 首先调用 `foc_platform_emergency_stop()`。

不要为了“先跑起来”直接把返回值改成成功。只有硬件失能、PWM、ADC 同步、过流保护和故障测试都完成后，才允许进入运行状态。

### 6.9 配置软件包

当前 `.config` 选择了：

```text
CMSIS-Core
STM32G4 CMSIS Driver
STM32G4 HAL Driver
```

`SConstruct` 的 `bsp_pkg_check()` 会在这些目录缺失时阻止构建：

```text
packages/CMSIS-Core-latest
packages/stm32g4_cmsis_driver-latest
packages/stm32g4_hal_driver-latest
```

换 MCU 系列时必须同时修改：

1. `.config` 软件包选择。
2. `SConstruct` 的必需包检查。
3. `build.ps1` 的必需包检查。
4. `board/SConscript` 或厂商库入口。
5. HAL 配置头和启动文件来源。

### 6.10 第一次生成与编译

先打开 `build.ps1` 和 `test.ps1`，确认工具根目录：

```powershell
$envRoot = 'D:\Environment\env-windows-latest'
```

如果新电脑没有这个目录，应把 `$envRoot` 改成实际 Env 位置，或者确保以下命令已经加入 `PATH`：

```text
cmake
ninja
scons
pkgs
arm-none-eabi-gcc
cargo
rustc
```

`build.ps1` 会优先使用已知 Env/STM32CubeCLT 路径，找不到时再查找 `PATH`。`rtconfig.py` 中的 `EXEC_PATH` 只是未设置 `RTT_EXEC_PATH` 时的后备值；正常一键构建会由 `build.ps1` 写入真实 GCC 目录。

在新工程根目录执行：

```powershell
.\build.ps1 -UpdatePackages -Regenerate
```

脚本按以下顺序工作：

1. 定位 CMake、Ninja、SCons、`pkgs`、Arm GCC、Cargo 和 rustc。
2. 设置 `RTT_ROOT`、`RTT_EXEC_PATH`、`ENV_ROOT` 和包索引路径。
3. 先执行 `scons --pyconfig-silent`，同步 `.config` 并生成 `rtconfig.h`。
4. 执行 `pkgs --update`，下载芯片 CMSIS/HAL 包。
5. 再次生成 `rtconfig.h`。
6. 执行 `scons --target=cmake -s`，生成根 `CMakeLists.txt`。
7. 使用 CMake 和 Ninja 配置 `cmake-build`。
8. `custom.cmake` 调用 Cargo 生成 M4F hard-float Rust 静态库。
9. 编译 C 并链接 C 对象与 Rust archive。
10. 生成 ELF、BIN、HEX 和 MAP。

第一次成功后，日常只需要：

```powershell
.\build.ps1 -BuildOnly
```

强制重新生成：

```powershell
.\build.ps1 -Regenerate
```

仅配置、不编译：

```powershell
.\build.ps1 -Regenerate -ConfigureOnly
```

清理 ARM CMake 和 Rust target 构建目录：

```powershell
.\build.ps1 -Clean
```

### 6.11 运行 C / Rust 测试

执行：

```powershell
.\test.ps1
```

该脚本依次验证：

- `foc-algorithm` 的 66 个算法测试。
- `foc-control` 的参数、圆限幅、启动序列和 BEMF+PLL 测试。
- `foc-rt-bridge` 的启停门、ST 参考控制调用和故障清零测试。
- `foc-sim` 的速度阶跃、负载扰动和硬件故障闭环场景。
- Rustfmt 格式检查。
- 全 workspace 的 Clippy `-D warnings`。
- `thumbv7em-none-eabihf` 静态库交叉编译。
- C 平台安全空实现与 ABI 结构尺寸。

这样可以在没有开发板时快速检查与硬件无关的逻辑。

### 6.12 在 VS Code 中使用

打开新工程目录：

```powershell
code $targetProject
```

建议扩展位于 `.vscode/extensions.json`：

- Microsoft C/C++。
- CMake Tools。
- Cortex-Debug。
- rust-analyzer。

常用任务：

```text
FOC: 增量编译
FOC: 重新生成并完整编译
FOC: C + Rust 全部测试
FOC: Rust Clippy
```

如果已经改了工程名称，应同步修改任务显示名称，但这不影响构建功能。

调试前必须修改 `.vscode/launch.json`：

```json
"device": "STM32G431RB",
"executable": "${workspaceFolder}/cmake-build/rtthread.elf"
```

其中 `device` 必须是调试插件支持的真实 MCU 名称。当前生成器的 ELF 默认名是 `rtthread.elf`，BIN/HEX/MAP 才使用产品名称。

## 7. 完全从官方 BSP 重新建立

如果不想复制当前工程，可以从官方 BSP 开始。以当前开发板为例：

```powershell
Set-Location E:\File\RT-Thread\rt-thread\bsp\stm32\stm32g431-st-nucleo
```

在已经加载 RT-Thread Env 的终端中执行：

```powershell
scons --menuconfig
pkgs --update
scons -j8
```

先确认官方 BSP 可以编译，再导出独立分发工程：

```powershell
scons --dist `
    --project-name=NEW_MOTOR_PROJECT `
    --project-path=E:\File\RT-Thread\projects\NEW_MOTOR_PROJECT
```

官方源码还支持生成 VS Code 和编译数据库：

```powershell
scons --target=vsc --cdb
```

也可以生成 CMake：

```powershell
scons --target=cmake
```

但官方 `--dist` 结果只是 BSP 分发工程，不会自动具有当前项目的 FOC 分层、主机测试、安全空实现和 PowerShell 构建封装。若要得到相同结构，还需继续加入：

```text
foc/include
foc/platform/<mcu>
rust/crates/foc-algorithm
rust/crates/foc-rt-bridge
custom.cmake
tests/host
build.ps1
test.ps1
.vscode
docs
```

## 8. 各文件的职责与修改规则

| 文件 | 职责 | 能否直接复制 |
|---|---|---|
| `.config` | RT-Thread、驱动和包的实际选择 | 同芯片可复制，换芯片需重配 |
| `Kconfig` | 配置入口、SoC 和板级菜单 | 作为模板复制后修改 |
| `SConstruct` | 顶层构建、RTT_ROOT、包检查、Rust 预构建、厂商驱动入口 | 同系列可复用 |
| `SConscript` | 产品宏和子目录扫描 | 修改 MCU/产品宏 |
| `rtconfig.py` | 工具链、CPU 参数、链接和输出 | 换内核必须重做 |
| `board/board.c` | 系统时钟和外设时钟 | 必须按板重做 |
| `board/board.h` | 内存和堆边界 | 必须按完整料号重做 |
| `board/Kconfig` | 板级外设开关 | 按硬件重做 |
| `board/SConscript` | 板级源文件清单 | 可作模板 |
| `board/*hal_conf.h` | HAL 模块和时钟常量 | 按厂商和系列重做 |
| `board/*hal_msp.c` | GPIO、复用和底层时钟 | 必须按原理图重做 |
| `link.lds` | Flash/RAM 布局 | 必须按完整料号重做 |
| `applications/main.c` | 应用入口和低频管理 | 可作安全模板 |
| `foc/include/foc_rust_bridge.h` | 固定 C/Rust ABI | 两侧同步修改并升级 ABI |
| `foc/platform/<mcu>` | 具体 PWM/ADC/DMA/故障实现 | 换 MCU 时新增目录 |
| `rust/crates/foc-algorithm` | no_std 纯算法库 | 换 MCU 时保持不变 |
| `rust/crates/foc-rt-bridge` | Rust 状态机、快环和 C ABI | CPU ABI 相同时复用 |
| `custom.cmake` | Cargo 产物与 CMake/Ninja 的依赖连接 | 修改 Rust target/产物名时更新 |
| `tests/host` | C 平台安全与 ABI 测试 | 应随工程一起复用 |
| `build.ps1` | 一键生成、取包和 ARM 构建 | 修改工具路径、包名、产物名 |
| `test.ps1` | PC 测试入口 | 通常可复用 |
| `.vscode` | VS Code 构建和调试入口 | 修改设备和任务名称 |

## 9. 什么时候必须重新生成

| 修改内容 | 推荐命令 |
|---|---|
| 只修改已存在的 `.c/.h/.rs` | `.\build.ps1 -BuildOnly` |
| 新增或删除源文件 | `.\build.ps1 -Regenerate` |
| 修改 `SConscript`/`SConstruct` | `.\build.ps1 -Regenerate` |
| 修改 Kconfig 或 `.config` | `.\build.ps1 -Regenerate` |
| 修改软件包选择 | `.\build.ps1 -UpdatePackages -Regenerate` |
| 切换 MCU 系列 | 删除旧生成目录后完整重新生成 |
| 修改链接脚本或启动文件 | 至少重新链接，建议 `-Regenerate` |
| IntelliSense 头文件或宏不正确 | `-Regenerate` 后检查 `compile_commands.json` |

不要手工修改生成的 `CMakeLists.txt` 或长期直接修改 `rtconfig.h`。下次生成时这些修改会丢失。

## 10. 换成其他 MCU 时的边界

### 10.1 同一个 STM32G4 型号或相近板卡

通常只需要修改：

- 时钟。
- 控制台和 GPIO。
- 链接内存。
- `.config` 中的板级驱动。
- 功率板适配。

### 10.2 换成其他 STM32 系列

需要进一步替换：

- CMSIS/HAL 软件包。
- 启动文件。
- HAL 公共驱动选择。
- `stm32xxxx_hal_conf.h`。
- 设备宏和 CPU/FPU 参数。
- 板级时钟实现。

### 10.3 换成国产 MCU

原则上保持以下内容不变：

```text
foc/include/foc_types.h
foc/include/foc_rust_bridge.h
rust/crates/foc-algorithm/
rust/crates/foc-rt-bridge/
tests/host/
上层协议、参数和状态机
```

新增或替换：

```text
board/
foc/platform/<new_mcu>/
厂商 SDK/BSP
启动文件和链接脚本
调试与烧录配置
```

如果国产 MCU 的 ADC/PWM 触发、DMA 双缓冲、比较器或 Break 机制不同，应在新平台适配层内部解决，不要把厂商寄存器扩散到 Rust crate。

## 11. 常见问题与处理

### 11.1 `pkgs --update` 出现配置或 `KeyError: 'path'`

原因通常是 `.config` 尚未同步或 `rtconfig.h` 尚未生成。

按顺序执行：

```powershell
scons --pyconfig-silent
pkgs --update
```

当前 `build.ps1` 已固定采用这个顺序。

### 11.2 提示缺少 CMSIS/HAL 包

执行：

```powershell
.\build.ps1 -UpdatePackages -Regenerate
```

不要从其他工程随意复制半套包目录。

### 11.3 修改了源文件，但 CMake 没有编译它

生成的 CMake 源码清单还是旧的。执行：

```powershell
.\build.ps1 -Regenerate
```

### 11.4 找不到预期的产品名 ELF

RT-Thread CMake 生成器当前默认输出：

```text
rtthread.elf
```

当前工程再通过 `rtconfig.py` 的后处理生成：

```text
fluxrt.bin
fluxrt.hex
fluxrt.map
```

调试配置应指向真实存在的 `rtthread.elf`。

### 11.5 HSE 或 LSE 起振失败

检查：

- `HSE_VALUE`/`LSE_VALUE` 是否正确。
- `RCC_HSE_ON` 和 `RCC_HSE_BYPASS` 是否选错。
- Nucleo 焊桥是否保持目标配置。
- 晶体型号、负载电容、ESR 和布局是否符合要求。
- LSE drive 档位是否匹配晶体。

不要通过无限增大超时时间掩盖硬件问题。

### 11.6 链接显示 ROM/RAM 溢出

先确认链接脚本容量与完整料号一致。不能因为需要更多空间就把链接脚本写成芯片实际不存在的容量。

### 11.7 VS Code 有红线，但命令行编译成功

检查：

```text
cmake-build/compile_commands.json
.vscode/settings.json
```

然后执行完整重新生成，并让 VS Code 重新加载窗口。

### 11.8 上游 RT-Thread 被意外修改

检查：

```powershell
git -C E:\File\RT-Thread\rt-thread status --short
```

正常应无输出。产品修改应放在 `projects/<project>`，不要直接堆到官方 BSP 和公共驱动目录中。

## 12. 新空工程验收清单

### 12.1 静态检查

- [ ] 完整 MCU 料号、Flash 和 RAM 与链接脚本一致。
- [ ] HSE/LSE 类型、频率和 PLL 计算正确。
- [ ] 控制台外设、GPIO 和复用与原理图一致。
- [ ] 不再残留旧工程名、旧 MCU 宏和旧平台目录。
- [ ] 三个目标端 Rust crate 未包含厂商 HAL、CMSIS 设备头或 RT-Thread 头；主机 `foc-sim` 不进入固件。
- [ ] C/Rust ABI 使用定宽类型，头文件尺寸断言通过。
- [ ] 未配置真实硬件前，启动和输出保持安全禁用。
- [ ] `git -C ...\rt-thread status --short` 无输出。

### 12.2 构建检查

```powershell
.\build.ps1 -UpdatePackages -Regenerate
.\build.ps1 -BuildOnly
.\test.ps1
```

- [ ] ARM 完整构建通过。
- [ ] ARM 增量构建通过。
- [ ] Rust 算法/桥接测试、Clippy、交叉编译和 C 主机测试全部通过。
- [ ] MAP 或符号表中能找到 `foc_rust_*`，证明 Rust archive 已进入 ELF。
- [ ] ELF、BIN、HEX、MAP 均存在。
- [ ] `compile_commands.json` 存在。
- [ ] Flash/RAM 使用率没有超出真实容量。

### 12.3 板级检查

- [ ] 下载和复位正常。
- [ ] 实测 SYSCLK 与预期一致。
- [ ] 控制台能看到启动信息和 MSH。
- [ ] HSE/LSE 起振稳定。
- [ ] 未实现 FOC 硬件前，PWM 引脚保持安全状态。

板级检查没有完成之前，只能写“目标构建通过”，不能写“开发板验证通过”。

## 13. 当前工程的验证命令

```powershell
Set-Location E:\File\RT-Thread\projects\FluxRT

.\build.ps1 -Regenerate
.\build.ps1 -BuildOnly
.\test.ps1

git -C E:\File\RT-Thread\rt-thread status --short
```

当前已取得的证据：

```text
ARM GCC 交叉编译：PASS
Rust 算法测试：66/66 PASS
Rust control 测试：5/5 PASS
Rust bridge 测试：4/4 PASS
Rust PMSM/端口故障场景：5/5 PASS
独立仿真：FOC_SIM_PASS，534.51 rpm / 524 rpm（含 0.004 N·m 负载）
Rustfmt：PASS
Rust Clippy（-D warnings）：PASS
C 平台安全测试：1/1 PASS
Rust M4F staticlib + CMake/Ninja 固件链接：PASS
ELF 已确认包含 foc_rust_configure_st_reference、foc_rust_fast_step、foc_rust_speed_step 和 CurrentLoop::update
ROM：64356 B / 128 KiB
RAM：5408 B / 32 KiB
开发板烧录与运行：尚未验证
FOC 功率级运行：尚未实现
```

## 14. 官方参考入口

- [RT-Thread 文档中心](https://www.rt-thread.org/document/site/)
- [RT-Thread Studio](https://www.rt-thread.org/studio.html)
- 当前工作区官方 BSP：`rt-thread/bsp/stm32/stm32g431-st-nucleo`
- 当前工作区 SCons 选项：`rt-thread/tools/options.py`
- 当前工作区 CMake/VS Code 生成逻辑：`rt-thread/tools/building.py`

以后创建新工程时，应优先复制本工程的安全骨架，再逐项重新确认目标 MCU、内存、时钟、控制台和平台适配；不要把某块开发板的时钟、引脚和功率参数当作通用模板直接投入另一块硬件。
