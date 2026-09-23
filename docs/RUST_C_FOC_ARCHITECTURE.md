# RT-Thread FOC 的 C / Rust 混合架构

## 1. 当前结论

本工程不是“C 工程旁边放一个 Rust 文件夹”，而是已经把控制核心切换为 Rust 静态库：

```text
STM32 启动 / RT-Thread / HAL / ISR（C）
                │ 固定 C ABI
                ▼
控制状态机与快环入口（Rust foc-rt-bridge）
                │ 普通 Rust API
                ▼
纯算法库（Rust foc-algorithm）
```

原来的 C `foc_core.c/.h` 已删除。Rust 持有控制器状态和算法状态，C 不再实现另一套重复的 FOC 核心。

当前已经装入 ST MCSDK 参考电机与连续时间控制参数，并增加 PC PMSM 闭环仿真。G431 平台已接通 TIM1 12 kHz、ADC 同步注入采样、三相使能、DP/Break、软件过流和 ISR 截止保护；`foc_start` 已实机跑通 Rust Id/Iq 电流环、强制角度 Rev-Up 和两轮 5 秒 SMO 无感接管。默认仍为 `closed_loop=0`；没有独立实机测速真值前，短时运行不能等同于无感闭环已完成全工况验证。

## 2. C 和 Rust 分别负责什么

| 责任 | C | Rust |
|---|---:|---:|
| Reset、时钟、链接脚本、启动文件 | ✓ | |
| RT-Thread 线程、设备框架、Shell、IPC | ✓ | |
| STM32 HAL/LL、ADC/PWM/DMA/编码器、GPIO | ✓ | |
| 中断入口和硬件故障的立即关断 | ✓ | |
| 把 DMA 数据整理成同一周期的反馈快照 | ✓ | |
| 控制器启停/故障逻辑状态 | | ✓ |
| 参数合法性检查 | | ✓ |
| Clarke/Park、PI、逆 Park、SVPWM | | ✓ |
| 观测器、滤波器、高级控制算法 | | ✓ |
| CORDIC / DSP 等片上数学外设适配 | ✓ | |
| 数学接口选择与 CPU 自动回退 | | ✓ |
| 算法单元测试与主机模拟 | | ✓ |
| 最终是否把占空比写入硬件 | ✓ | |

最重要的边界是：Rust 可以算出占空比，但不能自行使能栅极；C 平台层在写 PWM 前还必须检查硬件故障、控制状态和输出范围。这样未来替换国产 MCU 时，主要重写 `board/` 与 `foc/platform/<mcu>/`，Rust 算法与状态机保持不变。

## 3. 目录与依赖方向

```text
applications/main.c
    ├── RT-Thread 应用入口
    └── 只调用平台端口和 Rust C ABI

foc/include/foc_types.h
    └── C/Rust 共用的定宽 ABI 数据模型
foc/include/foc_rust_bridge.h
    └── Rust 导出给 C 的唯一公共入口
foc/include/foc_platform.h
    └── 芯片平台契约
foc/platform/stm32g431/
    ├── STM32G431 功率级实现（默认失能 + ISR 电流环 + 保护）
    └── 可选 CORDIC 数学适配器

rust/crates/foc-rt-bridge/
    ├── staticlib + rlib
    ├── C ABI、状态机、参数检查、快环入口
    └── 唯一允许少量 unsafe 的位置
rust/crates/foc-control/
    ├── ST 参考速度环/电流环/启动序列/观测器组合
    ├── ControlMath / CpuMath 可替换数学后端
    └── FeedbackPort / PwmPort / SafetyPort 硬件契约
rust/crates/foc-algorithm/
    ├── 从原 lib-rs 复制的 no_std 算法库
    └── 禁止 unsafe、堆、RTOS 和芯片 HAL
rust/crates/foc-sim/
    └── PC 逆变器 + PMSM plant + 假硬件闭环
```

依赖只能向下：C 平台不能依赖算法内部结构，算法库不能反向调用 HAL 或 RT-Thread。

当前 G431 默认用 CORDIC 加速快环 `sin/cos` 和矢量模长；无加速器或单次硬件调用
失败时由 bridge 立即回落到 `CpuMath`。配置、移植和实时约束见
`HARDWARE_MATH_ACCELERATION.md`。

## 4. C/Rust ABI 规则

边界头文件是 `foc/include/foc_rust_bridge.h`，Rust 对应实现是 `rust/crates/foc-rt-bridge/src/lib.rs`。

- 状态码和状态使用固定 `uint32_t` / `#[repr(u32)]`，避免 ARM GCC 短枚举与 Rust 枚举宽度不一致。
- 跨边界结构体使用顺序一致的字段和 `#[repr(C)]`，C 侧有 `_Static_assert` 检查尺寸。
- 不跨边界传 Rust 引用、`String`、切片、trait object、泛型或拥有析构逻辑的类型。
- 不跨边界分配内存。C 提供 2048 字节、8 字节对齐的 `foc_rust_context_t`，Rust 在其中原位构造控制器；启动时还会校验实际需求尺寸。
- C 必须先调用 `foc_rust_init()`；同一个上下文不得被线程与中断并发修改。
- 裸指针转换只存在于 bridge crate。纯算法 crate 继续 `#![forbid(unsafe_code)]`。
- 目标端 Rust panic handler 只做一件硬件相关的事：调用 C 的紧急关断钩子，随后停在自旋循环；它不会尝试恢复控制。
- ABI 版本由 `FOC_RUST_ABI_VERSION` 与 `foc_rust_abi_version()` 对照；修改字段或函数签名时必须提升版本并同时修改两侧。

## 5. 当前实时路径

当前受保护启动路径是：

```text
Shell foc_start 请求启动
  → C 检查母线/偏置/同步采样/DP/Break/故障位
  → Rust foc_rust_start_realtime() 清空 PI、观测器和启动状态
  → C 装载 50% 中性占空比并 arm 功率级
  → 12 kHz ADC ISR 调用 foc_rust_realtime_step()
  → Rust 执行 SMO 遥测、Rev-Up、Id/Iq PI、圆限幅和 SVPWM
  → C 检查 duty、过流、故障和截止时间后写 CCR
  → foc_stop 或任一错误立即 disarm
```

当前 PWM/ADC 同步中断路径：

```text
1. C ISR 读取同一采样周期的 ADC DMA、母线电压和电角度
2. C 检查 Break、COMP、驱动器故障、过流和欠压
3. 任一硬件故障：立即关断 PWM，再调用 foc_rust_latch_fault()
4. 无故障：调用 foc_rust_realtime_step()
5. 返回 OK 且三个 duty 都合法：C 写入预装载 CCR
6. 其他返回值：C 立即关断，禁止沿用上一周期占空比
```

快环中禁止日志、动态分配、睡眠、Mutex 等待和文件/设备访问。ISR 只使用固定大小栈对象或预分配上下文。

启动顺序也必须有两道门：

```text
C 完成硬件自检并保持栅极关闭
  → Rust 参数配置成功
  → ADC/PWM/DMA/保护链准备完成
  → foc_rust_start_realtime(context, 1, target_rpm)
  → 清空旧 PWM 并同步启动定时器
  → 最后才允许栅极
```

`platform_ready` 不是装饰参数。C 平台没有准备好时，即使 Rust 参数完整，Rust 也不会进入 `RUNNING`。

## 6. 当前已经接通的算法调用

bridge 已把 C 输入映射到 `foc-control` 中与仿真共用的电流环；`foc_rust_speed_step()` 使用同一 crate 的速度 PI：

```text
三相电流 + 电角度 + Id/Iq 给定
  → Clarke
  → Park
  → Id/Iq PI
  → 逆 Park
  → SVPWM
  → duty_a / duty_b / duty_c
```

此外，`foc_rust_open_loop_step()` 是板级 bring-up 的窄接口：它只在 Rust 状态为 `RUNNING` 时输出旋转电压矢量，电压不超过母线的 8%，不访问任何寄存器。它不是生产启动状态机，后续应由 `RevUpSequencer + BemfPllEstimator` 的正式组合替代。

`foc_rust_configure_st_reference()` 会装入当前 ST 参考参数。完整算法库仍保留在 `foc-algorithm` 中。以后使用 MTPA/MTPV、弱磁、滤波或高级控制时，先在 Rust 内组合算法，再给 C 增加少量稳定的“业务级入口”；不要给 63 个内部模块逐个制作 C 包装函数。闭环仿真、接口替换和当前限制见 `ST_MCSDK_REFERENCE_AND_SIMULATION.md`。

## 7. 算法库如何迁入与更新

工程内副本位于：

`rust/crates/foc-algorithm`

来源是：

`F:\BaiduSyncdisk\管理文件\项目\FOC 项目\lib-rs`

本次只复制 `src/`、`examples/` 和算法文档，没有复制原目录的 `target/`，也没有删除或修改原目录。两个 `src/lib.rs` 的 SHA-256 在迁入时一致。

以后同步时不要直接覆盖 bridge：只更新 `rust/crates/foc-algorithm`，检查 `Cargo.toml` 依赖变化，然后依次运行：

```powershell
.\test.ps1
.\build.ps1 -Regenerate
```

算法变更至少要重新确认采样周期 `ts`、符号方向、电角度、母线电压、输出限幅、最坏执行时间、Flash/RAM 和低压硬件波形。

## 8. 构建链

`SCons/Kconfig` 仍管理 RT-Thread 配置和生成；VS Code 日常构建走 CMake + Ninja：

```text
build.ps1
  → SCons 生成 rtconfig.h / CMakeLists.txt（需要时）
  → custom.cmake 调用 Cargo
  → libfoc_rt_bridge.a
  → Ninja 链接 C 对象 + Rust archive
  → rtthread.elf / bin / hex / map
```

`custom.cmake` 不会被 SCons 重新生成覆盖。Rust 使用固定工具链 1.98.1 和 `thumbv7em-none-eabihf`，与 C 侧 Cortex-M4F hard-float ABI 对齐。

日常命令：

```powershell
# Rust 格式、算法、桥接、Clippy、交叉编译和 C 平台安全测试
.\test.ps1

# 增量固件构建，Cargo 会自动增量更新静态库
.\build.ps1 -BuildOnly

# Kconfig、SConscript 或构建结构改变后
.\build.ps1 -Regenerate
```

## 9. 增加新功能时放在哪里

- 新增纯数学、控制器、观测器：放 `foc-algorithm`。
- 把多个算法组织成某个电机控制流程：优先放 `foc-rt-bridge` 的 Rust 安全代码中。
- 新增 UART/CAN 命令、参数持久化、RT-Thread 线程：放 C 应用层；只把验证后的参数快照传给 Rust。
- 新增 ADC/PWM/DMA/编码器或换 MCU：放 C 平台层。
- 新增跨边界能力：先设计小而稳定的 C 数据结构，再同时修改 `.h` 与 bridge，最后升级 ABI 版本并补测试。

不要让 C 直接修改 Rust 内部状态，也不要让 Rust 绕过 C 平台层写寄存器。这个约束是以后能换芯片、做主机模拟和定位实时故障的基础。
