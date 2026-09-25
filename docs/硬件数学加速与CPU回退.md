# FOC 硬件数学加速与 CPU 回退

## 1. 当前结论

当前 STM32G431 构建采用 CORDIC 与 FPU 分工，但算法并不依赖该后端才可运行：

```text
Rust 电流环
  → ControlMath 接口
      ├─ STM32G431：sin/cos、atan2 → CORDIC
      │              magnitude → FPU VSQRT.F32
      ├─ 显式候选：sin/cos、atan2 → Rust CPU 快速多项式
      └─ 无加速器 / 调用失败：CpuMath + libm
```

G431 的默认配置 `CONFIG_FOC_MATH_BACKEND_STM32G4_CORDIC=y` 会加速每个电流环
采样中的一组 `sin/cos`，并为观察器提供 `atan2`。电压矢量模长改由 Cortex-M4F
计算平方和并执行 `VSQRT.F32`；这比 CORDIC 的 Q15 缩放、打包、临界区和反缩放更短。
Park 与逆 Park 共用同一组 `sin/cos`，不会重复做四次三角函数计算。

当前没有把 FMAC 接入 PI。Id/Iq PI 的状态很小，先保留在 Rust/FPU 上更容易测试和
迁移；FMAC 只有在测得快环 WCET 确实受滤波或 PI 限制后再做独立后端才有意义。

## 2. 代码边界

| 文件 | 责任 |
|---|---|
| `rust/crates/foc-algorithm/src/math.rs` | CPU 快速多项式纯函数与密集误差测试 |
| `rust/crates/foc-control/src/math.rs` | `ControlMath`、`CpuMath` 和 `FastApproxMath` |
| `rust/crates/foc-control/src/controller.rs` | 只调用接口，不认识 MCU |
| `rust/crates/foc-rt-bridge/src/lib.rs` | 选择板级数学后端；每次失败自动调用 `CpuMath` |
| `foc/include/foc_math_accel.h` | C/Rust 间的最小硬件数学契约 |
| `foc/platform/stm32g431/foc_math_accel_stm32g431.c` | CORDIC 三角/相位、FPU 模长、互斥和超时 |
| `board/Kconfig` | 是否编译 G431 CORDIC 后端 |

CORDIC 寄存器和 `VSQRT.F32` 内联汇编只出现在 C 平台层。`foc-control`、算法库和 PC
仿真都不包含 ST 头文件，所以以后换国产 MCU 时不需要修改控制公式。

## 3. 两级回退

### 芯片没有数学加速器

关闭 `FOC_MATH_BACKEND_STM32G4_CORDIC` 并重新生成：

```powershell
scons --menuconfig
.\build.ps1 -Regenerate
```

Cargo 不会启用 `stm32g4-cordic` feature，bridge 只选择 `CpuMath` 路径。主机仿真
始终使用 CPU 路径，不需要伪造 CORDIC 寄存器或目标 FPU 指令。

### 芯片有加速器但某次调用不可用

C 适配器返回 `0` 时，Rust bridge 会在同一次控制采样内立即使用 `CpuMath` 重算。
以下情况会触发相应运算回退：

- CORDIC 尚未成功初始化（只影响 `sin/cos`、`atan2`）；
- 输入或输出不是有限浮点数；
- CORDIC 在限定轮询次数内没有给出结果（只影响 `sin/cos`、`atan2`）；
- 角度超出板级适配器允许的快速归一化范围；
- 模长平方和溢出为非有限值。

因此“启用硬件后端”不等于“失去软件实现”。启动日志会打印实际初始化后端：
`STM32G4-CORDIC+FPU` 或 `CPU`。

## 4. 实时与并发规则

CORDIC 是共享外设。当前一次操作会保存 `PRIMASK`、短暂屏蔽中断、完成寄存器事务，
然后恢复调用前的中断状态，避免线程和 ISR 同时改写 CORDIC 配置。禁止在其他模块中
绕过此适配器直接使用同一个 CORDIC；若以后多个实时模块需要共享它，应统一仲裁，
不能在快环里等待 RTOS Mutex。

适配器使用固定轮询上限，不分配堆内存、不睡眠、不记录日志。CORDIC Q1.31 用于
`sin/cos` 和 `atan2`；模长不占用 CORDIC、不进入中断临界区，直接用浮点平方和与
`VSQRT.F32`。当前内联汇编依赖 GNU Arm 工具链，改用其它编译器时必须重新验证。

## 5. 移植到其他 MCU

### 没有硬件加速

保持 Cargo feature 关闭即可。`CurrentLoop::update()` 默认构造 `CpuMath`；不需要写
空驱动，也不需要修改控制器。

### 有其他 CORDIC / FPU / DSP / 数学单元

1. 在 `foc/platform/<new_mcu>/` 实现 `foc_math_accel_sin_cos()` 与
   `foc_math_accel_magnitude()`。
2. 新增准确命名的 Kconfig 选项，不复用 STM32G4 选项。
3. 给 bridge 增加对应 Cargo feature，并保持失败返回 `0` 的语义。
4. 用 CPU 结果做角度象限、零向量、限幅边界、最大输入和随机向量对拍。
5. 在实板上用 DWT 或逻辑分析仪测 WCET；只有交叉编译通过不能证明加速正确或更快。

如果新外设只支持定点，缩放、饱和和输出顺序全部属于平台适配器责任，不应把 Q 格式
泄漏到 `foc-control`。

## 6. 当前验证范围

自动测试覆盖 CPU 数学结果、快速近似密集误差、替换接口是否每拍被调用一次、开启 feature 的 Rust
主机/交叉构建、关闭 feature 的交叉构建、C 无硬件 stub 和完整 G431 固件链接。
目标 ELF 反汇编已确认模长路径包含 `vmul.f32`、`vfma.f32`、`vsqrt.f32`，且不再执行
CORDIC modulus 寄存器事务。

Diagnostic + Rust `s` 的三次 582 rpm、开环、trace 关闭、每次 5 秒低功率实测均为
0 invalid / 0 deadline miss / 0 error / 0 fault；最坏完整 ISR 为 7,957 cycles，相对
共享 Clarke 基线 8,267 cycles 降低 310 cycles（3.75%）。详细尺寸、固件哈希、被否决
候选和证据边界见
[`performance/2026-09-24-A3混合CORDIC与FPU.md`](performance/2026-09-24-A3混合CORDIC与FPU.md)。

固定延迟读取候选已经完成同口径停机和完整 ISR A/B，实时健康计数与单次未就绪故障注入
也已通过：两类 CORDIC 运算均恰好回退/复位一次，后续约 6.36 万拍恢复成功且没有
deadline miss。候选仍默认关闭，因为 69-cycle 收益很小而 6 NOP 假设依赖时钟、Flash
wait-state 和工具链。CPU 快速近似同口径对比也已完成：专项开环最坏完整 ISR 从
7,975 降到 7,771 cycles，但没有编码器真值，仍默认关闭。尚未完成的是实机闭环角度真值、
其它保护故障和长时间运行证据；不能外推为电机闭环已经完成生产验收。

## 7. 停机分项基准

`sin/cos` 和 `atan2` 的当前硬件基线已经取得。专用固件用 16 个间隔 `pi/8` 的单位圆
向量，每个重复 64 次，三轮结果为：

| 操作 | 原始 min cycles | 原始 avg cycles | 原始 max cycles | 最大误差 |
|---|---:|---:|---:|---:|
| 固定测量开销 | 11 | 30 | 40 | 不适用 |
| `sin/cos` | 131 | 287～288 | 412～420 | 1,013 ppb |
| `atan2` | 154 | 357～358 | 475 | 1 µrad |

三轮合计两类操作各 3,072 次，全部成功；NaN 非法输入 6/6 按契约拒绝。测量只在单个
调用期间屏蔽中断，所以这些数字用于分项 A/B，不能替代完整 ISR WCET。

基准代码增加 2,428 B，会让 Diagnostic Flash 余量从 8,556 B 降到 6,128 B，因此
`FOC_MATH_DIAGNOSTIC_BENCHMARK` 默认关闭，Production 也强制裁掉。需要复测时临时启用
Kconfig、重新生成并运行：

```text
foc_math_bench 64
```

命令会在功率级已 arm 时拒绝。测量结束应再次关闭 Kconfig 并重新生成；普通固件里该命令
应为 `command not found`。完整过程、固件哈希和默认固件实机回归见
[`performance/2026-09-24-A3-CORDIC分项基准.md`](performance/2026-09-24-A3-CORDIC分项基准.md)。

## 8. 固定延迟读取候选

候选在写完 CORDIC 参数后执行 6 个 `NOP`，然后只检查一次 `RRDY`。检查失败仍返回 0，
由 Rust `CpuMath` 重算；不会盲读 `RDATA`。同一专项镜像内，固定延迟 `sin/cos` 平均只比
轮询短约 1～2 cycles，`atan2` 短约 22～29 cycles；严格完整 ISR A/B 则从 7,957 降到
7,888 cycles，三轮均为 0 invalid/miss/error/fault。

该候选由 `FOC_MATH_CORDIC_FIXED_DELAY_CANDIDATE` 控制，依赖停机基准和专项健康诊断且
默认关闭；Production 强制保留有界轮询。即使单次未就绪注入已经证明“复位 CORDIC ->
Rust CpuMath 同拍回退 -> 后续事务恢复”，当前仍不会用 69 cycles 的收益换掉默认保护。
A/B 见 [`performance/2026-09-24-A4-CORDIC固定延迟对比.md`](performance/2026-09-24-A4-CORDIC固定延迟对比.md)，
健康计数与注入证据见
[`performance/2026-09-24-A5-CORDIC健康计数与故障恢复.md`](performance/2026-09-24-A5-CORDIC健康计数与故障恢复.md)。

## 9. CPU 快速近似候选

`FastApproxMath` 不依赖 CORDIC：`sin/cos` 使用象限缩减后的 9/8 阶多项式，`atan2`
使用第一八分区奇多项式再恢复象限；magnitude 在 G431 上仍走 FPU `VSQRT.F32`。两个
Kconfig 开关分别控制停机 benchmark 与 Diagnostic 实时选择，均默认关闭，Production
强制忽略。

板上停机基准经 C ABI 测得 CPU-fast `sin/cos` 544 cycles、`atan2` 426 cycles；实时 Rust
内联后的完整 ISR 反而比 CORDIC 最坏少 204 cycles。最大板上向量分量误差 24,855 ppb，
最大角误差 12 urad。因为当前实机只做开环、观测角没有接管控制，候选不升为 G431 默认；
它主要作为没有 CORDIC 的新 MCU 起点。完整证据见
[`performance/2026-09-24-A6-CPU快速数学候选对比.md`](performance/2026-09-24-A6-CPU快速数学候选对比.md)。

## 10. 未就绪恢复与专项健康诊断

所有构建档都保留 CORDIC 未就绪恢复：若 `RRDY` 检查失败，平台层先通过
`RCC_AHB1RSTR.CORDICRST` 复位外设、丢弃迟到结果，再恢复 `PRIMASK` 并返回 0。
这样下一笔事务不会把旧 `RDATA` 当成自己的结果。

专项 `FOC_MATH_DIAGNOSTIC_HEALTH=y` 时额外提供：

```text
foc_math_health show
foc_math_health reset
foc_math_health inject sincos
foc_math_health inject atan2
```

计数包含 calls/success/fallback/notready/recovery 和 requested/consumed/pending。该功能默认
关闭，普通 Diagnostic 与 Production 没有每拍计数写入和注入命令；它是验证工具，不是
生产遥测。修改时钟、等待状态、编译器或 MCU 后，必须重新打开专项镜像复测。
