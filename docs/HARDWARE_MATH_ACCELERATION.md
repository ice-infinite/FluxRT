# FOC 硬件数学加速与 CPU 回退

## 1. 当前结论

当前 STM32G431 构建已经接入片上 CORDIC，但算法并不依赖 CORDIC 才能运行：

```text
Rust 电流环
  → ControlMath 接口
      ├─ STM32G431：C 适配器调用 CORDIC
      └─ 无加速器 / 调用失败：CpuMath + libm
```

G431 的默认配置 `CONFIG_FOC_MATH_BACKEND_STM32G4_CORDIC=y` 会加速每个电流环
采样中的一组 `sin/cos` 和电压矢量模长。Park 与逆 Park 共用同一组 `sin/cos`，
不会重复做四次三角函数计算。

当前没有把 FMAC 接入 PI。Id/Iq PI 的状态很小，先保留在 Rust/FPU 上更容易测试和
迁移；FMAC 只有在测得快环 WCET 确实受滤波或 PI 限制后再做独立后端才有意义。

## 2. 代码边界

| 文件 | 责任 |
|---|---|
| `rust/crates/foc-control/src/math.rs` | `ControlMath` 接口和可移植 `CpuMath` |
| `rust/crates/foc-control/src/controller.rs` | 只调用接口，不认识 MCU |
| `rust/crates/foc-rt-bridge/src/lib.rs` | 选择板级数学后端；每次失败自动调用 `CpuMath` |
| `foc/include/foc_math_accel.h` | C/Rust 间的最小硬件数学契约 |
| `foc/platform/stm32g431/foc_math_accel_stm32g431.c` | CORDIC 时钟、定点换算、互斥和超时 |
| `board/Kconfig` | 是否编译 G431 CORDIC 后端 |

CORDIC 寄存器只出现在 C 平台层。`foc-control`、算法库和 PC 仿真都不包含 ST 头文件，
所以以后换国产 MCU 时不需要修改控制公式。

## 3. 两级回退

### 芯片没有数学加速器

关闭 `FOC_MATH_BACKEND_STM32G4_CORDIC` 并重新生成：

```powershell
scons --menuconfig
.\build.ps1 -Regenerate
```

Cargo 不会启用 `stm32g4-cordic` feature，固件只编译 CPU 路径。主机仿真始终使用
CPU 路径，不需要伪造 CORDIC 寄存器。

### 芯片有加速器但某次调用不可用

C 适配器返回 `0` 时，Rust bridge 会在同一次控制采样内立即使用 `CpuMath` 重算。
以下情况会触发回退：

- 平台还没有调用 `foc_math_accel_init()`；
- 输入或输出不是有限浮点数；
- CORDIC 在限定轮询次数内没有给出结果；
- 角度超出板级适配器允许的快速归一化范围。

因此“启用硬件后端”不等于“失去软件实现”。启动日志会打印实际初始化后端：
`STM32G4 CORDIC` 或 `CPU`。

## 4. 实时与并发规则

CORDIC 是共享外设。当前一次操作会保存 `PRIMASK`、短暂屏蔽中断、完成寄存器事务，
然后恢复调用前的中断状态，避免线程和 ISR 同时改写 CORDIC 配置。禁止在其他模块中
绕过此适配器直接使用同一个 CORDIC；若以后多个实时模块需要共享它，应统一仲裁，
不能在快环里等待 RTOS Mutex。

适配器使用固定轮询上限，不分配堆内存、不睡眠、不记录日志。CORDIC Q1.31 用于
`sin/cos`；模长输入先缩放到 Q1.15 安全范围，结果再恢复到 SI 浮点量纲。

## 5. 移植到其他 MCU

### 没有硬件加速

保持 Cargo feature 关闭即可。`CurrentLoop::update()` 默认构造 `CpuMath`；不需要写
空驱动，也不需要修改控制器。

### 有其他 CORDIC / DSP / 数学单元

1. 在 `foc/platform/<new_mcu>/` 实现 `foc_math_accel_sin_cos()` 与
   `foc_math_accel_magnitude()`。
2. 新增准确命名的 Kconfig 选项，不复用 STM32G4 选项。
3. 给 bridge 增加对应 Cargo feature，并保持失败返回 `0` 的语义。
4. 用 CPU 结果做角度象限、零向量、限幅边界、最大输入和随机向量对拍。
5. 在实板上用 DWT 或逻辑分析仪测 WCET；只有交叉编译通过不能证明加速正确或更快。

如果新外设只支持定点，缩放、饱和和输出顺序全部属于平台适配器责任，不应把 Q 格式
泄漏到 `foc-control`。

## 6. 当前验证范围

自动测试覆盖 CPU 数学结果、替换接口是否每拍被调用一次、开启 feature 的 Rust
主机/交叉构建、关闭 feature 的交叉构建、C 无硬件 stub 和完整 G431 固件链接。

尚未完成的证据是实板 CORDIC 数值对拍与周期测量。接上 NUCLEO-G431RB 后，应先在
不使能功率级的情况下跑一组已知角度/向量，与 CPU 结果比较，再测快环时间。当前构建
成功不能替代这一步，也不能证明电机可安全运行。
