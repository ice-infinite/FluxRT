# 实时性与内存边界记录

记录日期：2026-09-21。以下数值由Windows x86_64 release示例 `cargo run --release --example footprint` 得到。当前所有状态对象只包含定长标量或嵌套定长结构，不包含堆指针、`Vec`、`Box`或trait object。

| 状态类型 | 对象大小（字节） |
|---|---:|
| `FocBasicState` | 72 |
| `CascadeState` | 120 |
| `AdaptiveSmoState` | 40 |
| `HigherOrderSmoState` | 44 |
| `SuperTwistingSmoState` | 36 |
| `HfInjectionState` | 20 |
| `RotatingHfState` | 24 |
| `PulseInjectionState` | 24 |
| `HfBemfState` | 80 |
| `KalmanState` | 16 |
| `LuenbergerState` | 16 |
| `EkfState` | 16 |
| `EkfFocState` | 108 |
| `UkfState` | 16 |
| `AdrcState` | 32 |
| `AdrcFastTdState` | 12 |
| `AdaptiveState` | 12 |
| `FuzzyState` | 8 |
| `MpcState` | 12 |
| `NeuralNetworkState` | 16 |
| `ReinforcementLearningState` | 136 |

## 固定容量与循环上界

- `NeuralNetworkState` 固定 3 个隐藏节点，单次推理固定执行 3 次 `tanh`。
- `ReinforcementLearningState` 固定保存 8×4 个 `f32` Q 值（128 字节），连同动作与误差字段合计 136 字节；有效状态数和动作数运行时校验，不会越界。
- 遗传算法辅助函数不持有种群，只维护当前最优标量；变异信号由调用方提供。
- MPC 不分配内存，但循环次数等于 `max(candidates, 2)`。库为保持 C 版语义没有擅自截断上限，产品配置必须限制候选数并实测 WCET。

## UKF临时数据

一维UKF更新使用三个固定数组：`sigma`、`sigma_pred`、`z_sigma`，每个为`[f32; 3]`，数组数据合计36字节。除此之外还有局部标量。编译器可能将部分值放入寄存器、内联调用或重新安排栈，因此36字节不是最终最坏栈占用。

## EKF/UKF回调

模型回调类型为`Option<extern "C" fn>`：

- 没有堆分配和虚表。
- 回调地址在参数对象中固定保存。
- 任一必需回调缺失时，更新返回`0.0`并保持估计状态不变。
- 回调本身必须可重入、固定时间、无阻塞且不得在控制ISR中分配内存。

## 尚未证明的内容

对象大小和交叉编译通过不等于实机实时性通过。投产前仍需在目标MCU、实际编译优化等级和最终链接配置下完成：

1. 通过DWT周期计数器或定时器测量每个算法的平均和最大执行周期。
2. 通过链接器map、栈涂色或RTOS栈水位记录测量任务/ISR最大栈占用。
3. 在最高PWM频率、最高中断嵌套和通信负载下保留明确周期裕量。
4. 对`sin/cos/sqrt/atan2`比较软件`libm`、硬件CORDIC和查表实现的误差与WCET。
5. EKF/UKF增加协方差非负、NaN/Inf、异常测量、模型失配和长期运行测试。

建议电流环ISR只执行采样读取、核心变换/控制和PWM写入；复杂估计器是否放入ISR，应以目标板WCET实测决定。

## 交叉构建归档大小

本次 release 构建得到：

- `thumbv7em-none-eabihf`：1,001,392 字节。
- `thumbv7em-none-eabi`：999,672 字节。

这些数字是 `.rlib` 归档大小，包含对象和元数据，不是最终固件的 Flash 占用。只有把库链接进具体固件后，最终 ELF/map/`.bin` 才能用于资源验收。
