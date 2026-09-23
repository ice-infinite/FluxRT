# FOC 工程架构与安全边界

## 分层原则

Rust C ABI 位于 `rust/crates/foc-rt-bridge`，板级无关控制组合位于
`rust/crates/foc-control`，纯算法位于 `rust/crates/foc-algorithm`。
这三个 crate 均不得包含：

- STM32 HAL、LL 或 CMSIS 设备头文件。
- RT-Thread 线程、设备或 IPC 头文件。
- 具体 ADC、PWM、GPIO 和 DMA 句柄。

`foc/platform/<mcu>` 负责把稳定端口转换为芯片实现：

```text
foc_platform_read_feedback()  → ADC/DMA/编码器快照
foc_platform_apply_output()   → PWM CCR/更新事件
foc_platform_emergency_stop() → 硬件安全关断
```

C 应用层只负责启停命令、状态上报、参数管理和故障协调，不实现重复的
FOC 算法；高频电流环通过固定 ABI 调用 Rust。

## 当前执行上下文

```text
PWM/ADC 同步中断（高优先级）
  ├─ 获取同一控制周期的电流、母线和角度快照
  ├─ 检查硬件故障并调用 foc_rust_realtime_step()
  ├─ 返回 OK 后更新下一周期 PWM
  └─ 立即退出

RT-Thread 管理线程（低频）
  ├─ 启停状态机
  ├─ 参数和通信
  ├─ 故障记录
  └─ 监控与日志
```

快环中禁止动态分配、阻塞、日志打印和等待 Mutex。

## 当前安全不变量

- 上电不会自动启动：PB13/PB14/PB15、TIM1 CH1～3 和 MOE 默认关闭。
- TIM1 持续运行 CH4 内部采样时基；ADC1/ADC2 以约 12 kHz 同步注入采样。
- 只有 Shell 显式执行 `foc_start`，且母线、偏置、同步采样、DP/Break 和 C/Rust ABI 全部通过后才会 arm。
- Rust 电压矢量使用可配置的母线利用率；当前板级默认 90%，C 再检查三相 duty 必须位于 3%～97%。
- ADC ISR 的软件过流阈值当前为 1.15 A；PA11/Break2 或驱动故障也直接清除 MOE、三相通道与使能 GPIO。
- SMO 默认运行用于遥测；两轮 5 秒实机试验已通过平滑接管并保持状态 7，但 `closed_loop_enable=0` 继续作为上电默认值，直到取得独立转速/角度真值并扩大验证范围。
- 闭环试验 ISR 实测最坏 10,083 cycles，软件截止为 12,500 cycles；12 kHz 的物理周期约 14,167 cycles。
- 命令结束、错误返回或 `foc_stop` 都先关断硬件，再复位 Rust 状态；故障位保持锁存并阻止再次 arm。
- Rust 算法返回错误前清零输出，C 不沿用上一周期占空比。

上述入口仍是 bring-up 接口，不是生产状态机。完成故障注入、波形、相序和观测器可靠性验证前，禁止启用无感闭环切换。

## 为什么不使用运行时函数表

芯片适配通过固定 C 符号链接；C/Rust 之间也使用固定函数 ABI，而不是在
快环里使用动态函数表。这样可以：

- 保持调用路径清楚。
- 便于编译器内联和优化。
- 避免运行时选择错误适配器。
- 更容易从 map 文件确认最终链接的硬件实现。

移植时由构建清单选择唯一的 `foc/platform/<mcu>`。

完整边界、ABI 和调用顺序见
[RT-Thread FOC 的 C / Rust 混合架构](RUST_C_FOC_ARCHITECTURE.md)。参考参数与
PC plant 闭环见 [ST MCSDK 参考参数与 PC 闭环仿真](ST_MCSDK_REFERENCE_AND_SIMULATION.md)。
后续整改的目标目录、模块所有权和工作包落点见
[FOC 整改工程架构分配与代码落位规范](REMEDIATION_ARCHITECTURE_ALLOCATION.md)。
