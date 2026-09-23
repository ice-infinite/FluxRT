# Rust workspace

本目录是 FluxRT 的 Rust 部分，当前 STM32G431 参考目标为 `thumbv7em-none-eabihf`。

- `crates/foc-algorithm`：从 `F:\BaiduSyncdisk\管理文件\项目\FOC 项目\lib-rs` 复制的纯算法库。保留 `no_std`、无堆、无 HAL/RTOS 依赖的边界；原目录没有被移动或修改。
- `crates/foc-control`：ST 参考参数、速度/电流环组合、启动序列、观测器和公开硬件 ports；`no_std`。
- `crates/foc-rt-bridge`：C ABI 静态库。它持有 FOC 控制状态并调用 `foc-control`，所有裸指针和 `unsafe` 仅允许出现在这个边界 crate。
- `crates/foc-sim`：PC `std` 测试程序，包含逆变器、PMSM plant 和实现硬件 ports 的 fake。

固件构建由项目根目录的 `build.ps1` 驱动。Cargo 产物放在 `build/rust-target`，再由 RT-Thread 生成的 CMake/Ninja 工程链接进最终 ELF。

常用检查：

```powershell
cd E:\File\RT-Thread\projects\FluxRT
.\test.ps1
.\build.ps1 -BuildOnly
```

## PC 死区模型与补偿

`foc-bringup-sim` 可以在 PC 上给平均值逆变器加入死区电压误差，并分别验证两层补偿：

- PWM 前馈：按三相电流方向修正占空比，抵消逆变器死区造成的平均电压损失；
- 观测器电压重构：用估算的实际桥臂电压，而不是命令占空比，作为 BEMF/SMO 的输入。

这些开关只存在于 PC `std` 构建，不进入 `thumbv7em-none-eabihf` 固件 ABI。目标板若要使用补偿，应在实测死区、电流极性和功率级压降后，通过独立的硬件配置接口接入，不能直接照搬仿真参数。

```powershell
cd E:\File\RT-Thread\projects\FluxRT\rust

# 有 550 ns 死区，不补偿
cargo run -p foc-sim --bin foc-bringup-sim -- `
    --closed-loop --duration 10 --dead-time --dead-time-ns 550 `
    --csv ..\simulation\results\rust_closedloop_deadtime550_uncompensated_12khz_10s.csv

# 同时启用 PWM 前馈和观测器电压补偿
cargo run -p foc-sim --bin foc-bringup-sim -- `
    --closed-loop --duration 10 --dead-time-compensation --dead-time-ns 550 `
    --csv ..\simulation\results\rust_closedloop_deadtime550_compensated_12khz_10s.csv
```

`--dead-time-compensation` 是组合开关；也可以用 `--dead-time-feedforward`、`--observer-dead-time-compensation` 单独分析。`--dead-time-gain` 调补偿比例，`--dead-time-current-band` 在零电流附近平滑极性切换，避免补偿方向来回跳变。

当前 12 kHz、12.3 V、582 rpm、空载、550 ns 基准如下：

| 工况 | 4～5 s 速度 RMSE | 8～10 s 观测器 RMSE | 8～10 s Iq RMSE | 8～10 s Id RMSE |
|---|---:|---:|---:|---:|
| 理想逆变器 | 3.189 rpm | 10.780 rpm | 0.000671 A | 0.000139 A |
| 有死区，未补偿 | 3.423 rpm | 11.281 rpm | 0.011551 A | 0.011612 A |
| 有死区，双补偿 | 3.283 rpm | 10.798 rpm | 0.006650 A | 0.006922 A |

这说明补偿直接改善了死区造成的 Iq/Id 误差，观察器误差也略有改善；速度环会掩盖一部分机械速度偏差，因此不能只看稳态转速。这里仍只是平均值模型验证；实机还需要辨识 MOSFET/驱动压降、母线纹波、ADC 零偏和零电流钳位区。

不要在算法或控制 crate 中加入 STM32 HAL、RT-Thread API、动态分配或硬件寄存器访问。新增的硬件无关算法写在 `foc-algorithm`，控制流程和 ports 写在 `foc-control`；新增 C/Rust 交互接口统一写在 `foc-rt-bridge` 和 `foc/include/foc_rust_bridge.h`。
