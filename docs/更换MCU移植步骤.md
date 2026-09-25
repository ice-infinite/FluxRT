# 更换 MCU 的移植步骤

## 应保持不变

- `foc/include/foc_types.h`
- `foc/include/foc_rust_bridge.h` 的稳定 ABI
- `rust/crates/foc-algorithm/`
- `rust/crates/foc-rt-bridge/`
- 协议、Rust 状态机和主机测试

## 需要替换

- `board/`：启动、时钟、链接脚本、内存和中断。
- `foc/platform/<mcu>/`：PWM、ADC、DMA、故障和角度采集。
- BSP/SDK 包和编译宏。
- 下载、调试和量产烧录配置。

如果替换后的芯片仍为 Cortex-M4F hard-float，可以继续使用
`thumbv7em-none-eabihf`；CPU/FPU ABI 不同则必须同时修改
`rust/rust-toolchain.toml`、`custom.cmake`、`foc/SConscript` 和 C 编译参数。

## 新适配器必须实现

```c
foc_status_t foc_platform_init(void);
void foc_platform_emergency_stop(void);
foc_status_t foc_platform_read_feedback(foc_feedback_t *feedback);
foc_status_t foc_platform_apply_output(const foc_output_t *output);
```

## 移植验收顺序

1. 空工程启动、时钟和控制台。
2. 确认 Flash/SRAM 链接范围和栈余量。
3. 单独验证安全关断和故障输入。
4. 仅输出低占空比互补 PWM，实测频率和死区。
5. 不接电机验证 ADC 同步触发和 DMA 数据一致性。
6. 校准零电流偏置和增益，注入已知电流核对。
7. 验证编码器/霍尔方向、零位和电角度。
8. 在限流电源、空载、机械固定条件下逐步闭环。

编译通过只能说明工具链和静态链接成立，不能证明新芯片的 ADC/PWM 同步和保护时序等价。
