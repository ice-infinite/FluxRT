# Rust FOC 与 MATLAB 联合仿真

## 1. 当前方案

本工程采用“Rust 计算、MATLAB 分析”的联合方式：

```text
MATLAB 设置转速、时长和负载参数
  → 调用 foc-sim（真实 Rust 控制器 + PMSM plant）
  → Rust 导出确定格式的 CSV 时序数据
  → MATLAB 读取、统计并绘图
  → PNG + FIG + MAT + 摘要 CSV
```

这样 Rust 仍是唯一的 FOC 实现，MATLAB 不复制一套 Clarke/Park、PI 或 SVPWM，避免
两份算法逐渐不一致。MATLAB 主要负责可视化、参数扫描、结果比较和后续系统辨识。

本机已确认 MATLAB R2025b、Simulink、Simscape、Motor Control Blockset、MATLAB
Coder、Embedded Coder、Simulink Coder 和 Fixed-Point Designer 已安装；MATLAB Coder
许可证可用，`codegen` 与 `coder` 均能解析。当前安装产品共 111 个，硬件支持包为
0 个。现有“Rust 仿真 + MATLAB 绘图”流程不依赖 Coder 或硬件支持包；如果以后要由
Simulink/Embedded Coder 直接生成并部署 STM32G431 固件，再安装 STM32 支持包。

## 2. 一键运行

```powershell
cd E:\File\RT-Thread\projects\FluxRT
.\simulation\run_matlab_sim.ps1
```

修改场景：

```powershell
.\simulation\run_matlab_sim.ps1 `
    -DurationS 4 `
    -TargetRpm 700 `
    -LoadStepTimeS 1.5 `
    -LoadTorqueNm 0.006 `
    -SampleEvery 10
```

`SampleEvery` 表示每多少个 12 kHz 电流环周期记录一次，只影响输出数据密度，不影响
控制器和 plant 的实际计算频率。

也可以在 MATLAB 命令窗口运行：

```matlab
addpath("E:\File\RT-Thread\projects\FluxRT\simulation\matlab")
summary = run_foc_matlab(TargetRpm=524, LoadTorqueNm=0.004, Visible=true)
```

## 3. 输出文件

默认输出到 `simulation/results/`，该目录已加入 `.gitignore`：

| 文件 | 内容 |
|---|---|
| `foc_sim_trace.csv` | Rust 导出的逐时序原始数据 |
| `foc_sim_overview.png` | 六联图，适合快速查看和放入报告 |
| `foc_sim_overview.fig` | MATLAB 可继续编辑的图形 |
| `foc_sim_results.mat` | table、参数和统计摘要 |
| `foc_sim_summary.csv` | 最终速度、误差、RMSE、峰值电流等 |

图中包含：目标/实际转速、dq 电流、三相电流、dq 电压与限幅状态、三相占空比、
电磁转矩与负载阶跃。

Rust CSV 接口也可单独使用：

```powershell
cargo run --manifest-path .\rust\Cargo.toml -p foc-sim --release -- `
    --csv .\simulation\results\foc_sim_trace.csv
```

## 4. 当前仿真边界

MATLAB 绘图让结果更直观，但不会自动提高模型精度。当前 plant 仍是平均值逆变器、
dq PMSM 方程和理想转子角度，不包含 MOSFET 开关纹波、死区、电流采样噪声、ADC
量化、母线动态、热模型或完整 STO-PLL 闭环。因此这些图证明的是同一 Rust 控制代码
在软件 plant 上的响应，不是实机稳定性证明。

## 5. 后续升级为 Simulink 每步联合仿真

当需要 Simscape Electrical 功率级或 Simulink Motor Control Blockset plant 时，可把
`foc-control` 包成 Windows C ABI 动态库，再由 S-Function 每个采样周期调用：

```text
Simulink/Simscape plant
  → Ia/Ib/Ic、母线、电角度
  → Rust controller DLL
  → duty_a/b/c
  → Simulink PWM/逆变器
```

升级前先保留当前 CSV 基线作为回归测试，再核对 DLL 与 Simulink 的采样时间、零阶
保持、浮点类型、初始化/复位和故障语义。若只是看曲线和做参数扫描，当前 CSV 方案更
简单、稳定，也更容易在 CI 中重复运行。

## 6. 与当前实机做同工况叠加

理想角度闭环图不能直接与当前无感启动实机比较。工程另提供
`foc-bringup-sim`、实机 CSV 采集和 MATLAB 叠加入口：

```powershell
.\simulation\run_matlab_correlation.ps1
```

输出为 `simulation/results/foc_sim_vs_hardware.png/.fig/.mat`。完整的采集命令、字段
定义、首轮结果和通过边界见 [SIMULATION_HARDWARE_CORRELATION.md](SIMULATION_HARDWARE_CORRELATION.md)。
