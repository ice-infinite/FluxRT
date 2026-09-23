# FluxRT MATLAB/Simulink 仿真

这是一套“手写 MATLAB Function 控制器和电机方程 + Simulink 拓扑”的离散仿真，
用于和当前 C/Rust 固件逐项对拍。它不是 Simscape 开关器件模型，也不能代替实机验证。

## 当前基线

参数已同步到 Rust ABI `0x00070000`、配置版本4：

| 项目 | 当前值 |
|---|---:|
| PWM / 速度环 | 12 kHz / 1 kHz |
| 电流PI Kp / Ki | 7.197815 / 35989.077 |
| SMO slide / boundary / alpha | 4.0 V / 0.16 A / 0.05 |
| PLL Kp / Ki | 80 / 1000 |
| 可靠性窗口 | 1 kHz下64点，即64 ms |
| 可靠性判据 | 524 rpm、0.25 V、方差比0.01、连续2次 |
| 接管 | 可靠后25 ms角度和Iq渐变 |
| 闭环保护 | 捕获0.5 s、失锁50 ms |
| 速度参考 / Iq限速 | 500 rpm/s / 32 A/s |

默认保持与实机固件相同的安全状态：`closedLoop=false`。闭环仿真必须显式打开。

## 最短使用方式

在 MATLAB 中：

```matlab
cd('E:/File/RT-Thread/projects/FluxRT/simulink')

% 当前固件默认：观察器遥测、强制角开环
openRun = run_foc_sim('duration',5);

% 观察器可靠后自动接管并进入速度闭环
closedRun = run_foc_sim('closedLoop',true,'duration',10, ...
                        'enableDeadTime',false);

% 完整回归
result = run_foc_validation();

% 生成10秒闭环稳定性图和稳态统计
report = plot_foc_stability();

% 死区补偿：理想 / 无补偿 / 双层补偿三组对照
deadtimeReport = plot_deadtime_compensation();

% 与最新5秒实机trace对比
comparison = compare_with_hardware();
```

也可以直接打开 `foc_bringup.slx` 后点击运行。模型的 `InitFcn` 会调用
`foc_workspace_init()`，直接运行使用当前固件、开环安全默认值。

## 本次回归结果

在 MATLAB R2025b 中完成的当前基线：

| 工况 | 结果 |
|---|---|
| 5 s开环、理想逆变器 | 末态5，无故障；最终真值/观察速度577.306/584.698 rpm |
| 10 s闭环、理想逆变器 | 2.084 s进入状态6，2.109 s进入状态7；最终真值约582.088 rpm；末态7，无故障 |
| 观察器不收敛注入 | 2.664 s触发捕获超时，末态8，`faultFlags=0x4` |
| 12 kHz开环实机trace | 两轮各5 s、0错误/0超时；首轮稳态Iq/Id RMSE 0.009186/0.008869 A |
| 12 kHz闭环实机trace | 两轮均约2.12 s进入状态7并保持；0错误/0超时；ISR最坏10,083/12,500 cycles |

理想逆变器结果用于和Rust主机plant对拍；实机对拍默认启用平均死区模型。

## 文件职责

| 文件 | 职责 |
|---|---|
| `init_foc_params.m` | 唯一命名参数入口，与当前Rust默认配置对应 |
| `foc_refresh_vectors.m` | 将命名参数打包为Simulink固定尺寸输入 |
| `controller_core.m` | SMO/PLL、可靠性、启动接管、速度PI、电流PI、SVPWM、故障锁存 |
| `pmsm_plant.m` | 离散dq PMSM和机械负载，可选平均死区损失 |
| `make_foc_model.m` | 生成并保存Simulink拓扑 |
| `run_foc_sim.m` | 单次运行，不改写任何`.m`源文件 |
| `run_foc_validation.m` | 参数、开环和闭环回归 |
| `plot_foc_stability.m` | 生成理想/550 ns死区闭环稳定性图和JSON统计 |
| `plot_deadtime_compensation.m` | 生成死区无补偿/有补偿对照图和量化指标 |
| `compare_with_hardware.m` | 与最新开环实机trace对拍 |
| `run_optimize.m` | 基于当前模型重新扫描观察器参数 |

## 与 Rust 主机仿真的两种模式

- `enableDeadTime=false`：对应 `rust/crates/foc-sim` 的理想平均逆变器，适合逐拍对拍。
- `enableDeadTime=true`：加入550 ns平均死区电压损失，更适合和当前功率板趋势比较。

两种结果不能混在同一张“精确一致”表里。

## 死区补偿仿真

死区补偿分成两个可以独立开关的部分：

1. 调制器前馈：按三相电流极性，在SVPWM前补回预计损失的相电压；
2. 观察器电压重构：SMO从占空比重构端电压时，扣除预计的死区损失。

当前名义参数为550 ns、增益1.0、电流过零线性带5 mA。12.3 V、12 kHz下，
预计每相死区损失约0.16236 V。运行：

```matlab
report = plot_deadtime_compensation();

% 单独测试前馈，观察器补偿关闭
run = run_foc_sim('closedLoop',true,'duration',10, ...
    'enableDeadTime',true, ...
    'enableDeadTimeCompensation',true, ...
    'enableObserverDeadTimeCompensation',false);
```

2026-09-23名义10秒仿真对照结果：

| 指标 | 550 ns无补偿 | 双层补偿 | 改善 |
|---|---:|---:|---:|
| 4-5 s目标转速RMSE | 6.9932 rpm | 3.1622 rpm | 54.8% |
| 4-5 s观察器RMSE | 10.9335 rpm | 10.6911 rpm | 2.2% |
| 8-10 s转速标准差 | 0.0472 rpm | 0.0892 rpm | 变差89.0% |
| 8-10 s Iq跟踪RMSE | 0.012805 A | 0.001814 A | 85.8% |
| 8-10 s Id RMSE | 0.010747 A | 0.004434 A | 58.7% |

两组都进入状态7、观察器可靠率100%、故障标志为0。补偿明显改善电流误差和过渡段速度误差，但本轮稳态速度标准差变差，不能把它描述为所有指标都改善。结果文件位于：

```text
results/foc_deadtime_compensation_10s.png
results/foc_deadtime_compensation_10s.fig
results/foc_deadtime_compensation_10s.mat
results/foc_deadtime_compensation_10s.json
```

分层消融结果表明，SVPWM前馈是主要改善来源，SMO电压重构补偿负责进一步修正接管
和估算；两项不是重复补偿。7秒仿真中，前馈单独开启、观察器单独开启、两项同时开启
均进入状态7且无故障。补偿增益从0.75扫描到1.25时也全部进入状态7、无故障；因此
以后实机首次试验应从0.75或更低增益开始，而不是直接假设名义增益1.0准确。

默认值仍为关闭两项补偿，因为当前550 ns和5 mA只是仿真名义值。实机移植前必须确认
驱动芯片和定时器的有效死区、采样极性、电流零偏，并从较小补偿增益开始；不能把
仿真通过直接视为实机安全。

## 状态和故障

状态编码与固件一致：

```text
3 alignment -> 4 open-loop-ramp -> 5 open-loop-hold
                                    |
                         observer reliable
                                    v
                     6 transition -> 7 closed-loop

任何锁存故障 -> 8 fault，三相duty回到0.5
```

`faultFlags`：bit0电流/非数、bit1母线、bit2观察器捕获超时、bit3闭环失锁、
bit4算法输出非数。

## 旧扫描结果

`data/optimal_observer.json` 和 `data/optimize_grid.csv` 是旧模型历史数据。它们使用了
错误的电流PI换算、4 ms可靠性窗口和过期固件参数，不能指导实机。

重新运行：

```matlab
results = run_optimize('duration',5);
```

新结果写入：

```text
data/observer_sweep_current.csv
data/observer_sweep_current.json
```

扫描结果仍然只是模型候选；任何参数上板前都必须保持电源限流、空载、短时运行，
并重新检查电流、deadline、观察器可靠性和失锁保护。

## 模型边界

当前 plant 没有ADC量化/噪声、PWM载波纹波、器件温升、母线动态、机械齿槽和结构声学。
因此它适合验证控制流程和低频趋势，不能预测刺耳声、EMI、开关尖峰或最终稳定性裕量。
实机也没有编码器/测速仪真值，所谓“582 rpm实机结果”目前是指令值和观察器估算，
不能当作轴端真值。
