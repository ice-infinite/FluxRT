# Rust 迁移状态

基线日期：2026-09-21。原 C 库共有 63 个生产模块和 64 个测试入口。当前 Rust 四个阶段完成 63/63 个生产模块；“已迁移”表示已实现、已移植对应 C 测试向量且通过主机测试，并不表示已经完成实机闭环验证。

| C 模块 | Rust 位置 | 状态 | 后续验证重点 |
|---|---|---|---|
| Common | `src/math.rs` | 已迁移 | 极值、NaN 策略、目标平台数学库误差 |
| Transform_ClarkePark | `src/transform.rs` | 已迁移 | 坐标方向、相序、电角度符号 |
| Controller_P | `src/controller.rs` | 已迁移 | 限幅边界 |
| Controller_PI | `src/controller.rs` | 已迁移 | 抗饱和、采样周期、定点/浮点误差 |
| Controller_PD | `src/controller.rs` | 已迁移 | 微分滤波、首次更新无冲击 |
| Controller_PID | `src/controller.rs` | 已迁移 | 抗饱和和微分噪声 |
| Controller_SpeedLoop | `src/controller.rs` | 已迁移 | 电流限幅和多速率调度 |
| Controller_PositionLoop | `src/controller.rs` | 已迁移 | 角度跨零和机械限位 |
| Filter_LowPass | `src/filter.rs` | 已迁移 | 截止频率与采样周期换算 |
| Utility_LimitRamp | `src/utility.rs` | 已迁移 | 正反向速率、异常参数 |
| Modulation_SinePWM | `src/modulation.rs` | 已迁移 | 占空比边界和相序 |
| Modulation_SVPWM | `src/modulation.rs` | 已迁移 | 扇区、过调制、母线利用率 |
| Modulation_DPWM | `src/modulation.rs` | 已迁移 | 钳位相切换和电流采样窗口 |
| Observer_PLL | `src/observer.rs` | 已迁移 | 锁定范围、噪声、相位跃变 |
| FOC_Basic | `src/foc.rs` | 已迁移 | 电流环闭环、时序、实机保护 |
| Angle_CORDIC | `src/angle.rs` | 已迁移 | 软件/硬件 CORDIC 后端抽象 |
| Commutation_SixStep | `src/commutation.rs` | 已迁移 | 换相表、方向、过零时序 |
| Controller_Cascade | `src/controller.rs` | 已迁移 | 多环限幅和复位传播 |
| Controller_FeedForward | `src/controller.rs` | 已迁移 | 单位、符号和饱和协调 |
| Filter_Digital | `src/filter.rs` | 已迁移 | 系数稳定性和状态初值 |
| Control_MTPA | `src/optimization.rs` | 已迁移 | 电机参数敏感度和电流圆限制 |
| Control_MTPV | `src/optimization.rs` | 已迁移并修正状态残留 | 电压椭圆和高速区切换 |
| Control_VF | `src/optimization.rs` | 已迁移 | 启动斜坡和低频补偿 |
| Control_Weakening | `src/optimization.rs` | 已迁移 | 母线裕量、Id 下限、切换无扰 |
| Observer_BEMF | `src/observer/bemf.rs` | 已迁移 | 低速不可观性和滤波延迟 |
| Observer_BEMFIntegral | `src/observer/bemf.rs` | 已迁移 | 积分漂移和复位 |
| Observer_BEMFZeroCross | `src/observer/bemf.rs` | 已迁移 | 消隐、过零抖动和换相延迟 |
| Observer_BEMF_PLL | `src/observer/bemf.rs` | 已迁移 | PLL 带宽和反电势符号 |
| Observer_Flux | `src/observer/flux.rs` | 已迁移 | Rs 偏差和积分漂移 |
| Observer_FluxCurrentModel | `src/observer/flux.rs` | 已迁移 | 模型参数温漂 |
| Observer_FluxHybrid | `src/observer/flux.rs` | 已迁移 | 低/高速切换连续性 |
| Observer_FluxImprovedIntegrator | `src/observer/flux.rs` | 已迁移 | 漂移抑制和幅值误差 |
| Observer_Flux_SMO | `src/observer/smo.rs` | 已迁移 | 滑模抖振和滤波相移 |
| Observer_SMO | `src/observer/smo.rs` | 已迁移 | 滑模增益和低速性能 |
| Observer_SMO_PLL | `src/observer/smo.rs` | 已迁移 | 观测器/PLL 联合带宽 |
| Observer_AdaptiveSMO | `src/observer/advanced_smo.rs` | 已迁移 | 自适应律稳定性 |
| Observer_HigherOrderSMO | `src/observer/advanced_smo.rs` | 已迁移 | 数值刚性和执行时间 |
| Observer_SuperTwistingSMO | `src/observer/advanced_smo.rs` | 已迁移 | 离散化、噪声和抖振 |
| Observer_HFInjection | `src/observer/injection.rs` | 已迁移 | 注入波形、解调和声噪 |
| Observer_RotatingHFInjection | `src/observer/injection.rs` | 已迁移 | 旋转注入方向和凸极性 |
| Observer_PulseInjection | `src/observer/injection.rs` | 已迁移 | 脉冲宽度和峰值电流 |
| Observer_HF_BEMF | `src/observer/injection.rs` | 已迁移 | 低高速融合切换 |
| Observer_Kalman | `src/observer/estimator.rs` | 已迁移 | 协方差正定性和执行时间 |
| Observer_Luenberger | `src/observer/estimator.rs` | 已迁移 | 极点配置和参数偏差 |
| Observer_EKF | `src/observer/estimator.rs` | 已迁移 | 回调边界、协方差和WCET |
| Observer_EKF_FOC | `src/observer/estimator.rs` | 已迁移 | 状态定义和闭环耦合 |
| Observer_UKF | `src/observer/estimator.rs` | 已迁移 | Sigma点、协方差和栈占用 |
| Control_Adaptive | `src/advanced_control.rs` | 已迁移 | 参数收敛和边界投影 |
| Control_ADRC | `src/adrc.rs` | 已迁移 | ESO 带宽、饱和、离散化 |
| Control_ADRC_Fal | `src/adrc.rs` | 已迁移 | 非线性函数零点附近行为 |
| Control_ADRC_FastTD | `src/adrc.rs` | 已迁移 | 跟踪微分器数值稳定性 |
| Control_ADRC_Nonlinear | `src/adrc.rs` | 已迁移 | 非线性反馈和限幅 |
| Control_ADRC_Tuning | `src/adrc.rs` | 已迁移且完整初始化 | 参数映射和带宽有效域 |
| Control_Backstepping | `src/advanced_control.rs` | 已迁移 | 模型依赖和稳定性条件 |
| Control_Fuzzy | `src/advanced_control.rs` | 已迁移 | 隶属度边界和规则表 |
| Control_Hinf | `src/advanced_control.rs` | 已迁移 | 参数尺度和鲁棒性验证 |
| Control_LQR | `src/advanced_control.rs` | 已迁移 | 状态尺度和增益参数 |
| Control_MPC | `src/advanced_control.rs` | 已迁移 | 候选数上界、求解耗时和降级策略 |
| Control_MRAC | `src/advanced_control.rs` | 已迁移 | 自适应律和参数漂移 |
| Control_NeuralNetwork | `src/learning.rs` | 已迁移 | 权重格式、确定性和 Flash/WCET |
| Control_ReinforcementLearning | `src/learning.rs` | 已迁移，固定 8×4 | 策略安全包络和在线学习风险 |
| Control_SMC | `src/advanced_control.rs` | 已迁移 | 抖振和离散化 |
| Optimizer_GeneticAlgorithm | `src/genetic.rs` | 已迁移，外部变异信号 | 随机源质量和离线/在线边界 |

## 迁移验收门槛

表中模块按以下条件标记“已迁移”：

1. Rust API 的状态、参数、单位和复位语义写清楚。
2. 对应 C 原测试向量在 Rust 中通过。
3. 增加异常参数、饱和、边界和状态复位测试。
4. `cargo fmt --check` 与 `cargo clippy --all-targets -- -D warnings` 通过。
5. 两个 `thumbv7em` 目标的 release 检查通过。
6. 对含矩阵、回调或大数组的模块记录栈占用和最坏执行时间。
7. 涉及闭环稳定性的模块必须另做仿真/HIL/实机测试，主机单元测试不能替代。
