# MATLAB源码入口

核心脚本仅按基础MATLAB函数编写，未在本次环境原生执行。完整运行与输出说明见[学习总览](../docs/00_学习总览与环境指南.md)。不要只复制某一个.m文件，需保留common依赖。

`startup_course`→`selftest_math`→`run_lab(4)`→`run_lab(11)`，最后再`run_all`。

## 公共函数

| 文件 | 功能 |
|---|---|
| [mc_params.m](common/mc_params.m) | 统一SPMSM教学参数；只改对象不改控制器的实验需另行分离参数。 |
| [mc_ipm_params.m](common/mc_ipm_params.m) | MTPA/弱磁用的第二套IPMSM参数，不能与SPMSM混合。 |
| [mc_wrap.m](common/mc_wrap.m) | 将角度包裹到[-π,π)，边界归属一致。 |
| [mc_rk4.m](common/mc_rk4.m) | 四阶Runge–Kutta对象更新；不是控制器更新。 |
| [mc_pmsm_rhs.m](common/mc_pmsm_rhs.m) | 接受静止αβ电压，随真实转子角转换并返回电气/机械导数。 |
| [mc_svpwm.m](common/mc_svpwm.m) | min/max零序注入并从占空比重构平均αβ电压。 |
| [mc_current_loop.m](common/mc_current_loop.m) | L07/L08共享RL与离散PI，含反馈样本延迟和可选反算。 |
| [mc_foc_sim.m](common/mc_foc_sim.m) | L11完整平均值有感FOC及角度/电流偏置注入。 |
| [mc_servo_sim.m](common/mc_servo_sim.m) | L15位置/速度级联与一阶力矩执行器。 |
| [mc_root.m](common/mc_root.m) | 依据源文件位置取得包根目录，不依赖当前工作目录。 |
| [mc_plot.m](common/mc_plot.m) | 每张图独立窗口，保存PNG后关闭；不自动弹窗。 |
| [mc_save.m](common/mc_save.m) | 写入列名CSV、MAT与指标，记录实际MATLAB运行版本。 |

## 各实验入口

- [L01 三相电流与旋转空间矢量](labs/lab01_rotating_field.m)
- [L02 RL电气时间常数与机械惯性](labs/lab02_rl_mechanics.m)
- [L03 六步与正弦电流的波形匹配](labs/lab03_sixstep.m)
- [L04 Clarke/Park与角度偏差](labs/lab04_transforms.m)
- [L05 PMSM耦合模型与功率平衡](labs/lab05_pmsm_model.m)
- [L06 SVPWM占空比、平均电压与开关波形](labs/lab06_svpwm.m)
- [L07 电流PI带宽与电压需求](labs/lab07_current_pi.m)
- [L08 积分饱和与反馈延迟](labs/lab08_windup_delay.m)
- [L09 采样时机、ADC量化与电流误差](labs/lab09_current_sensing.m)
- [L10 编码器跨零、量化与速度估计](labs/lab10_encoder.m)
- [L11 完整有感平均值FOC与故障注入](labs/lab11_foc_closedloop.m)
- [L12 SMO反电动势观测与低速信噪比](labs/lab12_sensorless_observer.m)
- [L13 MTPA数值参考搜索](labs/lab13_mtpa.m)
- [L14 电压电流约束与高速运行包络](labs/lab14_field_weakening.m)
- [L15 五次轨迹、位置速度级联与前馈](labs/lab15_servo_trajectory.m)
- [L16 两惯量机械频响与陷波](labs/lab16_two_inertia.m)
- [L17 关节阻抗与柔性接触](labs/lab17_joint_impedance.m)
- [L18 母线回灌与制动电阻](labs/lab18_regeneration.m)
- [L19 状态机、锁存故障与复位条件](labs/lab19_protection_state.m)
- [L20 R/L参数辨识与残差](labs/lab20_parameter_identification.m)
- [L21 多轴计算与通信预算](labs/lab21_multi_axis_timing.m)
- [L22 Q15量化与坐标变换误差](labs/lab22_fixedpoint.m)
- [L23 异步转差与步进静态偏差](labs/lab23_other_motors.m)
- [L24 开关损耗与一阶热模型](labs/lab24_loss_thermal.m)

## 可选Simulink

[build_rl_pi_model.m](simulink/build_rl_pi_model.m)需要Simulink，运行后才生成.slx。它是连续未饱和PI的入门基线，不能直接当作离散L07的相同实现。

## 本机数值比对

运行某个实验后，可执行`report=verify_reference(4)`，比较本地L04 CSV与随包Python参考。该工具不自动运行实验，先检查尺寸、NaN语义和逐列误差，再按绝对/相对容差报告。它也未在交付环境原生执行。源码：[verify_reference.m](verify_reference.m)。
