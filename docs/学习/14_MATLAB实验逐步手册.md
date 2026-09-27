# MATLAB实验逐步手册｜24个主实验

> 所有图和下面的“参考数值”来自本包实际执行的Python参考程序；MATLAB/Simulink未在交付环境原生运行。先按[环境指南](00_学习总览与环境指南.md)在本机运行，再用这些数据比对形态、数量级和约束。

每个实验都按：学习问题→模型→运行→代码阅读→结果图→修改任务→边界组织。末尾直接附本实验MATLAB入口源码，公共函数位于`matlab/common`。完整包中已包含全部依赖，不必手动从文档逐段复制。

## 实验目录

| 实验 | 主题 | 源码 |
|---|---|---|
| [L01](#lab01) | 三相电流与旋转空间矢量 | [lab01_rotating_field.m](../matlab/labs/lab01_rotating_field.m) |
| [L02](#lab02) | RL电气时间常数与机械惯性 | [lab02_rl_mechanics.m](../matlab/labs/lab02_rl_mechanics.m) |
| [L03](#lab03) | 六步与正弦电流的波形匹配 | [lab03_sixstep.m](../matlab/labs/lab03_sixstep.m) |
| [L04](#lab04) | Clarke/Park与角度偏差 | [lab04_transforms.m](../matlab/labs/lab04_transforms.m) |
| [L05](#lab05) | PMSM耦合模型与功率平衡 | [lab05_pmsm_model.m](../matlab/labs/lab05_pmsm_model.m) |
| [L06](#lab06) | SVPWM占空比、平均电压与开关波形 | [lab06_svpwm.m](../matlab/labs/lab06_svpwm.m) |
| [L07](#lab07) | 电流PI带宽与电压需求 | [lab07_current_pi.m](../matlab/labs/lab07_current_pi.m) |
| [L08](#lab08) | 积分饱和与反馈延迟 | [lab08_windup_delay.m](../matlab/labs/lab08_windup_delay.m) |
| [L09](#lab09) | 采样时机、ADC量化与电流误差 | [lab09_current_sensing.m](../matlab/labs/lab09_current_sensing.m) |
| [L10](#lab10) | 编码器跨零、量化与速度估计 | [lab10_encoder.m](../matlab/labs/lab10_encoder.m) |
| [L11](#lab11) | 完整有感平均值FOC与故障注入 | [lab11_foc_closedloop.m](../matlab/labs/lab11_foc_closedloop.m) |
| [L12](#lab12) | SMO反电动势观测与低速信噪比 | [lab12_sensorless_observer.m](../matlab/labs/lab12_sensorless_observer.m) |
| [L13](#lab13) | MTPA数值参考搜索 | [lab13_mtpa.m](../matlab/labs/lab13_mtpa.m) |
| [L14](#lab14) | 电压电流约束与高速运行包络 | [lab14_field_weakening.m](../matlab/labs/lab14_field_weakening.m) |
| [L15](#lab15) | 五次轨迹、位置速度级联与前馈 | [lab15_servo_trajectory.m](../matlab/labs/lab15_servo_trajectory.m) |
| [L16](#lab16) | 两惯量机械频响与陷波 | [lab16_two_inertia.m](../matlab/labs/lab16_two_inertia.m) |
| [L17](#lab17) | 关节阻抗与柔性接触 | [lab17_joint_impedance.m](../matlab/labs/lab17_joint_impedance.m) |
| [L18](#lab18) | 母线回灌与制动电阻 | [lab18_regeneration.m](../matlab/labs/lab18_regeneration.m) |
| [L19](#lab19) | 状态机、锁存故障与复位条件 | [lab19_protection_state.m](../matlab/labs/lab19_protection_state.m) |
| [L20](#lab20) | R/L参数辨识与残差 | [lab20_parameter_identification.m](../matlab/labs/lab20_parameter_identification.m) |
| [L21](#lab21) | 多轴计算与通信预算 | [lab21_multi_axis_timing.m](../matlab/labs/lab21_multi_axis_timing.m) |
| [L22](#lab22) | Q15量化与坐标变换误差 | [lab22_fixedpoint.m](../matlab/labs/lab22_fixedpoint.m) |
| [L23](#lab23) | 异步转差与步进静态偏差 | [lab23_other_motors.m](../matlab/labs/lab23_other_motors.m) |
| [L24](#lab24) | 开关损耗与一阶热模型 | [lab24_loss_thermal.m](../matlab/labs/lab24_loss_thermal.m) |

---

<a id="lab01"></a>
## L01｜三相电流与旋转空间矢量

### 1. 这次要回答的问题

**时间上相差120°的三相，如何变成幅值不变的旋转矢量？**

对应分册：[01 电机原理·参数与选型](./01_电机原理_参数与选型.md)。

### 2. 对象、假设与计算步骤

给定相电流峰值 I=6 A、电频率50 Hz，生成40 ms内两电周期数据。按等幅值Clarke计算α、β，再检查三相和与矢量模长。每个采样点代表理想连续波形的取值，不包含ADC量化或PWM纹波。空间旋转轨迹是电流坐标，不是计算出的气隙磁场强度。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(1);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab01_rotating_field.m](../matlab/labs/lab01_rotating_field.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L01`读取输出。

### 4. 代码阅读抓手

fe：电频率Hz；I：相电流峰值A；th：电角度rad；alpha/beta：等幅值静止坐标；mag：矢量范数。

数据列（按顺序）：

```text
time_s, ia_A, ib_A, ic_A, alpha_A, beta_A, magnitude_A
```

### 5. 结果应怎样看

先从三相图确认各相峰值相同、时间相位不同，再在αβ图确认轨迹闭合。两张图描述同一组电流：横轴时间改变观察方式，并不增加独立自由度。默认sum_abc_max与magnitude_error在浮点舍入量级；这只是理想数学恒等式的验证。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `sum_abc_max` | 8.215650382e-15 |
| `magnitude_error` | 4.440892099e-15 |

原始数据：[CSV](../reference_results/L01_data.csv) · [指标JSON](../reference_results/L01_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![三条相电流的时间相位相差120°；幅值为相电流峰值。](../figures/L01_abc.png)

**图示解读：**三条相电流的时间相位相差120°；幅值为相电流峰值。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![αβ电流轨迹；这是安培坐标，不是磁场强度。](../figures/L01_vector.png)

**图示解读：**αβ电流轨迹；这是安培坐标，不是磁场强度。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**将fe由50改100，保持I不变，预测圆不变、时间曲线周期减半。

**任务2：**将I由6改3，预测轨迹半径减半；检查原脚本断言是否仍用变量I。

**任务3：**只将B相幅值乘0.9。原平衡断言失败是故意破坏前提的正确反馈；在副本中增加“不平衡”评价，勿直接删掉原测试。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

不含真实匝数、磁路、气隙、磁饱和；不能根据这张圆算磁吸力或实际转矩。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab01_rotating_field(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L01'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

t=linspace(0,.04,2001)'; fe=50; I=6; th=2*pi*fe*t;
a=I*cos(th);b=I*cos(th-2*pi/3);c=I*cos(th+2*pi/3);
alpha=(2/3)*(a-.5*b-.5*c);beta=(b-c)/sqrt(3);
mag=hypot(alpha,beta);
r.data=[t,a,b,c,alpha,beta,mag];
r.columns={'time_s','ia_A','ib_A','ic_A','alpha_A','beta_A','magnitude_A'};
r.metrics.sum_abc_max=max(abs(a+b+c));r.metrics.magnitude_error=max(abs(mag-I));
mc_plot(outdir,'L01_abc',t,[a,b,c],{'ia','ib','ic'},'Time (s)','Current (A)','Three-phase currents: 120-degree time shifts');
mc_plot(outdir,'L01_vector',alpha,beta,{},'Alpha (A)','Beta (A)','Current space-vector locus (not magnetic flux in tesla)');
assert(r.metrics.sum_abc_max<1e-10 && r.metrics.magnitude_error<1e-10);

mc_save(outdir, 'L01', r);
end
```

---

<a id="lab02"></a>
## L02｜RL电气时间常数与机械惯性

### 1. 这次要回答的问题

**为什么电流与转速都不是瞬间跟上输入？**

对应分册：[01 电机原理·参数与选型](./01_电机原理_参数与选型.md)。

### 2. 对象、假设与计算步骤

电气子实验取R=.2Ω、L=.4mH、2V阶跃，用精确零阶保持离散解与解析解对照。机械子实验把RL电流按Kt换成转矩，再通过J与B积分得到速度；这是单向耦合，速度没有通过反电动势反馈到电气支路，不是完整电机模型。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(2);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab02_rl_mechanics.m](../matlab/labs/lab02_rl_mechanics.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L02`读取输出。

### 4. 代码阅读抓手

R/L：锁定电气对象；a/b：精确ZOH系数；J/B：机械惯量和粘性阻尼；i_exact：解析对照。

数据列（按顺序）：

```text
time_s, current_A, analytic_A, separate_mechanics_rad_s
```

### 5. 结果应怎样看

RL曲线在约2ms达到最终10A的63.2%左右；最后数值解应逼近解析解。机械曲线则由净转矩除惯量决定初始加速度，粘性阻尼影响最终速度。不要将两图的不同时间尺度误认为求解器错误。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `RL_max_error` | 5.329070518e-14 |
| `time_constant_s` | 0.002 |

原始数据：[CSV](../reference_results/L02_data.csv) · [指标JSON](../reference_results/L02_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![独立机械支路；没有反电动势回耦电气子实验。](../figures/L02_mechanical.png)

**图示解读：**独立机械支路；没有反电动势回耦电气子实验。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![同一RL对象的解析解与数值更新对照，时间常数2ms。](../figures/L02_rl.png)

**图示解读：**同一RL对象的解析解与数值更新对照，时间常数2ms。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**L加倍、R不变：最终电流不变，时间常数加倍。

**任务2：**R加倍、L不变：最终电流减半，时间常数减半。

**任务3：**只在副本中把精确更新替换为前向欧拉并增大步长，找出数值振荡，再恢复精确更新比较。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

机械支路接收Kt乘电流的转矩，但没有反向改变电气电流。电机转起来后的完整行为请看L11，不用本例的10A预测高速稳态相电流。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab02_rl_mechanics(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L02'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_params();h=10e-6;t=(0:h:.02)';u=2;N=numel(t);i=zeros(N,1);w=i;
a=exp(-p.R*h/p.Ld);b=(1-a)/p.R;
for k=1:N-1
 i(k+1)=a*i(k)+b*u;
 % 电流由堵转 RL 教学对象给定，此机械积分是独立的转矩-惯量示例，非完整电机闭环。
 Te=p.Kt*i(k);w(k+1)=w(k)+h*(Te-p.B*w(k))/p.J;
end
exact=u/p.R*(1-exp(-p.R*t/p.Ld));
r.data=[t,i,exact,w];r.columns={'time_s','current_A','analytic_A','separate_mechanics_rad_s'};
r.metrics.RL_max_error=max(abs(i-exact));r.metrics.time_constant_s=p.Ld/p.R;
mc_plot(outdir,'L02_rl',t,[i,exact],{'Exact-ZOH recurrence','Analytical curve'},'Time (s)','Current (A)','RL step response: tau=L/R');
mc_plot(outdir,'L02_mechanical',t,w,{'Ideal torque-driven inertia'},'Time (s)','Speed (rad/s)','Separate mechanical example; back EMF not coupled');
assert(r.metrics.RL_max_error<1e-9);

mc_save(outdir, 'L02', r);
end
```

---

<a id="lab03"></a>
## L03｜六步与正弦电流的波形匹配

### 1. 这次要回答的问题

**转矩脉动为何与反电动势和电流形状共同有关？**

对应分册：[02 六步换相·异步与步进电机](./02_六步换相_异步与步进电机.md)。

### 2. 对象、假设与计算步骤

指定正弦反电动势。六步支路每一时刻给最大反电动势相+3A、最小相−3A，另一相零；正弦支路采用2I/√3峰值，使三相电流平方和相同。计算Σei作为转换功率，再在相同基准下比较纹波。此处强制电流理想跟踪，不求有限电感换相。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(3);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab03_sixstep.m](../matlab/labs/lab03_sixstep.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L03`读取输出。

### 4. 代码阅读抓手

I：六步导通电流；positive/negative相选择由瞬时e决定；sine amplitude=2I/√3；power=Σei。

数据列（按顺序）：

```text
theta_e_rad, ia_A, ib_A, ic_A, sixstep_normalized_power, sine_normalized_power
```

### 5. 结果应怎样看

换相图中的电流跳变是理想设定，不是真实线圈瞬时跳变。功率图显示正弦反电动势配正弦电流可得到近常值；六步相对峰峰纹波约14.03%。先核对同铜耗比较条件，再解读优劣。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `sixstep_relative_ripple` | 0.140301532 |
| `sine_relative_ripple` | 1.538370149e-15 |

原始数据：[CSV](../reference_results/L03_data.csv) · [指标JSON](../reference_results/L03_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![理想六步相电流，跳变由指定电流模型强制产生。](../figures/L03_commutation.png)

**图示解读：**理想六步相电流，跳变由指定电流模型强制产生。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![相同铜耗代理量下的转换功率；仅适用指定正弦反电动势。](../figures/L03_power.png)

**图示解读：**相同铜耗代理量下的转换功率；仅适用指定正弦反电动势。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**把I由3改6，检查功率幅值与电流平方和如何变化。

**任务2：**把正弦比较峰值错误地设为I，观察比较基准变了，说明为什么不能据此得出效率结论。

**任务3：**新增梯形反电动势和有限换相模型作为扩展，先定义相同平均转矩还是相同铜耗的比较目标。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

不是通用BLDC与PMSM效率排名；无PWM器件、转子动态、换相重叠和真实铁耗。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab03_sixstep(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L03'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

th=linspace(0,2*pi,3601)';I=3;e=[cos(th),cos(th-2*pi/3),cos(th+2*pi/3)];
% 选最大反电势相注入电流、最小相抽出，其余悬空：每60度换一次。
cur=zeros(size(e));
for k=1:numel(th)
 [~,hi]=max(e(k,:));[~,lo]=min(e(k,:));cur(k,hi)=I;cur(k,lo)=-I;
end
% 两种电流总RMS平方相同：sum(sixstep.^2)=2I^2，正弦峰值=2I/sqrt(3)。
isin=(2*I/sqrt(3))*e;power6=sum(e.*cur,2);powersin=sum(e.*isin,2);
r.data=[th,cur,power6,powersin];r.columns={'theta_e_rad','ia_A','ib_A','ic_A','sixstep_normalized_power','sine_normalized_power'};
r.metrics.sixstep_relative_ripple=(max(power6)-min(power6))/mean(power6);
r.metrics.sine_relative_ripple=(max(powersin)-min(powersin))/mean(powersin);
mc_plot(outdir,'L03_commutation',th*180/pi,cur,{'ia','ib','ic'},'Electrical angle (deg)','Current (A)','Ideal six-step current, no finite commutation dynamics');
mc_plot(outdir,'L03_power',th*180/pi,[power6,powersin],{'Six-step on sinusoidal EMF','Sinusoidal current'},'Electrical angle (deg)','Normalized conversion power','Same copper-loss proxy; not a universal BLDC-vs-FOC verdict');

mc_save(outdir, 'L03', r);
end
```

---

<a id="lab04"></a>
## L04｜Clarke/Park与角度偏差

### 1. 这次要回答的问题

**为什么15°角度错误会产生约1.55A假d电流？**

对应分册：[03 坐标变换·PMSM数学模型](./03_坐标变换_PMSM数学模型.md)。

### 2. 对象、假设与计算步骤

设真实id=0、iq=6A，由反Park生成αβ并构造abc，再从abc恢复dq。第二组只把观测角增加15°，不改变真实电流。借助旋转矩阵验证误差投影关系。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(4);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab04_transforms.m](../matlab/labs/lab04_transforms.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L04`读取输出。

### 4. 代码阅读抓手

th：真实电角；delta：观测角误差；a/b/c：重建三相；d/q：正确投影；db/qb：错误角投影。

数据列（按顺序）：

```text
theta_e_rad, ia_A, ib_A, ic_A, id_A, iq_A, id_15deg_A, iq_15deg_A
```

### 5. 结果应怎样看

正确dq应为0和6A；偏角组为id=6sin15°≈1.5529A、iq=6cos15°≈5.7956A。L04是真实电流固定的观测实验；L11闭环会改变真实电流，不能直接期待同样符号和数值。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `dq_error` | 1.776356839e-15 |
| `id_offset_mean` | 1.552914271 |
| `iq_offset_mean` | 5.795554958 |

原始数据：[CSV](../reference_results/L04_data.csv) · [指标JSON](../reference_results/L04_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![固定真实电流在正确/错误坐标中的投影差异。](../figures/L04_dq.png)

**图示解读：**固定真实电流在正确/错误坐标中的投影差异。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**偏角改−15°，假id变负、iq幅值相同。

**任务2：**偏角改90°，应测得id≈6A、iq≈0。

**任务3：**给估计角增加小频率差，观察固定偏角的直流混轴变成随时间旋转的dq纹波。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

没有电机动力学，正确投影不等于完成FOC控制；三相零序为零。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab04_transforms(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L04'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

th=linspace(0,4*pi,2001)';id0=0;iq0=6;
alpha=id0*cos(th)-iq0*sin(th);beta=id0*sin(th)+iq0*cos(th);
a=alpha;b=-.5*alpha+sqrt(3)/2*beta;c=-.5*alpha-sqrt(3)/2*beta;
al=(2/3)*(a-.5*b-.5*c);be=(b-c)/sqrt(3);
id=al.*cos(th)+be.*sin(th);iq=-al.*sin(th)+be.*cos(th);
delta=15*pi/180;id_bad=al.*cos(th+delta)+be.*sin(th+delta);iq_bad=-al.*sin(th+delta)+be.*cos(th+delta);
r.data=[th,a,b,c,id,iq,id_bad,iq_bad];r.columns={'theta_e_rad','ia_A','ib_A','ic_A','id_A','iq_A','id_15deg_A','iq_15deg_A'};
r.metrics.dq_error=max(abs([id-id0;iq-iq0]));r.metrics.id_offset_mean=mean(id_bad);r.metrics.iq_offset_mean=mean(iq_bad);
mc_plot(outdir,'L04_dq',th,[id,iq,id_bad,iq_bad],{'id correct','iq correct','id with +15deg','iq with +15deg'},'Electrical angle (rad)','Current (A)','A fixed encoder offset rotates the measured dq vector');
assert(r.metrics.dq_error<1e-10);

mc_save(outdir, 'L04', r);
end
```

---

<a id="lab05"></a>
## L05｜PMSM耦合模型与功率平衡

### 1. 这次要回答的问题

**dq方程的耦合符号和3/2系数怎样自检？**

对应分册：[03 坐标变换·PMSM数学模型](./03_坐标变换_PMSM数学模型.md)。

### 2. 对象、假设与计算步骤

机械电角速度固定400rad/s，外部维持转动；用课程SPMSM参数与稳态id0/iq4A对应电压驱动dq动态。RK4推进电流状态，同时用方程导数计算输入功率、铜耗、储能变化和机械转换功率。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(5);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab05_pmsm_model.m](../matlab/labs/lab05_pmsm_model.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L05`读取输出。

### 4. 代码阅读抓手

we：外部电角速度；x=[id iq]；vd/vq：施加电压；power_residual：Pe−Pcu−dWL/dt−Teωm。

数据列（按顺序）：

```text
time_s, id_A, iq_A, torque_Nm, power_balance_residual_W
```

### 5. 结果应怎样看

电流逼近预定平衡点，启动过程中输入功率还用于电感储能变化。能量图中的残差接近零，表示同一线性模型符号和系数一致；真实损耗误差没有被这个指标测量。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `power_residual_max` | 2.842170943e-14 |
| `final_iq` | 4.000026423 |

原始数据：[CSV](../reference_results/L05_data.csv) · [指标JSON](../reference_results/L05_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![恒定外部转速下dq耦合电流趋向预定工作点。](../figures/L05_dq.png)

**图示解读：**恒定外部转速下dq耦合电流趋向预定工作点。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![同一线性模型的功率平衡，不是实机损耗精度。](../figures/L05_energy.png)

**图示解读：**同一线性模型的功率平衡，不是实机损耗精度。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**将施加vd故意设0，对比id是否仍能保持0。

**任务2：**将速度提高一倍但保持旧电压，观察平衡点改变。

**任务3：**保持输入和控制周期，减半RK4步长比较轨迹；再独立检查能量恒等式，二者不是相同测试。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

速度由外部固定，没有闭合机械加速度；无铁耗、磁饱和与参数随温度变化。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab05_pmsm_model(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L05'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_params();we=400;h=10e-6;t=(0:h:.02)';N=numel(t);X=zeros(N,2);
vd=-we*p.Lq*4;vq=p.R*4+we*p.psi;
f=@(x)[(vd-p.R*x(1)+we*p.Lq*x(2))/p.Ld;(vq-p.R*x(2)-we*(p.Ld*x(1)+p.psi))/p.Lq];
for k=1:N-1,X(k+1,:)=mc_rk4(f,X(k,:)',h)';end
Te=1.5*p.p*(p.psi*X(:,2)+(p.Ld-p.Lq)*X(:,1).*X(:,2));
res=zeros(N,1);
for k=1:N
 dx=f(X(k,:)');pe=1.5*(vd*X(k,1)+vq*X(k,2));pcu=1.5*p.R*sum(X(k,:).^2);
 dW=1.5*(p.Ld*X(k,1)*dx(1)+p.Lq*X(k,2)*dx(2));
 res(k)=pe-pcu-dW-Te(k)*(we/p.p);
end
r.data=[t,X,Te,res];r.columns={'time_s','id_A','iq_A','torque_Nm','power_balance_residual_W'};
r.metrics.power_residual_max=max(abs(res));r.metrics.final_iq=X(end,2);
mc_plot(outdir,'L05_dq',t,X,{'id','iq'},'Time (s)','Current (A)','Coupled PMSM dq dynamics at imposed constant speed');
mc_plot(outdir,'L05_energy',t,res,{'pe-pcu-dW-Te*wm'},'Time (s)','Residual (W)','Instantaneous energy-balance check');
assert(r.metrics.power_residual_max<1e-8);

mc_save(outdir, 'L05', r);
end
```

---

<a id="lab06"></a>
## L06｜SVPWM占空比、平均电压与开关波形

### 1. 这次要回答的问题

**如何从dq/αβ电压真正得到可实现的三相占空比？**

对应分册：[04 PWM·SVPWM·功率驱动](./04_PWM_SVPWM_功率驱动.md)。

### 2. 对象、假设与计算步骤

48V母线，参考矢量幅值为线性上限的98%，扫描电角。用min/max公共偏置产生占空比，再从占空比重建平均电压并检验误差。另用20kHz理想载波与0.1μs观察分辨率画线电压开关波形。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(6);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab06_svpwm.m](../matlab/labs/lab06_svpwm.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L06`读取输出。

### 4. 代码阅读抓手

Vdc：母线；vab：参考空间电压；v0：公共偏置；duty：三相0..1占空比；vab_actual：从占空比重构电压。

数据列（按顺序）：

```text
theta_e_rad, duty_a, duty_b, duty_c, alpha_error_V, beta_error_V
```

### 5. 结果应怎样看

占空比最小约.01、最大约.99，平均电压误差接近浮点精度。开关线电压不是同幅值正弦，而是高频离散电平；周期平均和基波才与参考相关。可视化数学范围不证明真实99%占空比满足自举/采样。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `reconstruction_error` | 1.065814104e-14 |
| `duty_min` | 0.01 |
| `duty_max` | 0.99 |

原始数据：[CSV](../reference_results/L06_data.csv) · [指标JSON](../reference_results/L06_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![零序注入后的三相占空比；数学范围不是硬件采样范围保证。](../figures/L06_duty.png)

**图示解读：**零序注入后的三相占空比；数学范围不是硬件采样范围保证。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![理想高频线电压，放大观察与平均正弦的区别。](../figures/L06_switching.png)

**图示解读：**理想高频线电压，放大观察与平均正弦的区别。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**将幅值降到50%，看占空比靠近中点但开关电平幅值仍由母线决定。

**任务2：**在参考超过线性范围前加矢量缩放，比较与逐相裁剪的方向误差。

**任务3：**用不同扇区边界点检查平均电压连续性，不只检查扇区编号。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

理想开关无死区、寄生、二极管恢复与模拟采样；不能据此选择实际驱动器最小脉宽。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab06_svpwm(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L06'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Vdc=48;th=linspace(0,2*pi,2001)';A=.98*Vdc/sqrt(3);N=numel(th);D=zeros(N,3);re=zeros(N,2);
for k=1:N
 ref=A*[cos(th(k));sin(th(k))];[d,v]=mc_svpwm(ref,Vdc);D(k,:)=d';re(k,:)=(v-ref)';
end
% 一段独立开关示例，20 kHz 三角载波，参考角近似恒定。
h=1e-7;tt=(0:h:150e-6)';carrier=1-abs(2*mod(tt*20000,1)-1);
[d,~]=mc_svpwm(A*[cos(.4);sin(.4)],Vdc);g=double(carrier<d');vab=Vdc*(g(:,1)-g(:,2));
r.data=[th,D,re];r.columns={'theta_e_rad','duty_a','duty_b','duty_c','alpha_error_V','beta_error_V'};
r.metrics.reconstruction_error=max(abs(re(:)));r.metrics.duty_min=min(D(:));r.metrics.duty_max=max(D(:));
mc_plot(outdir,'L06_duty',th*180/pi,D,{'dA','dB','dC'},'Electrical angle (deg)','Duty ratio','Centered zero-sequence injection (linear SVPWM equivalent)');
mc_plot(outdir,'L06_switching',tt*1e6,vab,{'vAB'},'Time (us)','Line voltage (V)','Ideal switched line voltage; no dead time or parasitics');
assert(r.metrics.reconstruction_error<1e-10 && min(D(:))>=0 && max(D(:))<=1);

mc_save(outdir, 'L06', r);
end
```

---

<a id="lab07"></a>
## L07｜电流PI带宽与电压需求

### 1. 这次要回答的问题

**更快的PI为什么需要更大瞬态电压？**

对应分册：[05 PI整定·电流环·完整FOC](./05_PI整定_电流环_完整FOC.md)。

### 2. 对象、假设与计算步骤

同一RL对象、Ts50μs，5A阶跃，4V输出限制。分别以200Hz和800Hz按Kp=Lωc、Ki=Rωc初估控制器，使用相同对象精确离散更新与反算。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(7);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab07_current_pi.m](../matlab/labs/lab07_current_pi.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L07`读取输出。

### 4. 代码阅读抓手

fc：目标理想带宽；Kp/Ki：连续控制参数；z：积分状态；u：限幅后电压；ref：电流参考。

数据列（按顺序）：

```text
time_s, reference_A, i_fc200_A, i_fc800_A, v_fc200_V, v_fc800_V
```

### 5. 结果应怎样看

电流图展示响应速度，电压图展示执行器代价。饱和区不能用理想一阶小信号闭环解释。默认最终误差很小只说明该工况闭环收敛，不意味着所有参数与延迟都稳定。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `final_error_slow` | 7.105427358e-15 |
| `final_error_fast` | 8.881784197e-16 |

原始数据：[CSV](../reference_results/L07_data.csv) · [指标JSON](../reference_results/L07_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![同对象、同限压下两个PI初始带宽比较。](../figures/L07_response.png)

**图示解读：**同对象、同限压下两个PI初始带宽比较。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![快响应要求较大瞬态电压；饱和会改变小信号结论。](../figures/L07_voltage.png)

**图示解读：**快响应要求较大瞬态电压；饱和会改变小信号结论。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**把参考改0.5A观察更接近线性的响应。

**任务2：**把参考改15A观察电压受限阶段。

**任务3：**只改变R或L的对象值而不改控制器参数，观察模型匹配误差；要在副本中把对象与控制参数拆开。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

锁定RL对象，没有dq耦合、反电动势和机械运动；参考限幅与硬件保护不在本例。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab07_current_pi(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L07'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_current_loop(200,0,true,1);b=mc_current_loop(800,0,true,1);
r.data=[a(:,1:3),b(:,3),a(:,4),b(:,4)];r.columns={'time_s','reference_A','i_fc200_A','i_fc800_A','v_fc200_V','v_fc800_V'};
r.metrics.final_error_slow=abs(a(end,3)-5);r.metrics.final_error_fast=abs(b(end,3)-5);
mc_plot(outdir,'L07_response',a(:,1)*1000,[a(:,2),a(:,3),b(:,3)],{'Reference','fc=200Hz','fc=800Hz'},'Time (ms)','Current (A)','PI bandwidth change with identical RL plant and voltage limit');
mc_plot(outdir,'L07_voltage',a(:,1)*1000,[a(:,4),b(:,4)],{'fc=200Hz','fc=800Hz'},'Time (ms)','Voltage (V)','Faster reference response demands more voltage');
assert(r.metrics.final_error_slow<1e-4 && r.metrics.final_error_fast<1e-4);

mc_save(outdir, 'L07', r);
end
```

---

<a id="lab08"></a>
## L08｜积分饱和与反馈延迟

### 1. 这次要回答的问题

**不可实现的参考为什么会拖累参考下降后的恢复？**

对应分册：[05 PI整定·电流环·完整FOC](./05_PI整定_电流环_完整FOC.md)。

### 2. 对象、假设与计算步骤

第一组给30A再降5A，而4V/.2Ω只能稳态20A，比较有/无反算。第二组固定800Hz PI，比较无额外测量延迟与4采样周期的测量延迟。两组实验目的不同，不能混合解释。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(8);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab08_windup_delay.m](../matlab/labs/lab08_windup_delay.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L08`读取输出。

### 4. 代码阅读抓手

profile：可达/不可达参考；aw：是否反算；delay：反馈滞后样本数；integ：控制器内部状态而非物理电压。

数据列（按顺序）：

```text
time_s, reference_A, noAW_A, AW_A, integral_noAW_V, integral_AW_V, delay0_A, delay4_A
```

### 5. 结果应怎样看

饱和恢复图要和积分状态图一起看。默认无AW积分峰值约169.96V，有AW约21.36V，实际电压仍被限制4V。延迟图默认只有暂态更波动、最终仍收敛；最后RMS很小并不代表延迟没有代价。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `windup_peak` | 169.9612137 |
| `aw_integral_peak` | 21.36137158 |
| `delay4_rms` | 9.946149605e-14 |

原始数据：[CSV](../reference_results/L08_data.csv) · [指标JSON](../reference_results/L08_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![不可达参考及回落后恢复，有/无反算对照。](../figures/L08_aw.png)

**图示解读：**不可达参考及回落后恢复，有/无反算对照。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![默认四周期测量延迟增加暂态波动，但最终仍收敛。](../figures/L08_delay.png)

**图示解读：**默认四周期测量延迟增加暂态波动，但最终仍收敛。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![内部积分状态可能远大于实际施加电压。](../figures/L08_integral.png)

**图示解读：**内部积分状态可能远大于实际施加电压。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**改变30A不可达参考持续时间，比较积分积累。

**任务2：**反算增益减半/加倍，观察恢复与噪声敏感性。

**任务3：**逐步扫描延迟0..8和带宽，在仿真中寻找边界；保存失败也属于实验结果，不能只留稳定参数。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

反馈延迟是整数样本队列，未包含实际ADC窗口与PWM连续时序；不得宣称4个周期必然失稳。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab08_windup_delay(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L08'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_current_loop(500,0,false,2);b=mc_current_loop(500,0,true,2);
c=mc_current_loop(800,0,true,1);d=mc_current_loop(800,4,true,1);
r.data=[a(:,1:3),b(:,3),a(:,5),b(:,5),c(:,3),d(:,3)];
r.columns={'time_s','reference_A','noAW_A','AW_A','integral_noAW_V','integral_AW_V','delay0_A','delay4_A'};
r.metrics.windup_peak=max(abs(a(:,5)));r.metrics.aw_integral_peak=max(abs(b(:,5)));
r.metrics.delay4_rms=sqrt(mean((d(end-400:end,3)-5).^2));
mc_plot(outdir,'L08_aw',a(:,1)*1000,[a(:,2:3),b(:,3)],{'Reference','No anti-windup','Back-calculation'},'Time (ms)','Current (A)','Unreachable 30 A request: plant limit is 4/0.2=20 A');
mc_plot(outdir,'L08_integral',a(:,1)*1000,[a(:,5),b(:,5)],{'No AW','AW'},'Time (ms)','Integrator state (V)','Saturation recovery is a state-management problem');
mc_plot(outdir,'L08_delay',c(:,1)*1000,[c(:,2:3),d(:,3)],{'Reference','No extra delay','4-sample measurement delay'},'Time (ms)','Current (A)','The same PI gains with additional feedback delay');

mc_save(outdir, 'L08', r);
end
```

---

<a id="lab09"></a>
## L09｜采样时机、ADC量化与电流误差

### 1. 这次要回答的问题

**错误时刻采样会压过ADC位数优势吗？**

对应分册：[06 电流采样·编码器·MCU实时执行](./06_电流采样_编码器_MCU实时执行.md)。

### 2. 对象、假设与计算步骤

真实电流为8A、200Hz正弦。每PWM周期在相位.15人工加入窄干扰，比较.15和.5触发。前端Rs=.005Ω、G=20、中点1.65V、12位3.3VADC，分别在各自真实采样时刻计算误差。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(9);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab09_current_sensing.m](../matlab/labs/lab09_current_sensing.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L09`读取输出。

### 4. 代码阅读抓手

phase：PWM相位；spike：人为干扰；RsG：电流到电压灵敏度；LSB_A：理想码步长；err：时间对齐后的误差。

数据列（按顺序）：

```text
edge_sample_time_s, center_sample_time_s, truth_at_edge_A, truth_at_center_A, edge_sample_A, center_sample_A
```

### 5. 结果应怎样看

一组错误主要约4A，另一组约2.6mA。前者由人为瞬态主导，后者接近理想量化误差量级。不同触发点采到的是不同物理时间，因此比较误差时必须各自对齐真实值。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `edge_RMS_error` | 3.99996164 |
| `center_RMS_error` | 0.002598431444 |
| `LSB_A` | 0.008056640625 |

原始数据：[CSV](../reference_results/L09_data.csv) · [指标JSON](../reference_results/L09_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![误差相对于各自真实采样时刻计算，避免把时间差当幅值误差。](../figures/L09_error.png)

**图示解读：**误差相对于各自真实采样时刻计算，避免把时间差当幅值误差。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![两种触发时刻采到不同质量样本。](../figures/L09_samples.png)

**图示解读：**两种触发时刻采到不同质量样本。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**移动干扰中心到.5，观察原“好采样”变坏。

**任务2：**固定触发点改变ADC位数，识别量化地板。

**任务3：**增加零偏10mV或增益1%，分别比较直流偏差与幅值误差。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

合成毛刺不是某板实测；.5不是通用最佳点；未实现单电阻/多低侧有效状态重构。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab09_current_sensing(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L09'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Ts=50e-6;k=(0:399)';ta=(k+.15)*Ts;tb=(k+.5)*Ts;
truth=@(t)8*sin(2*pi*200*t);
% 合成开关尖峰：只用于隔离“采样位置”效应，不是某运放模型。
spike=@(t)4*exp(-((mod(t/Ts,1)-.15)/.025).^2);
rawa=truth(ta)+spike(ta);rawb=truth(tb)+spike(tb);
Rs=.005;G=20;Vref=3.3;Vbias=1.65;
quant=@(x)(min(4095,max(0,round((Vbias+Rs*G*x)/Vref*4096)))*Vref/4096-Vbias)/(Rs*G);
ia=quant(rawa);ib=quant(rawb);
r.data=[ta,tb,truth(ta),truth(tb),ia,ib];r.columns={'edge_sample_time_s','center_sample_time_s','truth_at_edge_A','truth_at_center_A','edge_sample_A','center_sample_A'};
r.metrics.edge_RMS_error=sqrt(mean((ia-truth(ta)).^2));r.metrics.center_RMS_error=sqrt(mean((ib-truth(tb)).^2));r.metrics.LSB_A=Vref/4096/(Rs*G);
mc_plot(outdir,'L09_samples',tb,[truth(tb),ia,ib],{'Truth at center','Edge sample (own time)','Center sample'},'Time (s)','Current (A)','ADC samples at contaminated vs settled parts of PWM');
mc_plot(outdir,'L09_error',tb,[ia-truth(ta),ib-truth(tb)],{'Edge error','Center error'},'Time (s)','Error (A)','Errors compared against truth at each actual sample instant');
assert(r.metrics.center_RMS_error<.01 && r.metrics.edge_RMS_error>3);

mc_save(outdir, 'L09', r);
end
```

---

<a id="lab10"></a>
## L10｜编码器跨零、量化与速度估计

### 1. 这次要回答的问题

**角度分辨率为什么影响差分速度噪声？**

对应分册：[06 电流采样·编码器·MCU实时执行](./06_电流采样_编码器_MCU实时执行.md)。

### 2. 对象、假设与计算步骤

机械角从6rad以100rpm增加，跨过2π；Ts50μs，比较12位与18位单圈量化，环绕差分后得到速度，再给12位速度加60Hz低通。另解析计算6000rpm、p7、20μs电角滞后。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(10);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab10_encoder.m](../matlab/labs/lab10_encoder.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L10`读取输出。

### 4. 代码阅读抓手

angle：单圈测量；delta：最短环绕差；bits：量化位数；fc：速度低通截止；Td：独立延迟算例。

数据列（按顺序）：

```text
time_s, true_angle_rad, angle12_rad, speed12_rad_s, speed18_rad_s, filtered_speed12_rad_s
```

### 5. 结果应怎样看

环绕角会跳回零但连续位置不应逆跳；低位速度呈量化台阶，高位噪声更低，滤波低位更平滑但有响应滞后。5.04°延迟角是另一独立量级计算，不是该100rpm波形的误差。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `rms_speed12` | 14.55251533 |
| `rms_speed18` | 0.1731998358 |
| `highspeed_delay_deg` | 5.04 |

原始数据：[CSV](../reference_results/L10_data.csv) · [指标JSON](../reference_results/L10_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![单圈环绕与连续角度；跨零不代表真实方向反转。](../figures/L10_angle.png)

**图示解读：**单圈环绕与连续角度；跨零不代表真实方向反转。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![位数量化和低通影响速度估计，滤波引入动态代价。](../figures/L10_speed.png)

**图示解读：**位数量化和低通影响速度估计，滤波引入动态代价。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**把速度降到10rpm，看计数稀疏程度。

**任务2：**加一个丢帧区段并使用真实Δt而非固定Ts。

**任务3：**增加周期角非线性，比较高分辨率仍可能存在系统性精度误差。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

无真实编码器安装误差、CRC或电气传输；最短差要求相邻有效运动小于半圈。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab10_encoder(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L10'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Ts=50e-6;t=(0:Ts:.08)';wm=100*2*pi/60;th=6+wm*t;
q12=round(th/(2*pi/4096))*(2*pi/4096);q18=round(th/(2*pi/2^18))*(2*pi/2^18);
w12=[0;mc_wrap(diff(q12))/Ts];w18=[0;mc_wrap(diff(q18))/Ts];
a=exp(-2*pi*60*Ts);wf=zeros(size(w12));
for k=2:numel(t),wf(k)=a*wf(k-1)+(1-a)*w12(k);end
r.data=[t,mod(th,2*pi),mod(q12,2*pi),w12,w18,wf];r.columns={'time_s','true_angle_rad','angle12_rad','speed12_rad_s','speed18_rad_s','filtered_speed12_rad_s'};
r.metrics.rms_speed12=sqrt(mean((w12(2:end)-wm).^2));r.metrics.rms_speed18=sqrt(mean((w18(2:end)-wm).^2));r.metrics.highspeed_delay_deg=7*(6000*2*pi/60)*20e-6*180/pi;
mc_plot(outdir,'L10_angle',t,mod(q12,2*pi),{'12-bit single-turn angle'},'Time (s)','Angle (rad)','Angle wrap must not become a speed spike');
sel=t<.02;
mc_plot(outdir,'L10_speed',t(sel),[wm*ones(sum(sel),1),w12(sel),w18(sel),wf(sel)],{'True','12-bit difference','18-bit difference','12-bit filtered'},'Time (s)','Speed (rad/s)','Resolution, differentiation noise, and filter lag');

mc_save(outdir, 'L10', r);
end
```

---

<a id="lab11"></a>
## L11｜完整有感平均值FOC与故障注入

### 1. 这次要回答的问题

**把采样、变换、PI、SVPWM和机械运动接起来会发生什么？**

对应分册：[05 PI整定·电流环·完整FOC](./05_PI整定_电流环_完整FOC.md)、[10 保护状态机·再生制动·故障诊断](./10_保护状态机_再生制动_故障诊断.md)、[13 项目案例·面试表达·实验报告](./13_项目案例_面试表达_实验报告.md)。

### 2. 对象、假设与计算步骤

SPMSM状态[id iq ωm θe]，固定48V可双向吸能母线。50μs电流环、500μs速度环，电流PI理想600Hz设计，速度环按20Hz特征频率初估。速度在.02s到100rad/s、.20s到150，负载.12s增加.15Nm。另两组同一时刻分别注入15°电角错误和A相.3A偏置。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(11);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab11_foc_closedloop.m](../matlab/labs/lab11_foc_closedloop.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L11`读取输出。

### 4. 代码阅读抓手

x：真实对象；idm：控制器测量dq；z/zs：电流/速度积分；un/usat：限幅前后电压；thv：中点预测；duty：调制输出。

数据列（按顺序）：

```text
time_s, speed_ref_rad_s, speed_rad_s, id_A, iq_A, iq_ref_A, voltage_norm_V, Te_Nm, load_Nm, dutyA, dutyB, dutyC, offset_speed_rad_s, offset_id_A, offset_iq_A, bias_speed_rad_s, bias_id_A, bias_iq_A
```

### 5. 结果应怎样看

先看速度对参考和负载的响应，再看真实dq与iq参考，最后看电压。默认正常组电压峰值18.32V，未触及26.33V限幅；电流参考8A而真实峰值8.0024A。偏角组真实id变化，说明速度恢复不代表坐标正确。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `final_speed_error` | 0.0007915601411 |
| `max_voltage` | 18.32109658 |
| `max_current` | 8.002363812 |
| `offset_final_id` | -0.3685398408 |

原始数据：[CSV](../reference_results/L11_data.csv) · [指标JSON](../reference_results/L11_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![真实dq与iq参考；参考限流不是实际电流硬上限。](../figures/L11_current.png)

**图示解读：**真实dq与iq参考；参考限流不是实际电流硬上限。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![相同闭环中两类故障的真实d电流特征。](../figures/L11_fault_effect.png)

**图示解读：**相同闭环中两类故障的真实d电流特征。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![完整平均值闭环：速度参考变化与负载扰动。](../figures/L11_speed.png)

**图示解读：**完整平均值闭环：速度参考变化与负载扰动。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![默认未触发电压圆饱和，不能据此验收饱和恢复。](../figures/L11_voltage.png)

**图示解读：**默认未触发电压圆饱和，不能据此验收饱和恢复。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**只增负载，找转矩限幅后的稳态/动态变化。

**任务2：**只降母线，设计真正触发电压圆限制的工况，再检查总输出反算。

**任务3：**把故障时刻和幅值独立改动，记录真实dq与测量dq差异；当前CSV没有保存全部内部中间量，需要在副本新增记录。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

无开关纹波、采样有效窗、真实硬件延迟、真实编码器量化及有限母线。不是可直接烧录的驱动固件，也不是实机调参结果。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab11_foc_closedloop(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L11'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_foc_sim(0);b=mc_foc_sim(1);c=mc_foc_sim(2);
r.data=[a,b(:,3:5),c(:,3:5)];
r.columns={'time_s','speed_ref_rad_s','speed_rad_s','id_A','iq_A','iq_ref_A','voltage_norm_V','Te_Nm','load_Nm','dutyA','dutyB','dutyC','offset_speed_rad_s','offset_id_A','offset_iq_A','bias_speed_rad_s','bias_id_A','bias_iq_A'};
r.metrics.final_speed_error=abs(a(end,3)-a(end,2));r.metrics.max_voltage=max(a(:,7));r.metrics.max_current=max(hypot(a(:,4),a(:,5)));r.metrics.offset_final_id=b(end,4);
mc_plot(outdir,'L11_speed',a(:,1),a(:,2:3),{'Reference','Measured'},'Time (s)','Speed (rad/s)','Full average-value FOC: start, load step, speed step');
mc_plot(outdir,'L11_current',a(:,1),[a(:,4:5),a(:,6)],{'True id','True iq','iq reference'},'Time (s)','Current (A)','Current loops inside the speed loop');
mc_plot(outdir,'L11_voltage',a(:,1),a(:,7),{'Applied dq norm'},'Time (s)','Voltage (V)','Circular voltage limit and actuator demand');
mc_plot(outdir,'L11_fault_effect',a(:,1),[a(:,4),b(:,4),c(:,4)],{'Normal true id','15deg encoder error','0.3A phase-A sensing bias'},'Time (s)','True rotor-frame id (A)','Apparently rotating is not equivalent to correctly calibrated');
assert(r.metrics.final_speed_error<2 && r.metrics.max_voltage<=.95*48/sqrt(3)+1e-8);

mc_save(outdir, 'L11', r);
end
```

---

<a id="lab12"></a>
## L12｜SMO反电动势观测与低速信噪比

### 1. 这次要回答的问题

**为什么估到反电动势不等于完成无感启动？**

对应分册：[07 无感观测器·启动与接管](./07_无感观测器_启动与接管.md)。

### 2. 对象、假设与计算步骤

外部固定ωe400rad/s，合成正弦i、di与对应真实v/e。以K=12V、边界.12A的tanh滑模校正，低通1200Hz提取e，再恢复角度。已知真实速度只用于离线滤波相位诊断；另用相同电压扰动比较.2V/8V幅值的角敏感性。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(12);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab12_sensorless_observer.m](../matlab/labs/lab12_sensorless_observer.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L12`读取输出。

### 4. 代码阅读抓手

hat：估计电流；inj：滑模电压校正；ef/Eh：低通反电动势；rawerr：原始角误差；comp：已知速度诊断修正。

数据列（按顺序）：

```text
time_s, emf_alpha_V, emf_beta_V, estimated_alpha_V, estimated_beta_V, raw_angle_error_rad, diagnostic_comp_error_rad, low_emf_error_rad, high_emf_error_rad
```

### 5. 结果应怎样看

先比较eα估计跟随，再观察原始角度滞后和诊断补偿。默认补偿RMS约1.42°。低/高信号对照约14.62°/.36°，表达信号幅度影响；它不是两台真实电机最低转速测试。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `smo_comp_rms_deg` | 1.420944607 |
| `low_emf_rms_deg` | 14.62151375 |
| `high_emf_rms_deg` | 0.3578288672 |

原始数据：[CSV](../reference_results/L12_data.csv) · [指标JSON](../reference_results/L12_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![诊断补偿借助已知速度，只用于离线误差分解。](../figures/L12_angle.png)

**图示解读：**诊断补偿借助已知速度，只用于离线误差分解。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![由合成电压/电流驱动的SMO子系统。](../figures/L12_emf.png)

**图示解读：**由合成电压/电流驱动的SMO子系统。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![同噪声、不同EMF幅值；不是完成的低速无感控制。](../figures/L12_low_speed.png)

**图示解读：**同噪声、不同EMF幅值；不是完成的低速无感控制。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**只改低通频率，观察平滑与滞后取舍。

**任务2：**只改K/边界层，观察校正强弱与数值刚性。

**任务3：**把对象R与观测器R拆开做失配，避免两边共用错误参数却误认为测试了鲁棒性。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

没有自主启动、PLL闭环、HFI、接管或估计角驱动FOC；真实速度诊断补偿不能当作生产无感算法输入。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab12_sensorless_observer(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L12'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

% 恒速、外力强制旋转的SPMSM输入。SMO不是完整无感启动闭环。
p=mc_params();h=10e-6;t=(0:h:.08)';we=400;th=we*t;I=2;
i=[-I*sin(th),I*cos(th)];di=[-I*we*cos(th),-I*we*sin(th)];
e=p.psi*we*[-sin(th),cos(th)];v=p.R*i+p.Ld*di+e;
hat=[0 0];ef=[0 0];Eh=zeros(size(i));K=12;boundary=.12;fc=1200;af=exp(-2*pi*fc*h);
for k=1:numel(t)
 inj=K*tanh((hat-i(k,:))/boundary);
 hat=hat+h*(-p.R*hat+v(k,:)-inj)/p.Ld;
 ef=af*ef+(1-af)*inj;Eh(k,:)=ef;
end
angle=atan2(-Eh(:,1),Eh(:,2));rawerr=mc_wrap(angle-th);
% 仅离线诊断使用已知恒速。真实无感补偿应改用可靠的估计速度。
comp=mc_wrap(angle+atan(we/(2*pi*fc)));err=mc_wrap(comp-th);
% 同一个测量电压扰动，比较低速和高速反电动势的可观测条件。
noise=[.08*sin(2*pi*1700*t),.06*cos(2*pi*1300*t)];
low=.2*[-sin(th),cos(th)]+noise;high=8*[-sin(th),cos(th)]+noise;
lowerr=mc_wrap(atan2(-low(:,1),low(:,2))-th);higherr=mc_wrap(atan2(-high(:,1),high(:,2))-th);
r.data=[t,e,Eh,rawerr,err,lowerr,higherr];r.columns={'time_s','emf_alpha_V','emf_beta_V','estimated_alpha_V','estimated_beta_V','raw_angle_error_rad','diagnostic_comp_error_rad','low_emf_error_rad','high_emf_error_rad'};
sel=t>.03;r.metrics.smo_comp_rms_deg=sqrt(mean(err(sel).^2))*180/pi;r.metrics.low_emf_rms_deg=sqrt(mean(lowerr.^2))*180/pi;r.metrics.high_emf_rms_deg=sqrt(mean(higherr.^2))*180/pi;
mc_plot(outdir,'L12_emf',t,[e(:,1),Eh(:,1)],{'True e-alpha','SMO + LPF'},'Time (s)','Back EMF (V)','SMO supplied with synthesized measured voltage/current');
mc_plot(outdir,'L12_angle',t,[rawerr,err]*180/pi,{'Uncompensated','Known-speed diagnostic correction'},'Time (s)','Angle error (deg)','Observer lag; correction is not an autonomous startup solution');
mc_plot(outdir,'L12_low_speed',t,[lowerr,higherr]*180/pi,{'0.2 V EMF','8 V EMF'},'Time (s)','Angle error (deg)','Identical voltage disturbance, different EMF signal levels');
assert(r.metrics.low_emf_rms_deg>r.metrics.high_emf_rms_deg);

mc_save(outdir, 'L12', r);
end
```

---

<a id="lab13"></a>
## L13｜MTPA数值参考搜索

### 1. 这次要回答的问题

**负id怎样在同转矩下减少总电流？**

对应分册：[08 MTPA·弱磁·运行包络](./08_MTPA_弱磁_运行包络.md)。

### 2. 对象、假设与计算步骤

IPMSM采用Ld.6mH、Lq1mH、ψ.012Wb、p4、12A限制。对0..0.8Nm目标沿id[-12,0]4001点扫描，按转矩式解iq，选最小电流范数并重新核对转矩。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(13);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab13_mtpa.m](../matlab/labs/lab13_mtpa.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L13`读取输出。

### 4. 代码阅读抓手

ID候选：d电流网格；IQ：转矩约束解；norm：目标函数；Imax：电流圆；torque check：独立重算。

数据列（按顺序）：

```text
target_torque_Nm, id_MTPA_A, iq_MTPA_A, I_MTPA_A, I_id0_A
```

### 5. 结果应怎样看

id逐渐变负、iq低于id0解，但真正收益要看sqrt(id²+iq²)，默认最大减少约.574A。有限网格产生近似，不是对任意电感图的解析解。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `max_current_saving_A` | 0.5743270504 |
| `recomputed_torque_error` | 1.110223025e-16 |

原始数据：[CSV](../reference_results/L13_data.csv) · [指标JSON](../reference_results/L13_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![给定转矩下的MTPA候选id/iq与id0对照。](../figures/L13_currents.png)

**图示解读：**给定转矩下的MTPA候选id/iq与id0对照。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![必须比较总电流范数，不只比较iq。](../figures/L13_saving.png)

**图示解读：**必须比较总电流范数，不只比较iq。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**让Lq=Ld，检查最优id回零。

**任务2：**网格减到201点，观察精度和参考台阶。

**任务3：**改变目标转矩上限，处理无可行候选，不用NaN或默认零静默冒充合法解。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

只求电流最优，不包括铁耗、热、退磁与电压限制；不是完整动态MTPA控制器。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab13_mtpa(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L13'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_ipm_params();T=linspace(0,.8,81)';idgrid=linspace(-p.Imax,0,4001)';D=zeros(numel(T),5);
for k=1:numel(T)
 iq=T(k)./(1.5*p.p*(p.psi+(p.Ld-p.Lq)*idgrid));I2=idgrid.^2+iq.^2;
 I2(I2>p.Imax^2)=Inf;[val,j]=min(I2);assert(isfinite(val));
 D(k,:)=[idgrid(j),iq(j),sqrt(val),T(k)/(1.5*p.p*p.psi),T(k)];
end
r.data=[T,D(:,1:4)];r.columns={'target_torque_Nm','id_MTPA_A','iq_MTPA_A','I_MTPA_A','I_id0_A'};
r.metrics.max_current_saving_A=max(D(:,4)-D(:,3));r.metrics.recomputed_torque_error=max(abs(1.5*p.p*(p.psi+(p.Ld-p.Lq)*D(:,1)).*D(:,2)-T));
mc_plot(outdir,'L13_currents',T,D(:,1:2),{'id','iq'},'Torque (Nm)','Current (A)','MTPA current references found by a bounded grid search');
mc_plot(outdir,'L13_saving',T,D(:,3:4),{'MTPA','id=0'},'Torque (Nm)','Current-vector magnitude (A)','Same requested torque, different required current');
assert(r.metrics.recomputed_torque_error<1e-10 && all(D(:,3)<=D(:,4)+1e-5));

mc_save(outdir, 'L13', r);
end
```

---

<a id="lab14"></a>
## L14｜电压电流约束与高速运行包络

### 1. 这次要回答的问题

**为什么高速要用负id换取电压裕量？**

对应分册：[08 MTPA·弱磁·运行包络](./08_MTPA_弱磁_运行包络.md)。

### 2. 对象、假设与计算步骤

同一IPMSM，在0..1200rad/s机械速度的121点扫描241×241电流网格。保留电流圆及含R的稳态电压限制内候选，选最大正转矩；另限制id=0作对照。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(14);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab14_field_weakening.m](../matlab/labs/lab14_field_weakening.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L14`读取输出。

### 4. 代码阅读抓手

ID/IQ：候选网格；mask：电流和电压共同可行性；D：每速度最大点；NaN：无可行id0解。

数据列（按顺序）：

```text
speed_rad_s, max_torque_Nm, id_A, iq_A, voltage_V, max_torque_id0_Nm
```

### 5. 结果应怎样看

低速由电流限制主导，高速电压限制收紧。默认最高速度处可行最大转矩约.23046Nm。id0曲线无可行解处断开，不代表电机在该处“必然零转矩”；仅表示当前约束下无该参考形式的合法点。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `current_max` | 11.99854158 |
| `voltage_max` | 27.7105494 |
| `highspeed_torque` | 0.23046 |

原始数据：[CSV](../reference_results/L14_data.csv) · [指标JSON](../reference_results/L14_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![稳态可行最大转矩；曲线断开代表指定限制下无可行点。](../figures/L14_envelope.png)

**图示解读：**稳态可行最大转矩；曲线断开代表指定限制下无可行点。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![高速以更负id换取电压裕量，但占用总电流。](../figures/L14_idiq.png)

**图示解读：**高速以更负id换取电压裕量，但占用总电流。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**母线从48降36V，比较高速边界。

**任务2：**将Imax降低，观察转矩与弱磁能力同时变化。

**任务3：**加密网格并比较包络收敛，记录计算成本与数值阶梯；不要把网格小波纹当真实机械振动。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

稳态包络而非动态弱磁/ MTPV控制；无退磁、铁耗、机械超速、热约束和真实电感饱和地图。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab14_field_weakening(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L14'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_ipm_params();[ID,IQ]=ndgrid(linspace(-p.Imax,0,241),linspace(0,p.Imax,241));
T=1.5*p.p*(p.psi+(p.Ld-p.Lq)*ID).*IQ;cur=ID.^2+IQ.^2<=p.Imax^2+1e-9;
wm=linspace(0,1200,121)';D=nan(numel(wm),5);vlim=p.Vdc/sqrt(3);
for k=1:numel(wm)
 we=p.p*wm(k);VD=p.R*ID-we*p.Lq*IQ;VQ=p.R*IQ+we*(p.Ld*ID+p.psi);
 mask=cur & hypot(VD,VQ)<=vlim;Tc=T;Tc(~mask)=-Inf;
 [best,idx]=max(Tc(:));
 if isfinite(best),D(k,1:4)=[best,ID(idx),IQ(idx),hypot(VD(idx),VQ(idx))];end
 mask0=mask & abs(ID)<1e-12;vals=T(mask0);if ~isempty(vals),D(k,5)=max(vals);end
end
r.data=[wm,D];r.columns={'speed_rad_s','max_torque_Nm','id_A','iq_A','voltage_V','max_torque_id0_Nm'};
r.metrics.current_max=max(hypot(D(:,2),D(:,3)));r.metrics.voltage_max=max(D(:,4));r.metrics.highspeed_torque=D(end,1);
mc_plot(outdir,'L14_envelope',wm*60/(2*pi),D(:,[1 5]),{'Optimized under I/V limits','id=0 only'},'Mechanical speed (rpm)','Feasible maximum torque (Nm)','Steady-state grid envelope, not a dynamic weakening controller');
mc_plot(outdir,'L14_idiq',wm*60/(2*pi),D(:,2:3),{'id','iq'},'Mechanical speed (rpm)','Current (A)','Negative id trades current capacity for voltage headroom');
assert(r.metrics.current_max<=p.Imax+1e-8 && r.metrics.voltage_max<=vlim+1e-8);

mc_save(outdir, 'L14', r);
end
```

---

<a id="lab15"></a>
## L15｜五次轨迹、位置速度级联与前馈

### 1. 这次要回答的问题

**为什么好的轨迹与前馈能减少跟随误差？**

对应分册：[09 伺服三环·机械共振·机器人关节](./09_伺服三环_机械共振_机器人关节.md)。

### 2. 对象、假设与计算步骤

位置0到1rad，.05s开始、.3s完成五次轨迹，后续.6s加.2Nm负载。J=.01、B=.03，内层转矩执行器一阶时间常数1ms、最大转矩1Nm。比较无前馈和速度/加速度前馈。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(15);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab15_servo_trajectory.m](../matlab/labs/lab15_servo_trajectory.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L15`读取输出。

### 4. 代码阅读抓手

s：归一时间；position/speed/acceleration reference：解析轨迹；FF：速度与Jα+Bω前馈；actuator：1ms执行动态。

数据列（按顺序）：

```text
time_s, position_ref_rad, position_noFF_rad, speed_noFF_rad_s, torque_cmd_noFF_Nm, torque_noFF_Nm, load_Nm, position_FF_rad, speed_FF_rad_s, torque_cmd_FF_Nm, torque_FF_Nm
```

### 5. 结果应怎样看

位置图显示明显的基线滞后与理想匹配前馈改善；误差图同时显示未知负载扰动。默认轨迹段RMS约.337rad与.000469rad，是该简化对象与选定增益结果，不是普遍提升比例。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `tracking_RMS_noFF` | 0.3369097996 |
| `tracking_RMS_FF` | 0.0004687361579 |

原始数据：[CSV](../reference_results/L15_data.csv) · [指标JSON](../reference_results/L15_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![默认理想匹配收益不可直接外推到参数不确定实机。](../figures/L15_error.png)

**图示解读：**默认理想匹配收益不可直接外推到参数不确定实机。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![级联伺服与理想模型前馈的轨迹跟随比较。](../figures/L15_position.png)

**图示解读：**级联伺服与理想模型前馈的轨迹跟随比较。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**轨迹时间从.3改.6，预测速度和加速度要求降低。

**任务2：**保持前馈参数不变而将真实J加倍，检验模型误差。

**任务3：**降低转矩上限，观察平滑轨迹也可能不可实现；当前脚本参数需在mc_servo_sim副本中修改。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

内层为一阶转矩对象而非完整FOC；无背隙、静摩擦、真实编码器和多关节重力。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab15_servo_trajectory(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L15'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_servo_sim(false);b=mc_servo_sim(true);r.data=[a,b(:,3:6)];
r.columns={'time_s','position_ref_rad','position_noFF_rad','speed_noFF_rad_s','torque_cmd_noFF_Nm','torque_noFF_Nm','load_Nm','position_FF_rad','speed_FF_rad_s','torque_cmd_FF_Nm','torque_FF_Nm'};
sel=a(:,1)>.05 & a(:,1)<.35;
r.metrics.tracking_RMS_noFF=sqrt(mean((a(sel,3)-a(sel,2)).^2));r.metrics.tracking_RMS_FF=sqrt(mean((b(sel,3)-b(sel,2)).^2));
mc_plot(outdir,'L15_position',a(:,1),[a(:,2:3),b(:,3)],{'Quintic reference','No feedforward','Velocity/acceleration FF'},'Time (s)','Position (rad)','Position-speed cascade with a 1 ms torque actuator');
mc_plot(outdir,'L15_error',a(:,1),[a(:,3)-a(:,2),b(:,3)-b(:,2)],{'No feedforward','With feedforward'},'Time (s)','Tracking error (rad)','Tracking lag and rejection of unknown load torque');

mc_save(outdir, 'L15', r);
end
```

---

<a id="lab16"></a>
## L16｜两惯量机械频响与陷波

### 1. 这次要回答的问题

**为什么机械弹性会限制伺服带宽？**

对应分册：[09 伺服三环·机械共振·机器人关节](./09_伺服三环_机械共振_机器人关节.md)。

### 2. 对象、假设与计算步骤

Jm=.002、Jl=.01、K=40、C=.01，含小粘性阻尼。逐频率解两惯量复数矩阵，输出电机速度/输入转矩频响，再乘一个阻尼比.04/.4的输入陷波。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(16);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab16_two_inertia.m](../matlab/labs/lab16_two_inertia.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L16`读取输出。

### 4. 代码阅读抓手

A：机械动态复数矩阵；G：电机速度/转矩传递；H：输入陷波；fr：无阻尼柔性模态估计。

数据列（按顺序）：

```text
frequency_Hz, plant_dB, input_notched_dB, plant_phase_deg
```

### 5. 结果应怎样看

从低频惯性区到反共振/共振，可以看见不能由单惯量表达的模态。横轴为log10(f/Hz)，不是普通Hz线性轴。输入陷波压峰仅展示滤波，不是闭环稳定裕量。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `resonance_undamped_Hz` | 24.65617776 |
| `antiresonance_undamped_Hz` | 10.06584242 |

原始数据：[CSV](../reference_results/L16_data.csv) · [指标JSON](../reference_results/L16_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![两惯量输入频响乘陷波；不是闭环稳定性证明。](../figures/L16_resonance.png)

**图示解读：**两惯量输入频响乘陷波；不是闭环稳定性证明。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**K变四倍，理想柔性频率约翻倍。

**任务2：**Jl变化而陷波中心固定，观察失配。

**任务3：**输出改负载速度，比较测量端对频响的影响；需要修改矩阵输出而不是只改图例。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

线性频域机械对象，无反馈控制器、饱和与采样延迟；陷波后的曲线不证明可安全提高闭环增益。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab16_two_inertia(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L16'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Jm=.002;Jl=.01;K=40;C=.01;Bm=.002;Bl=.003;f=logspace(-1,3,1200)';
fr=sqrt(K*(1/Jm+1/Jl))/(2*pi);wn=2*pi*fr;G=zeros(size(f));H=G;
for k=1:numel(f)
 s=1i*2*pi*f(k);A=[Jm*s^2+(Bm+C)*s+K,-C*s-K;-C*s-K,Jl*s^2+(Bl+C)*s+K];
 x=A\[1;0];G(k)=s*x(1);H(k)=(s^2+2*.04*wn*s+wn^2)/(s^2+2*.4*wn*s+wn^2);
end
r.data=[f,20*log10(abs(G)),20*log10(abs(G.*H)),unwrap(angle(G))*180/pi];r.columns={'frequency_Hz','plant_dB','input_notched_dB','plant_phase_deg'};
r.metrics.resonance_undamped_Hz=fr;r.metrics.antiresonance_undamped_Hz=sqrt(K/Jl)/(2*pi);
mc_plot(outdir,'L16_resonance',log10(f),r.data(:,2:3),{'Plant','Plant times input notch'},'log10 frequency (Hz)','Magnitude (dB re 1 (rad/s)/Nm)','Two-inertia FRF; filtered input is not a stability proof');

mc_save(outdir, 'L16', r);
end
```

---

<a id="lab17"></a>
## L17｜关节阻抗与柔性接触

### 1. 这次要回答的问题

**把位置刚度提高，接触时会发生什么？**

对应分册：[09 伺服三环·机械共振·机器人关节](./09_伺服三环_机械共振_机器人关节.md)。

### 2. 对象、假设与计算步骤

单关节J=.02，理想力矩执行器限幅8Nm，目标1rad、墙.6rad，墙弹性150Nm/rad、阻尼.4。比较虚拟刚度5与30Nm/rad，关节阻尼均1。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(17);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab17_joint_impedance.m](../matlab/labs/lab17_joint_impedance.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L17`读取输出。

### 4. 代码阅读抓手

K/D：虚拟刚度阻尼；wall：接触位置；contact torque：墙反作用；tau_limit：执行器饱和。

数据列（按顺序）：

```text
time_s, soft_position_rad, soft_contact_Nm, stiff_position_rad, stiff_contact_Nm
```

### 5. 结果应怎样看

先观察位置何时碰墙，再比较接触转矩。更高刚度可能产生更大压入和载荷，而不是无条件更好的控制。位置受墙限制时，持续误差会转化为力矩。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `peak_contact_soft` | 3.617746749 |
| `peak_contact_stiff` | 22.16300456 |

原始数据：[CSV](../reference_results/L17_data.csv) · [指标JSON](../reference_results/L17_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![虚拟刚度改变接触转矩，不提供真实机器人安全结论。](../figures/L17_contact.png)

**图示解读：**虚拟刚度改变接触转矩，不提供真实机器人安全结论。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![墙后目标下的关节位置与接触关系。](../figures/L17_position.png)

**图示解读：**墙后目标下的关节位置与接触关系。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**固定K只增D，观察振荡和压入量各受什么影响。

**任务2：**把目标移到墙前，确认无接触工况。

**任务3：**新增1/5ms命令延迟和速度噪声测试，明确属于扩展、不在默认脚本中。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

单自由度理想墙与力矩执行器，无真实机器人接触安全验证；不得直接用于实机碰撞试验。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab17_joint_impedance(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L17'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=1e-4;t=(0:h:.8)';N=numel(t);Kset=[5,30];Y=zeros(N,4);wall=.6;
for j=1:2
 theta=0;w=0;
 for k=1:N
  cmd=min(8,max(-8,Kset(j)*(1-theta)-1*w));
  if theta>wall,contact=max(0,150*(theta-wall)+.4*w);else,contact=0;end
  Y(k,[2*j-1,2*j])=[theta,contact];
  w=w+h*(cmd-contact-.02*w)/.02;theta=theta+h*w;
 end
end
r.data=[t,Y];r.columns={'time_s','soft_position_rad','soft_contact_Nm','stiff_position_rad','stiff_contact_Nm'};
r.metrics.peak_contact_soft=max(Y(:,2));r.metrics.peak_contact_stiff=max(Y(:,4));
mc_plot(outdir,'L17_position',t,[Y(:,1),Y(:,3),wall*ones(N,1)],{'K=5 Nm/rad','K=30 Nm/rad','Wall angle'},'Time (s)','Joint angle (rad)','Impedance against a unilateral compliant contact');
mc_plot(outdir,'L17_contact',t,Y(:,[2 4]),{'Soft controller','Stiff controller'},'Time (s)','Contact torque (Nm)','Higher command stiffness can increase contact loads');

mc_save(outdir, 'L17', r);
end
```

---

<a id="lab18"></a>
## L18｜母线回灌与制动电阻

### 1. 这次要回答的问题

**9J动能为什么能让48V母线远超额定值？**

对应分册：[10 保护状态机·再生制动·故障诊断](./10_保护状态机_再生制动_故障诊断.md)、[13 项目案例·面试表达·实验报告](./13_项目案例_面试表达_实验报告.md)。

### 2. 对象、假设与计算步骤

J2e-4、初速300rad/s，预设.25s线性减速，初始48V与2.2mF母线。理想回馈功率由机械减速给定；比较无吸能与20Ω电阻、52V开50V关滞回，时间步长.1ms。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(18);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab18_regeneration.m](../matlab/labs/lab18_regeneration.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L18`读取输出。

### 4. 代码阅读抓手

Ea/Eb：两组母线能量；Preg：预定再生功率；on：滞回开关；Pbr：瞬时耗散；diss：累计电阻能量。

数据列（按顺序）：

```text
time_s, speed_rad_s, bus_no_sink_V, bus_brake_V, regen_W, brake_W, dissipated_J
```

### 5. 结果应怎样看

无吸能电压解析最终约102.40V，数值约102.42V；有制动最大约52.06V。功率图的脉冲峰值和总能量是不同选型维度。积分误差来自有限步长，默认没有让真实控制器产生这一减速轨迹。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `analytic_final_no_sink` | 102.4002841 |
| `numeric_final_no_sink` | 102.4162629 |
| `max_braked_voltage` | 52.05771061 |
| `initial_mechanical_J` | 9 |

原始数据：[CSV](../reference_results/L18_data.csv) · [指标JSON](../reference_results/L18_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![有限电容吸收9J动能的危险电压，仅作仿真。](../figures/L18_bus.png)

**图示解读：**有限电容吸收9J动能的危险电压，仅作仿真。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![瞬时制动脉冲功率与总能量是不同指标。](../figures/L18_power.png)

**图示解读：**瞬时制动脉冲功率与总能量是不同指标。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**电容加倍，先用能量公式预测而不是期待电压升幅简单减半。

**任务2：**改变减速时间，理想总动能相同但峰值功率不同。

**任务3：**改变电阻与滞回，比较最高母线、瞬时电阻功率和耗散能量。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

危险过压只作数学演示；无真实电池/BMS、驱动限压、电流环、制动器热模型及故障认证。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab18_regeneration(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L18'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=1e-4;t=(0:h:.4)';J=2e-4;w0=300;Tb=.25;C=.0022;V0=48;Rb=20;N=numel(t);
w=w0*max(0,1-t/Tb);Preg=J*w*(w0/Tb);E0=.5*C*V0^2;Ea=E0;Eb=E0;on=false;D=zeros(N,5);diss=0;
for k=1:N
 va=sqrt(2*Ea/C);vb=sqrt(2*Eb/C);
 if vb>52,on=true;elseif vb<50,on=false;end
 Pbr=double(on)*vb^2/Rb;D(k,:)=[va,vb,Preg(k),Pbr,diss];
 if k<N
  Ea=Ea+h*Preg(k);Eb=Eb+h*(Preg(k)-Pbr);diss=diss+h*Pbr;
 end
end
r.data=[t,w,D];r.columns={'time_s','speed_rad_s','bus_no_sink_V','bus_brake_V','regen_W','brake_W','dissipated_J'};
r.metrics.analytic_final_no_sink=sqrt(V0^2+J*w0^2/C);r.metrics.numeric_final_no_sink=D(end,1);r.metrics.max_braked_voltage=max(D(:,2));r.metrics.initial_mechanical_J=.5*J*w0^2;
mc_plot(outdir,'L18_bus',t,D(:,1:2),{'No energy sink','Hysteretic brake resistor'},'Time (s)','DC bus voltage (V)','Finite capacitor energy: ideal braking energy balance');
mc_plot(outdir,'L18_power',t,D(:,3:4),{'Regeneration','Resistor pulse power'},'Time (s)','Power (W)','Pulse power rating and total dissipated energy are different');
assert(abs(r.metrics.numeric_final_no_sink-r.metrics.analytic_final_no_sink)<.1 && r.metrics.max_braked_voltage<54);

mc_save(outdir, 'L18', r);
end
```

---

<a id="lab19"></a>
## L19｜状态机、锁存故障与复位条件

### 1. 这次要回答的问题

**为什么复位和启动必须分开？**

对应分册：[10 保护状态机·再生制动·故障诊断](./10_保护状态机_再生制动_故障诊断.md)。

### 2. 对象、假设与计算步骤

离散事件模型含INIT/CAL/READY/RUN/FAULT。注入一次持续过流、故障期间复位、故障消失后复位、重新启动与通信丢失。状态决定运行许可，故障输入具有优先级。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(19);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab19_protection_state.m](../matlab/labs/lab19_protection_state.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L19`读取输出。

### 4. 代码阅读抓手

state：离散状态；fault_latched：锁存；enable：请求；permit：实际许可；reset：受条件限制的事件。

数据列（按顺序）：

```text
time_s, state_0init_1cal_2ready_3run_4fault, torque_permission, overcurrent, communication_loss, reset_event
```

### 5. 结果应怎样看

对齐事件时刻读状态图：.13s故障期间复位不应清锁存；.18s只回READY；.20s新启动后才RUN；.25s失联后FAULT。图表示逻辑许可，不是实际门极关断时间。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `unsafe_permission_count` | 0 |

原始数据：[CSV](../reference_results/L19_data.csv) · [指标JSON](../reference_results/L19_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![事件、锁存状态与许可；图不是实际硬件保护延迟。](../figures/L19_states.png)

**图示解读：**事件、锁存状态与许可；图不是实际硬件保护延迟。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**把复位与故障同周期触发，验证故障优先。

**任务2：**让校准失败，检查无法进入正常运行。

**任务3：**加入连续自动重试请求，定义最大次数与冷却策略；不能简单无限重启。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

非Stateflow实机模型、无安全认证或模拟比较器响应；实际故障停车还取决于机械和能源。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab19_protection_state(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L19'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=.001;t=(0:h:.32)';N=numel(t);state=0;D=zeros(N,5);
start=false(N,1);start(round([.05,.2]/h)+1)=true;
reset=false(N,1);reset(round([.13,.18]/h)+1)=true;
for k=1:N
 oc=t(k)>=.12 && t(k)<.14;lost=t(k)>=.25;active=oc||lost;
 if active
  state=4; % 故障优先于同周期start/reset，且锁存。
 elseif state==4
  if reset(k),state=2;end
 elseif state==0
  state=1;
 elseif state==1
  if t(k)>=.02,state=2;end
 elseif state==2 && start(k)
  state=3;
 end
 permit=(state==3)&&~active;D(k,:)=[state,permit,oc,lost,reset(k)];
 assert(~(active && permit));
end
r.data=[t,D];r.columns={'time_s','state_0init_1cal_2ready_3run_4fault','torque_permission','overcurrent','communication_loss','reset_event'};
r.metrics.unsafe_permission_count=sum((D(:,3)|D(:,4)) & D(:,2));
mc_plot(outdir,'L19_states',t,D(:,1:2),{'State code','Torque permission'},'Time (s)','Code / boolean','Fault latch: early reset rejected, cleared reset returns READY');
assert(r.metrics.unsafe_permission_count==0 && D(end,1)==4);

mc_save(outdir, 'L19', r);
end
```

---

<a id="lab20"></a>
## L20｜R/L参数辨识与残差

### 1. 这次要回答的问题

**怎样从电压电流数据恢复RL参数，同时避免过度相信拟合？**

对应分册：[12 MATLAB·Simulink·辨识与验证](./12_MATLAB_Simulink_辨识与验证.md)、[13 项目案例·面试表达·实验报告](./13_项目案例_面试表达_实验报告.md)。

### 2. 对象、假设与计算步骤

锁定RL采用精确离散模型，50μs采样、120ms合成电压激励。电流加入确定性小噪声，以Phi\Y拟合a,b，再由R=(1−a)/b、L=−RTs/log(a)恢复参数并重放。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(20);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab20_parameter_identification.m](../matlab/labs/lab20_parameter_identification.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L20`读取输出。

### 4. 代码阅读抓手

Phi：回归矩阵；coef=[a;b]；Rest/Lest：反解物理参数；cond(Phi)：激励与数值条件；ip：拟合模型回放。

数据列（按顺序）：

```text
time_s, voltage_V, true_current_A, measured_current_A, fitted_model_current_A
```

### 5. 结果应怎样看

拟合电流与合成测量很接近，残差帮助观察未解释部分。默认R约.200016Ω、L约.399988mH。它们在同一组拟合数据上评价，不能冒充独立工况精度。电流噪声在回归两侧存在可能引入偏差。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `R_est_Ohm` | 0.2000159433 |
| `L_est_H` | 0.000399987946 |
| `R_relative_error` | 7.971660825e-05 |
| `L_relative_error` | 3.01349454e-05 |
| `condition_Phi` | 1.223673396 |

原始数据：[CSV](../reference_results/L20_data.csv) · [指标JSON](../reference_results/L20_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![同一组合成辨识数据与模型回放，尚非独立验证集。](../figures/L20_fit.png)

**图示解读：**同一组合成辨识数据与模型回放，尚非独立验证集。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![残差结构帮助识别未解释因素，参数值不是唯一结果。](../figures/L20_residual.png)

**图示解读：**残差结构帮助识别未解释因素，参数值不是唯一结果。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**减少输入变化，观察条件数和参数可信度。

**任务2：**增加电流噪声，比较残差与参数偏差。

**任务3：**另生成不同频率/幅值验证集，冻结已估参数做预测；本步骤是必须补的独立验证扩展。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

锁转子合成RL而非实机在线辨识；无死区、传感增益误差、转子运动或磁饱和。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab20_parameter_identification(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L20'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_params();h=50e-6;t=(0:h:.12)';N=numel(t);u=2*(2*mod(floor(t/.001),2)-1)+.5*sin(2*pi*131*t);i=zeros(N,1);
a=exp(-p.R*h/p.Ld);b=(1-a)/p.R;
for k=1:N-1,i(k+1)=a*i(k)+b*u(k);end
im=i+.003*sin(2*pi*1573*t)+.002*cos(2*pi*2331*t);
Phi=[im(1:end-1),u(1:end-1)];coef=Phi\im(2:end);ae=coef(1);be=coef(2);
assert(ae>0 && ae<1 && be>0);Rest=(1-ae)/be;Lest=-Rest*h/log(ae);
ip=zeros(N,1);for k=1:N-1,ip(k+1)=ae*ip(k)+be*u(k);end
r.data=[t,u,i,im,ip];r.columns={'time_s','voltage_V','true_current_A','measured_current_A','fitted_model_current_A'};
r.metrics.R_est_Ohm=Rest;r.metrics.L_est_H=Lest;r.metrics.R_relative_error=abs(Rest-p.R)/p.R;r.metrics.L_relative_error=abs(Lest-p.Ld)/p.Ld;r.metrics.condition_Phi=cond(Phi);
mc_plot(outdir,'L20_fit',t,[im,ip],{'Synthetic measurement','Identified RL model'},'Time (s)','Current (A)','Locked-rotor RL identification with persistent excitation');
mc_plot(outdir,'L20_residual',t,im-ip,{'Measurement minus prediction'},'Time (s)','Residual (A)','Residual is needed; fitted parameters alone do not validate a model');
assert(r.metrics.R_relative_error<.05 && r.metrics.L_relative_error<.05);

mc_save(outdir, 'L20', r);
end
```

---

<a id="lab21"></a>
## L21｜多轴计算与通信预算

### 1. 这次要回答的问题

**为什么MCU还有空闲，总线却已经排不下？**

对应分册：[06 电流采样·编码器·MCU实时执行](./06_电流采样_编码器_MCU实时执行.md)、[11 通信多轴·嵌入式代码·数值验证](./11_通信多轴_嵌入式代码_数值验证.md)。

### 2. 对象、假设与计算步骤

假设50μs截止期、每轴8μs、共享6μs、额外抖动预算4μs，扫描1..8轴。经典CAN按每轴2帧/ms、每帧估计150bit、1Mbit/s计算利用率。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(21);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab21_multi_axis_timing.m](../matlab/labs/lab21_multi_axis_timing.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L21`读取输出。

### 4. 代码阅读抓手

axes：数量；per_axis：假设计算；shared/jitter：共享与裕量；frame_bits/rate：总线预算输入。

数据列（按顺序）：

```text
axes, budget_us, CPU_period_fraction, assumed_CAN_bus_fraction
```

### 5. 结果应怎样看

四轴计算约42μs即84%，总线却120%；两种资源的百分比不能互换。曲线跨100%说明假设预算不可行，并非实测某MCU失败。150bit只是示例开销估计。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `four_axis_budget_us` | 42 |
| `four_axis_CAN_fraction` | 1.2 |

原始数据：[CSV](../reference_results/L21_data.csv) · [指标JSON](../reference_results/L21_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![假设资源预算，CPU与总线分别核算。](../figures/L21_budget.png)

**图示解读：**假设资源预算，CPU与总线分别核算。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**把每轴最坏计算改11μs，再看四轴是否可行。

**任务2：**降低反馈频率或批量打包，重新算消息预算。

**任务3：**另建ADC、编码器SPI与DMA资源表，即使CPU/总线都通过也继续检查并发冲突。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

不是实测WCET、实时调度证明、CAN位填充/仲裁模拟或EtherCAT时钟测试。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab21_multi_axis_timing(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L21'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

axes=(1:8)';period=50;peraxis=8;management=6;jitter_budget=4;
worst=axes*peraxis+management+jitter_budget;util=worst/period;
% 每轴每毫秒一帧命令+一帧反馈；150bit/帧只是显式预算假设，不是标准最坏证明。
can_load=axes*2*1000*150/1e6;
r.data=[axes,worst,util,can_load];r.columns={'axes','budget_us','CPU_period_fraction','assumed_CAN_bus_fraction'};
r.metrics.four_axis_budget_us=worst(4);r.metrics.four_axis_CAN_fraction=can_load(4);
mc_plot(outdir,'L21_budget',axes,[util,can_load,ones(size(axes))],{'CPU period budget','Assumed CAN occupancy','100% boundary'},'Number of axes','Fraction','Compute timing and communication are separate feasibility checks');

mc_save(outdir, 'L21', r);
end
```

---

<a id="lab22"></a>
## L22｜Q15量化与坐标变换误差

### 1. 这次要回答的问题

**定点实现需要如何定义范围、乘积与舍入？**

对应分册：[11 通信多轴·嵌入式代码·数值验证](./11_通信多轴_嵌入式代码_数值验证.md)。

### 2. 对象、假设与计算步骤

对幅值.9的电流矢量以及正余弦分别作Q15量化，宽中间乘加后恢复数值，与浮点变换比较。计算在MATLAB/参考Python双精度中承载宽乘积，重点观察量化链。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(22);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab22_fixedpoint.m](../matlab/labs/lab22_fixedpoint.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L22`读取输出。

### 4. 代码阅读抓手

scale：定点缩放；quantized sin/cos：系数量化；wide products：中间乘加；error：与浮点参考逐样本差。

数据列（按顺序）：

```text
theta_rad, d_float, d_Q15_inputs, q_float, q_Q15_inputs
```

### 5. 结果应怎样看

小的周期误差来自输入和系数同时量化，误差不应被简单归因于某一三角函数。默认不发生目标MCU整数溢出，因为中间数使用宽表示。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `d_max_error` | 3.690049052e-05 |
| `q_max_error` | 3.138743341e-05 |

原始数据：[CSV](../reference_results/L22_data.csv) · [指标JSON](../reference_results/L22_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![Q15输入和三角系数量化误差，未运行目标整数指令。](../figures/L22_quantization.png)

**图示解读：**Q15输入和三角系数量化误差，未运行目标整数指令。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**输入由.9改接近1，明确正满量程边界。

**任务2：**改12/16位量化，比较误差趋势。

**任务3：**用显式int32/int64的C或Rust副本规定舍入与饱和，回放相同向量做位级测试；本包未执行这一目标实现。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

不是完整bit-exact FOC，不包含目标ISA时延或C有符号溢出语义。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab22_fixedpoint(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L22'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

th=linspace(-pi,pi,4001)';x=.9*cos(th);y=.9*sin(th);
q=@(v)min(32767,max(-32768,round(v*32768)));
xq=q(x);yq=q(y);cq=q(cos(th));sq=q(sin(th));
dq=(xq.*cq+yq.*sq)/32768^2;qq=(-xq.*sq+yq.*cq)/32768^2;
% 上式用double精确承载Q15整数乘加，展示量化误差；并非已执行MCU定点指令。
dtrue=x.*cos(th)+y.*sin(th);qtrue=-x.*sin(th)+y.*cos(th);
r.data=[th,dtrue,dq,qtrue,qq];r.columns={'theta_rad','d_float','d_Q15_inputs','q_float','q_Q15_inputs'};
r.metrics.d_max_error=max(abs(dq-dtrue));r.metrics.q_max_error=max(abs(qq-qtrue));
mc_plot(outdir,'L22_quantization',th,[dq-dtrue,qq-qtrue],{'d error','q error'},'Angle (rad)','Normalized error','Q15 input/trigonometric quantization with wide accumulation');
assert(max([r.metrics.d_max_error,r.metrics.q_max_error])<2e-4);

mc_save(outdir, 'L22', r);
end
```

---

<a id="lab23"></a>
## L23｜异步转差与步进静态偏差

### 1. 这次要回答的问题

**电机命令与真实机械位置/速度为什么可能不同？**

对应分册：[02 六步换相·异步与步进电机](./02_六步换相_异步与步进电机.md)。

### 2. 对象、假设与计算步骤

第一支路使用简化归一化转矩—转差关系，最大点s=.2。第二支路用步进静态正弦转矩关系、Nr50，比较负载角与1.8°整步/16细分的命令步长。两条支路独立，非统一电机动态模型。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(23);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab23_other_motors.m](../matlab/labs/lab23_other_motors.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L23`读取输出。

### 4. 代码阅读抓手

slip：转差；sm：归一化最大转矩位置；Nr：转矩角周期参数；delta：静态负载角；microstep：命令分辨率。

数据列（按顺序）：

```text
slip, normalized_induction_torque
```

### 5. 结果应怎样看

异步曲线展示转差与转矩的非线性；步进在.3Tmax负载处偏移约.3492°，大于.1125°命令微步。输出分辨率不是静态负载精度。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `microstep_mechanical_deg` | 0.1125 |
| `static_load_error_deg` | 0.3491520625 |

原始数据：[CSV](../reference_results/L23_data.csv) · [指标JSON](../reference_results/L23_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![简化归一化异步转矩—转差关系，不是完整电磁模型。](../figures/L23_induction.png)

**图示解读：**简化归一化异步转矩—转差关系，不是完整电磁模型。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![静态负载偏角可大于命令微步分辨率。](../figures/L23_stepper.png)

**图示解读：**静态负载偏角可大于命令微步分辨率。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**改变最大转差参数，看曲线最大点移动。

**任务2：**将步进细分从16改64，负载角不因此自动缩小。

**任务3：**加入不同负载比例，指出接近最大静态转矩时模型平衡的敏感性。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

无完整异步电磁状态、动态失步、微步驱动衰减、摩擦或真实标定。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab23_other_motors(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L23'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

s=linspace(-1,1,2001)';sm=.2;T=2*s*sm./(s.^2+sm^2);
% 步进静态平衡：Te=Tmax*sin(Nr*(command-position))。
Nr=50;micro=16;cmd=(0:micro)'*(pi/2/micro)/Nr;load=.3;Tmax=1;position=cmd-asin(load/Tmax)/Nr;
r.data=[s,T];r.columns={'slip','normalized_induction_torque'};
r.metrics.microstep_mechanical_deg=(pi/2/micro)/Nr*180/pi;r.metrics.static_load_error_deg=asin(load/Tmax)/Nr*180/pi;
mc_plot(outdir,'L23_induction',s,T,{'Kloss-type approximation'},'Slip','Normalized torque','Induction-motor torque-slip shape; no real machine parameters');
mc_plot(outdir,'L23_stepper',cmd*180/pi,[cmd,position]*180/pi,{'Command','Loaded equilibrium'},'Command (mechanical deg)','Position (mechanical deg)','Finer microstep resolution does not remove static load deflection');

mc_save(outdir, 'L23', r);
end
```

---

<a id="lab24"></a>
## L24｜开关损耗与一阶热模型

### 1. 这次要回答的问题

**为什么电流翻倍后的温度不能只按瞬时I²R判断？**

对应分册：[01 电机原理·参数与选型](./01_电机原理_参数与选型.md)、[04 PWM·SVPWM·功率驱动](./04_PWM_SVPWM_功率驱动.md)。

### 2. 对象、假设与计算步骤

单器件RMS电流60s时20→40A；R25=4mΩ、温度系数.006/K，48V、20kHz、边沿和60ns。导通与开关损耗进入一阶热网络Rth8K/W、Cth8J/K、环境25°C；Qg30nC、10V栅驱功率单独显示。

### 3. 在MATLAB中运行

```matlab
startup_course;
r = run_lab(24);
disp(r.columns);
disp(r.metrics);
```

源文件：[lab24_loss_thermal.m](../matlab/labs/lab24_loss_thermal.m)。需要公共函数时，优先打开[common目录说明](../matlab/README.md)，不要只复制入口函数。运行后到`results_matlab/L24`读取输出。

### 4. 代码阅读抓手

Irms：单器件有效值；Pcond：温度相关导通损耗；Psw：重叠估计；Pgate：栅驱供电；Tj：一阶节点温度。

数据列（按顺序）：

```text
time_s, junction_C, conduction_W, switching_W, gate_drive_W, device_rms_A
```

### 5. 结果应怎样看

温度慢变化而损耗阶跃，导通损耗还随温度增长；默认180s时约91.4°C。栅驱供电功率约.006W未全部加到MOS结温中。这里Irms已是单器件自身RMS，不再额外乘占空比。

下表为**当前Python参考实际计算值**，不是MATLAB已执行结果，更不是实机指标：

| 指标名 | 默认参考值 |
|---|---:|
| `final_temperature_C` | 91.39789659 |
| `gate_drive_W` | 0.006 |

原始数据：[CSV](../reference_results/L24_data.csv) · [指标JSON](../reference_results/L24_metrics.json)。不同版本、步长或代码修改可能改变数字；先检查定义，不追求无意义地复制最后几位。

![栅驱供电功率单列；器件RMS不重复乘占空比。](../figures/L24_losses.png)

**图示解读：**栅驱供电功率单列；器件RMS不重复乘占空比。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

![单节点温度估计，不是具体器件实测。](../figures/L24_temperature.png)

**图示解读：**单节点温度估计，不是具体器件实测。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

### 6. 改参数与对照实验

**任务1：**频率由20改40kHz，比较开关项而非导通项。

**任务2：**电流阶跃时间/占空工作周期改变，观察热积累。

**任务3：**改变散热Rth，检查最终温度趋势和接近热失稳的模型边界；参数只是教学假设。

每个变体先写预测，再保存结果并解释差异。故意破坏模型前提的变体可能触发原始断言；应另立实验与验收标准，不默默删除失败记录。

### 7. 不应从这组图推断的结论

不是具体MOS选型、热仿真或实测结温；无反向恢复、输出电容能量、多节点热网络和PCB寄生。

### 8. 本实验MATLAB入口源码

以下与包内源码一致；公共函数链接与运行环境见总览。

```matlab
function r = lab24_loss_thermal(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L24'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=.02;t=(0:h:180)';N=numel(t);Ta=25;Rth=8;Cth=8;Tj=Ta;D=zeros(N,5);
for k=1:N
 Irms=20+20*(t(k)>=60);Iedge=Irms;V=48;fpwm=20000;R25=.004;alpha=.006;trtf=60e-9;
 Pcond=Irms^2*R25*(1+alpha*(Tj-25));Psw=.5*V*Iedge*trtf*fpwm;
 Pgate=30e-9*10*fpwm;D(k,:)=[Tj,Pcond,Psw,Pgate,Irms];
 Tj=Tj+h*(Pcond+Psw-(Tj-Ta)/Rth)/Cth;
end
r.data=[t,D];r.columns={'time_s','junction_C','conduction_W','switching_W','gate_drive_W','device_rms_A'};
r.metrics.final_temperature_C=D(end,1);r.metrics.gate_drive_W=D(end,4);
mc_plot(outdir,'L24_temperature',t,D(:,1),{'One-pole thermal estimate'},'Time (s)','Temperature (degC)','Per-device RMS current step: 20A to 40A');
mc_plot(outdir,'L24_losses',t,D(:,2:4),{'Conduction','Switching overlap','Gate-drive supply'},'Time (s)','Power (W)','Gate-drive loss is shown separately, not added to MOSFET junction heat');

mc_save(outdir, 'L24', r);
end
```

---

## 实验记录要求

完成一次运行后使用[实验报告模板](../templates/01_实验报告模板.md)。一次默认PASS只证明所运行的脚本在当前输入与断言下通过，不能代表尚未运行的扩展、真实电机或目标固件。先把所有默认实验在本机MATLAB跑通，再挑专项修改，避免多个未确认因素同时叠加。
