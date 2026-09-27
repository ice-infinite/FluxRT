# 第08册｜MTPA · 弱磁 · 运行包络

> 《电机控制：原理、案例与MATLAB实验》 · 版本1.0 · 2026-09-27
> 原题范围：Q089, Q090, Q091, Q092, Q093, Q094, Q095, Q096
> 本册对应实验：[L13 MTPA数值参考搜索](14_MATLAB实验逐步手册.md#lab13)；[L14 电压电流约束与高速运行包络](14_MATLAB实验逐步手册.md#lab14)

[返回总览](00_学习总览与环境指南.md) · [200题索引](99_200题分册与实验索引.md) · [实验手册](14_MATLAB实验逐步手册.md) · [验证边界](16_验证记录与已知边界.md)

**阅读方式：**先读下面连贯教程，再看原题逐项讲解与新增案例，最后按实验手册运行和改参数。正文所有数值模型为教学设定，不是你的电机实测参数。原题答案完整保留其主体内容，并补充新的案例与验证任务；不是只按标题切割原文件。

**执行边界：**本包的47张结果图来自独立Python参考实现。MATLAB主实验源码共24项，未在交付环境原生执行；扩展任务、PLL/HFI和动态弱磁等未完成项均明确标注。基础MATLAB实验不要求专业控制工具箱，Simulink入门模型另需Simulink。

## 一、先把“要什么”和“做得到什么”写成数学条件

低速希望用更小电流产生给定转矩，高速还必须让所需电压位于逆变器能力内。以等幅值 dq 电流表示：

$$T_e=\frac32p[\psi_f+(L_d-L_q)i_d]i_q,$$
$$i_d^2+i_q^2\le I_{max}^2.$$

本册使用独立 IPMSM 教学参数：R=0.2 Ω，Ld=0.6 mH，Lq=1.0 mH，ψ=0.012 Wb，p=4，Imax=12 A，Vdc=48 V。**这不是前几册 SPMSM 的另一种写法，而是第二台虚拟电机。**需要凸极性才能观察这里的磁阻转矩收益。[S03]

## 二、MTPA：给定转矩，最小化电流平方

目标可以写成：

$$\min_{i_d,i_q}(i_d^2+i_q^2),\quad
\text{s.t.}\; T_e(i_d,i_q)=T^*.$$

在某个 id 候选下，可以直接解出：

$$i_q=\frac{T^*}{(3p/2)[\psi_f+(L_d-L_q)i_d]}.$$

因此 L13 沿负 id 网格扫描，算出 iq，排除分母异常、超电流和不符合当前电动象限的候选，再选最小电流幅值。这样不需要 Optimization Toolbox，也便于直观看到约束与最优点。它是数值近似，网格有限，不能宣称解析全局解精确到任意位。

Lq>Ld 时，负 id 使磁阻转矩项对正转矩有贡献。但 id 本身也占用电流幅值，太负会得不偿失。图中先比较 id、iq，再比较电流幅值，不能只看到 iq 下降便认定总铜耗下降。默认最高 0.8 N·m 目标点较 id=0 的电流幅值减少约 0.574 A。

### 验证任务

把 Lq 改到与 Ld 相同。最优 id 应回到零附近，因为磁阻转矩不再提供收益。然后把网格从 4001 点改成 201 点，观察电流参考阶梯与目标误差。最后增加一个简化铁耗权重，体会“最小电流”与“最高总效率”不是同一个优化问题。最后这一步是扩展任务，基础脚本未包含铁耗模型。

## 三、高速为什么要弱磁

稳态 $\dot i_d=\dot i_q=0$ 时：

$$v_d=Ri_d-\omega_eL_qi_q,$$
$$v_q=Ri_q+\omega_e(L_di_d+\psi_f).$$

电压约束为 $v_d^2+v_q^2\le(V_{dc}/\sqrt3)^2$。随着转速提高，速度相关电势增大；即使电流没有达到上限，所需电压也可能不可实现。负 id 减小 $L_di_d+\psi_f$，给 q 轴电压留空间，但同时占用电流并改变 d 轴电压需求。它是满足电压约束的参考分配，不是神奇地“额外产生母线电压”。[S21]

忽略 R 时，电压边界是 id/iq 平面中的椭圆；保留 R 后一般有旋转交叉项，但仍可作为二次约束分析。温升导致 R 改变会移动实际边界，不能只凭冷机模型确定所有高速裕量。

## 四、L14：先算运行包络，再谈动态控制

L14 在每个速度点扫描二维电流网格，保留同时满足电流圆和电压限制的点，选择其中最大转矩。再与 id=0 的可行点比较。某速度下 id=0 没有可行点，图中用 NaN 断开，**不是把它解释成真实电机转矩恰好为零**。

默认扫描到 1200 rad/s 机械速度，高速端可行最大转矩约 0.23046 N·m。这个值只代表当前简化参数、网格、象限和约束，不包含退磁边界、铁耗、机械限速、温度和真实器件压降。也没有保证该最优点能够通过稳定动态过程到达。

## 五、从包络变成真实弱磁控制还缺什么

真实控制器通常根据电压利用率或裕量调节负 id，结合转矩参考和电流圆重新分配 iq，再处理积分抗饱和、参考变化率和模式切换。在超速时还需校验退磁限制、位置估计稳定性、母线和负载能量。

MTPV 关注电压受限条件下的转矩利用，不应把整个弱磁区都简单命名为 MTPV。MTPA、弱磁和 MTPV 边界随参数与限制变化。**L13 是 MTPA 参考扫描，L14 是稳态可行包络，均不是已完成的动态 MTPV/弱磁控制器。**[S03][S21]

## 六、一个完整学习作业

先用 L13 得到低速 id/iq 参考，再用 L14 标出它在哪个速度开始超电压。将该工作点代入本册稳态方程，逐项算 vd、vq。然后降低母线到 36 V 重做包络，比较基速变化。最后在 L11 的另存副本中换用 IPMSM 参数，明确重整定电流环和速度转矩映射后，再研究参考调度；不要只把 Ld/Lq 改掉却继续使用 SPMSM 的固定转矩换算。

## 本册代表结果预览

![稳态可行最大转矩；曲线断开代表指定限制下无可行点。](../figures/L14_envelope.png)

**图示解读：**稳态可行最大转矩；曲线断开代表指定限制下无可行点。

> 此图为交付环境中实际执行的 **Python数值参考**。对应MATLAB源码已提供，但本次没有MATLAB/Simulink原生执行。

## 原题逐项精讲与新增案例

以下保留原题的参考回答、前提、追问与易错点；每题补充独立案例和可执行实验或纸面推演任务。原答案提到的附录A—E保存在[原题备份](../source/original_question_bank.md#appendix-a)中；本套新实验不依赖其中的C代码。

 | 题号 | 主题 |
|---|---|
| [Q089](#q089) | MTPA 优化的是什么？ |
| [Q090](#q090) | 为什么 IPMSM 的最优 Id 可能为负？ |
| [Q091](#q091) | 基速是什么？为什么高速受电压限制？ |
| [Q092](#q092) | 弱磁控制的目标与代价是什么？ |
| [Q093](#q093) | 电流限制圆与电压限制椭圆怎样写？ |
| [Q094](#q094) | MTPA、弱磁和 MTPV 各适用什么区间？ |
| [Q095](#q095) | 怎样协调弱磁、转矩要求和限幅？ |
| [Q096](#q096) | 解析公式与查表方案如何选？ |

---

<a id="q089"></a>
### Q089. MTPA 优化的是什么？

**参考回答：**Maximum Torque Per Ampere 以电流幅值为约束提高转矩，或在给定转矩下最小化所需电流幅值。对本文线性模型，可以把转矩方程与 $i_d^2+i_q^2$ 的约束结合，求最优 Id/Iq 分配。

这通常有利于降低铜耗，但不是自动使整个驱动系统总损耗最小；铁耗、逆变器开关损耗、温度和机械损耗不一定由该目标覆盖。

**追问：**MTPA 是提高峰值转矩还是提高效率？它的数学目标是转矩/电流关系，其他收益要结合约束和损耗模型验证。[S03]

#### 新增案例：把原理放进具体情境

同一目标 .8 N·m，id=0 与负 id＋较低 iq 可能使用不同总电流。MTPA 优化的是电流矢量幅值，不是单独 iq。

#### MATLAB验证 / 工程推演任务

运行 L13，比较 sqrt(id²+iq²) 与两部分转矩；确认同一目标、同一电机参数与同一电流定义。

---

<a id="q090"></a>
### Q090. 为什么 IPMSM 的最优 Id 可能为负？

**参考回答：**常见 IPMSM 有 $L_q>L_d$，因此转矩中的 $(L_d-L_q)i_di_q$ 在正 Iq、适当负 Id 时为正。把一部分电流分配到负 d 轴，可获得额外磁阻转矩，在某些工作点比全用 q 轴更有效。

最优值由磁链、两轴电感和目标转矩决定。强饱和或交叉饱和时，固定参数解析关系可能偏离真实最优轨迹。

**易错点：**把所有 PMSM 都设一个固定负 Id；对近似非凸极电机，在低速可能只增加铜耗而不增加相应转矩。[S03]

#### 新增案例：把原理放进具体情境

本册 IPMSM 的 Lq>Ld，使负 id 与正 iq 的磁阻项为正。若把 Lq 改成 Ld，该收益消失，最优负 id 应回到零附近。

#### MATLAB验证 / 工程推演任务

做这一参数对照，记录最优 id 曲线变化；不要把人为修改电感当作实机可随意改变的控制参数。

---

<a id="q091"></a>
### Q091. 基速是什么？为什么高速受电压限制？

**参考回答：**在指定母线、限流和参考策略下，当保持目标转矩所需电压达到逆变器可用电压边界时，对应的速度可称这一条件下的基速。它不是完全独立于负载和供电的固定物理常数。

随速度升高，$\omega_e\psi_f$ 和耦合电压增加，PI 可用来改变电流的剩余电压变小。即使电流没有超过额定值，也可能无法跟踪目标电流。

**追问：**母线从 48 V 降到 40 V 后基速会怎样？通常可用电压降低，必须重新评估工作包络，而不能保持原有峰值速度承诺。[S21]

#### 新增案例：把原理放进具体情境

低速电流受限时还有电压裕量，高速可能在电流没满之前先碰到电压边界。这就是仅增大电流限制不能继续提速的原因之一。

#### MATLAB验证 / 工程推演任务

从 L14 取两个速度点，分别计算电流范数和电压范数并标出哪个约束活跃。

---

<a id="q092"></a>
### Q092. 弱磁控制的目标与代价是什么？

**参考回答：**对适用 PMSM，通过负 d 轴电流降低等效 d 轴磁链，减小高速电压需求，从而在母线不变时扩展转速范围。它不是让永磁体永久变弱，而是运行时用定子电流抵消部分磁链。

负 Id 占用总电流容量，通常减少可分配 Iq，并增加损耗和温升；还必须遵守退磁电流、磁钢温度和机械最高转速边界。

**易错点：**把弱磁理解成免费增加功率，或把不可逆退磁误当成正常控制效果。[S21]

#### 新增案例：把原理放进具体情境

负 id 降低等效 d 轴磁链，但会占用电流容量，且可能引入退磁风险。高速弱磁不是“负电流越大越好”。

#### MATLAB验证 / 工程推演任务

在 L14 的某高速度点扫描 id，观察可行 iq 与转矩；额外列出基础模型尚未包含的退磁边界。

---

<a id="q093"></a>
### Q093. 电流限制圆与电压限制椭圆怎样写？

**参考回答：**电流幅值约束是：

$$
i_d^2+i_q^2\le I_{max}^2
$$

稳态且暂时忽略 Rs、动态项时，电压约束为：

$$
(\omega_e L_q i_q)^2+
\left[\omega_e(L_d i_d+\psi_f)\right]^2\le V_{max}^2
$$

在 Id/Iq 平面形成随速度变化的椭圆。真实 Rs、参数非线性和动态项会改变边界。

**自测要求：**能解释速度升高时电压允许区域为什么收缩，以及最优工作点为什么可能从 MTPA 轨迹移动到电压边界。[S03][S21]

#### 新增案例：把原理放进具体情境

电流边界是圆，电压边界随速度变化。忽略 R 可直观看到椭圆，保留 R 后约束中的交叉项也要计入，不能把旧图不加修改套用。

#### MATLAB验证 / 工程推演任务

用本册公式给固定速度画等电压线与电流圆，叠加 L14 选出的点，检查可行性而不是只看最优转矩数值。

---

<a id="q094"></a>
### Q094. MTPA、弱磁和 MTPV 各适用什么区间？

**参考回答：**电压裕量充足时可按 MTPA 分配电流；达到电压边界后，需要沿满足电压/电流约束的轨迹进行弱磁；在适用电机和高速约束区，可使用最大转矩每电压 MTPV 轨迹进一步选择工作点。

不是每台电机或驱动器都会进入完整的三个区域。区域划分依赖凸极性、磁链、限流、机械限速和母线能力。

**易错点：**把 MTPV 当成所有电机达到某固定 rpm 就开启的模式。应先画出该电机的约束图。[S03]

#### 新增案例：把原理放进具体情境

MTPA 优化电流，MTPV 关注电压约束下的转矩利用；弱磁描述高速电压限制下调整磁链的过程。三者不是互相替换的名称。

#### MATLAB验证 / 工程推演任务

在 L14 包络上标注哪些点受电流/电压同时约束；不要将所有负 id 工作点直接标成 MTPV。

---

<a id="q095"></a>
### Q095. 怎样协调弱磁、转矩要求和限幅？

**参考回答：**参考生成器根据转矩要求和当前母线/速度给 Id/Iq，再施加电流圆、退磁边界、温度和功率限制。电流控制器保留抗饱和；如果使用电压裕量反馈调节 Id，需控制该调节环的带宽和恢复过程。

加减速、母线下跌和转矩阶跃可能同时触发多个限制，应明确优先级和状态转移，并处理退出弱磁时 Id/Iq 的连续性。

**追问：**进入弱磁后角度反馈丢失怎么办？高速能量和电压条件更严苛，必须使用事先验证的故障策略，而不是沿用静止时简单关断的假设。

#### 新增案例：把原理放进具体情境

速度突然增加而 id 参考来不及变化时，稳态包络内的目标也可能暂时不可实现。动态弱磁还需要电压裕量调节与电流参考变化率。

#### MATLAB验证 / 工程推演任务

把 L14 稳态最优值当作候选表，列出进入 L11 前必须增加的限速、限流、抗饱和和参数一致性检查；当前包未实现完整动态弱磁。

---

<a id="q096"></a>
### Q096. 解析公式与查表方案如何选？

**参考回答：**固定参数解析式代码紧凑、可解释，适合作为基线；查表可以覆盖饱和、温度和实测效率，但需要高质量标定、插值连续性、边界和存储管理。也可以采用解析初值加实测修正。

表格应带适用电机版本、温度、单位和电流定义；查表越界时要有安全退化策略。在线辨识不能绕过同样的有效性检查。

**工程落点：**拿独立测试点而非标定数据本身验证转矩误差、损耗和控制稳定性，防止“在已测点看起来很好”。[S08]

#### 新增案例：把原理放进具体情境

解析公式依赖常参数，电感地图随电流变化时查表更贴近对象，但插值、边界和表外行为都需定义。表点越多也不自动意味着可信。

#### MATLAB验证 / 工程推演任务

在 L13 网格由细变粗时看参考台阶；为真实标定表设计温度/电流覆盖与表外拒绝策略，而不是无边界外推。

---

## 本册完成检查

能否在不看答案时解释本册核心因果链？能否先预测改参数后曲线往哪个方向变化？能否指出模型省略的物理因素？把实际结果、失败条件和剩余问题填写到[实验报告模板](../templates/01_实验报告模板.md)。原理理解、MATLAB运行、目标MCU和实机验证是不同完成状态，不合并打勾。

## 引用与进一步阅读

方括号S编号沿用原题官方资料；N编号是本次新增的官方软件接口资料。详见[资料与符号](15_参考资料与符号约定.md)。具体公式、教学案例与实验代码为本教程组织和推导，模型结果只适用于所列条件。


[S01]: https://www.mathworks.com/help/mcb/vector-control-foc-and-dtc.html
[S02]: https://www.mathworks.com/help/mcb/gs/obtain-controller-gains-foc-example.html
[S03]: https://www.mathworks.com/help/mcb/ref/mtpacontrolreference.html
[S04]: https://www.mathworks.com/help/mcb/ref/pwmreferencegenerator.html
[S05]: https://www.mathworks.com/help/mcb/ug/prepare-task-scheduling.html
[S06]: https://www.mathworks.com/help/mcb/sensor-calibration.html
[S07]: https://www.mathworks.com/help/mcb/sensorless-approach.html
[S08]: https://www.mathworks.com/help/mcb/motor-parameter-estimation-and-plant-modelling.html
[S09]: https://www.mathworks.com/help/mcb/deployment-and-validation.html
[S10]: https://wiki.st.com/stm32mcu/wiki/STM32MotorControl:Introduction_to_Motor_Control_with_STM32
[S11]: https://docs.odriverobotics.com/v/latest/manual/control.html
[S12]: https://docs.odriverobotics.com/v/latest/manual/hardware-config.html
[S13]: https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control
[S14]: https://www.analog.com/en/resources/analog-dialogue/articles/mastering-precision-understanding-microstepping.html
[S15]: https://www.ti.com/lit/SLYT762
[S16]: https://www.ti.com/lit/an/slva959b/slva959b.pdf
[S17]: https://www.mathworks.com/help/mcb/ref/surfacemountpmsm.html
[S18]: https://www.mathworks.com/help/mcb/ref/parktransform.html
[S19]: https://www.mathworks.com/help/mcb/ref/clarketransform.html
[S20]: https://www.mathworks.com/help/mcb/ref/fieldorientedcurrentcontroller.html
[S21]: https://www.mathworks.com/help/mcb/gs/field-weakening-control.html
[S22]: https://www.can-cia.org/can-knowledge/can-fd-the-basic-idea
[S23]: https://www.mathworks.com/help/mcb/ref/slidingmodeobserver.html
[S24]: https://www.mathworks.com/help/mcb/ref/fluxobserver.html
[S25]: https://www.mathworks.com/help/mcb/ref/extendedemfobserver.html
[S26]: https://www.mathworks.com/help/mcb/ref/pulsatinghighfreqobserver.html
[S27]: https://search.abb.com/library/Download.aspx?Action=Launch&DocumentID=3AXD50000035169&DocumentPartId=&LanguageCode=en
[S28]: https://www.beckhoff.com/en-en/products/i-o/ethercat-terminals/el-ed6xxx-communication/el6692.html
[N01]: https://www.mathworks.com/help/simulink/ug/integrate-ccode-ccaller.html
[N02]: https://www.mathworks.com/matlabcentral/fileexchange/183591-simulink-support-package-for-rust-code
[N03]: https://www.mathworks.com/help/simulink/slref/add_block.html
[N04]: https://www.mathworks.com/help/simulink/slref/sim.html
