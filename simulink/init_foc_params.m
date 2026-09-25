function p = init_foc_params(varargin)
%INIT_FOC_PARAMS Parameters mirrored from the current C/Rust firmware.
%
% p = init_foc_params()
% p = init_foc_params('closedLoop',true,'duration',10)
%
% The default profile is the current firmware, not an experimental observer.
% controllerVector and plantVector are the only vectors consumed by Simulink;
% their named fields remain available for review, plots and validation.
%
% FluxRT —— simulink/ 目录的参数唯一来源（single source of truth）。
% FluxRT - the single parameter source for the simulink/ folder.
%
% 职责 / Responsibility:
%   - 汇总环路时序、母线/逆变器换算、电机标称参数、PI 增益、启动接管、SMO+PLL
%     观察器与保护门限，并打包成 Simulink 唯一消费的两个定长向量；
%   - controllerVector / plantVector 是进入模型的唯一数据通道，命名字段只用于
%     复核、绘图和校验；改过命名字段后必须调用 foc_refresh_vectors 重新打包。
%   - Collects loop timing, bus/inverter scaling, motor data, PI gains, start-up
%     handoff, the SMO+PLL observer and protection thresholds, then packs them into
%     the two fixed-size vectors Simulink actually consumes. Named fields are for
%     review, plots and validation only: they reach the model exclusively through
%     foc_refresh_vectors.
%
% 固定值来源标签 / Provenance tags（全目录统一 / used folder-wide）:
%   [HW] 实机实测值或功率板硬件参数   measured on the rig, or given by the power board
%   [ST] ST MCSDK 6.4.1 生成工程转录  transcribed from the generated MCSDK project
%   [FW] 当前固件 C/Rust 配置值       value currently compiled into the firmware
%   [OPT] 本目录参数扫描得到          found by the parameter sweep in this folder
%
% 未辨识参数 / Unidentified parameters:
%   Rs、Ld、Lq、磁链、转动惯量和粘滞摩擦全部继承 ST Workbench 数据库，尚未在实物
%   电机上重新辨识（校准顺序见 docs/仿真实机相关性验证.md §6）。其中
%   转动惯量和摩擦是仿真与实机残差的主要来源之一。
%   Rs, Ld, Lq, flux linkage, inertia and viscous friction are inherited from the ST
%   Workbench database and have NOT been re-identified on the physical motor.
%   Inertia and friction in particular are a known source of residual model error.
%
% 边界 / Boundary:
%   - 只返回结构体：不访问串口、不驱动功率级、不改写任何 .m 源文件；
%   - closedLoop 默认 false，与固件上电保持功率级关闭的安全默认一致，闭环仿真必须
%     显式打开。
%   - Returns a struct only: no serial access, no power-stage actuation, no .m source
%     rewriting. closedLoop defaults to false, matching the safe power-on default of
%     the firmware; a closed-loop simulation must be requested explicitly.
%
% 参考 / Reference: simulink/Simulink仿真工程说明.md, docs/仿真实机相关性验证.md

% 场景覆盖项 / Scenario overrides（括号内为默认值 / defaults in parentheses）:
%   observerProfile 观察器剖面，'firmware' 或 'legacy-optimal'
%   closedLoop      是否允许观察器接管并进入速度闭环 [-]，默认 false
%   targetRpm       速度指令 [rpm]，默认 582
%   busVoltage      仿真母线电压 [V]，默认 12.3（实测值）
%   duration        仿真时长 [s]，默认 5.0；StopTime = duration - Ts
%   deadTimeNs      平均死区时间 [ns]，默认 550（功率板硬件死区）
%   loadTorqueNm    负载转矩 [N*m]，默认 0（空载）
%   enable*         死区建模与两级前馈补偿开关 [-]；前馈分调制器前馈和观察器电压
%                   重构补偿两层，可独立 A/B；默认全部关闭，因为 550 ns 与 5 mA
%                   目前只是仿真名义值。

ip = inputParser;
ip.addParameter('observerProfile','firmware',@(s)ischar(s)||isstring(s));
ip.addParameter('closedLoop',false,@(x)islogical(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('targetRpm',582,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('busVoltage',12.3,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('duration',5.0,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('pwmFrequencyHz',12000,@(x)isnumeric(x)&&isscalar(x)&&x>0&&x==fix(x));
ip.addParameter('controlFrequencyHz',12000,@(x)isnumeric(x)&&isscalar(x)&&x>0&&x==fix(x));
ip.addParameter('actuationDelayPwmTicks',1,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x==fix(x));
ip.addParameter('deadTimeNs',550,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('loadTorqueNm',0,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('enableDeadTime',true,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableObserverDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('deadTimeCompensationGain',1.0,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('deadTimeCompensationCurrentBandA',0.005, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.parse(varargin{:});
opt = ip.Results;
if mod(opt.pwmFrequencyHz,opt.controlFrequencyHz) ~= 0
    error('init_foc_params:timing','PWM/control frequencies must have an integer ratio.');
end
if opt.actuationDelayPwmTicks > opt.pwmFrequencyHz/opt.controlFrequencyHz
    error('init_foc_params:timing', ...
        'Actuation delay must not exceed one control period in PWM ticks.');
end

p = struct();
p.meta.createdBy = 'init_foc_params.m';
p.meta.matlabRelease = version('-release');
p.meta.profile = lower(char(opt.observerProfile));
p.meta.firmwareAbi = '0x00080000';
p.meta.configVersion = 4;
p.meta.simulationConfigVersion = 3;
% firmwareAbi 是这套参数所镜像的固件 ABI 版本（0x00080000，主/次版本编码）。
% 参数向量长度、字段顺序与语义都绑定该版本：固件侧 FOC_RUST_ABI_VERSION 变化时，
% 这里的镜像必须同步，否则"参数一致"只是假象。
% firmwareAbi is the firmware ABI revision these parameters mirror (0x00080000). The
% vector length, field order and semantics are bound to it, so an ABI bump on the
% target invalidates this mirror even if the numbers look unchanged.

%% Timing and test scenario
% 单位 / Units: timerClockHz、pwmFrequencyHz、speedLoopFrequencyHz [Hz]；
% speedLoopDivider、arr [counts]；ts、duration [s]；targetRpm [rpm]；
% loadTorqueNm [N*m]。
%
% 默认 PWM/控制均为 12 kHz（控制 Ts = 83.33 us），速度环 1 kHz；候选只把
% PWM 提到 24 kHz，完整控制仍为 12 kHz。arr = f_timer/(2*f_pwm)。[FW]
% 控制频率与 Rust CONTROL_PWM_HZ 一致，载波频率由目标平台层拥有。
p.timing.timerClockHz = 170e6;
p.timing.pwmFrequencyHz = opt.pwmFrequencyHz;
p.timing.controlFrequencyHz = opt.controlFrequencyHz;
p.timing.speedLoopFrequencyHz = 1000;
p.timing.speedLoopDivider = ...
    p.timing.controlFrequencyHz/p.timing.speedLoopFrequencyHz;
p.timing.pwmTicksPerControl = ...
    p.timing.pwmFrequencyHz/p.timing.controlFrequencyHz;
p.timing.actuationDelayPwmTicks = opt.actuationDelayPwmTicks;
p.timing.pwmTs = 1/p.timing.pwmFrequencyHz;
p.timing.controlTs = 1/p.timing.controlFrequencyHz;
% `ts` 是旧脚本兼容别名，语义固定为控制周期；新代码必须显式选 pwmTs/controlTs。
p.timing.ts = p.timing.controlTs;
p.timing.arr = p.timing.timerClockHz/(2*p.timing.pwmFrequencyHz);
p.timing.duration = opt.duration;
p.timing.targetRpm = opt.targetRpm;
p.loadTorqueNm = opt.loadTorqueNm;

%% Bus, motor and inverter
% 母线 / Bus: nominalV、minimumV、maximumV [V]；partitionFactor [-] 为分压比 1/16；
% adcFullScale [counts] 是 12 位满量程；adcRefV [V] 是 ADC 参考电压。
% [HW] 12.3 V 是实测母线；[FW] 7..18 V 窗口和 1.15 A 跳闸来自平台配置。
% 自洽性检查：bus_mv 的满量程 52800 mV = 3.3 V/0.0625*1000，与之互为反算。
% 注意这些换算字段大多只用于参数推导和文档：partitionFactor、adcFullScale、adcRefV、
% physicalAdcCountsPerAmp 都不进 Simulink 向量，模型是平均值逆变器，不建模 ADC 量化、
% 零偏与噪声（见 pmsm_plant.m 的模型边界）。
%
% 电机 / Motor: polePairs [-]、Rs [ohm]、Ld/Lq [H]、fluxLinkageWb [Wb]、
% ratedCurrentA [A]、maxSpeedRpm [rpm]、nominalBusV [V]、inertiaKgM2 [kg*m^2]、
% frictionNmS [N*m*s]。
% [ST] 极对数、Rs、Ld/Lq、额定电流、最高转速、磁链来自 MCSDK 6.4.1 Workbench 生成
% 工程（NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T），未经实物辨识。
% 磁链写成 0.034739897/(2*pi)：Workbench 的 M1_MOTOR_RATED_FLUX 用自己的角度基准，
% 除以 2*pi 才得到 [Wb]。
% [ST] inertiaKgM2 = 0.291e-4 与 frictionNmS = 0.937e-5 只是 Workbench 的仿真估计，
% 不是辨识数据，是已知的残差误差来源。
%
% Bus: nominalV/minimumV/maximumV in [V], partitionFactor [-] is the 1/16 divider,
% adcFullScale in [counts] and adcRefV in [V]. Motor: polePairs [-], Rs [ohm], Ld/Lq
% [H], fluxLinkageWb [Wb], ratedCurrentA [A], maxSpeedRpm [rpm], nominalBusV [V],
% inertiaKgM2 [kg*m^2], frictionNmS [N*m*s]. [ST] the electrical values come from the
% generated MCSDK 6.4.1 Workbench project and are NOT identified on the real motor;
% the flux constant is divided by 2*pi because Workbench stores it in its own angular
% basis. [ST] inertia and friction are Workbench estimates, not identified data, and
% are a known source of residual model error. Most of the scaling fields above are for
% derivation and documentation only: partitionFactor, adcFullScale, adcRefV and
% physicalAdcCountsPerAmp never enter a Simulink vector, because the model is an
% average-value inverter with no ADC quantisation, offset or noise.
p.bus.nominalV = opt.busVoltage;
p.bus.minimumV = 7.0;
p.bus.maximumV = 18.0;
p.bus.partitionFactor = 0.0625;
p.bus.adcFullScale = 4095;
p.bus.adcRefV = 3.3;

p.motor.polePairs = 7;
p.motor.Rs = 5.29;
p.motor.Ld = 1.058e-3;
p.motor.Lq = 1.058e-3;
p.motor.fluxLinkageWb = 0.034739897/(2*pi);
p.motor.ratedCurrentA = 0.8;
p.motor.maxSpeedRpm = 1572;
p.motor.nominalBusV = 13.0;
p.motor.inertiaKgM2 = 0.291e-4;
p.motor.frictionNmS = 0.937e-5;

% 逆变器 / Inverter:
% shuntOhm [ohm]、amplifierGain [-] 取自 IHM16M1 参考工程（[ST]，板级硬件值）。
% physicalAdcCountsPerAmp [counts/A] 是 12 位实际 ADC 的换算；mcsdkNormalizedCountsPerAmp
% [-] 是 MCSDK 的 Q15 归一化换算（65536 满量程），PI 增益换算用的是后者，两者不能混用。
% dutyMin/dutyMax [-]：平台层占空比窗口，给自举电容充电和高侧驱动留出最小脉宽。
% deadTimeNs [ns] 是功率板硬件死区；deadTimeVoltageAtNominalBusV [V] 是额定母线下的
% 每相平均损失 2*t_dead/Ts*Vbus，12.3 V/12 kHz/550 ns 下约 0.1624 V。
% deadTimeCompensationGain [-] 为前馈增益，deadTimeCompensationCurrentBandA [A] 为
% 过零附近的线性极性带（避免符号跳变引起抖振）。
% 默认关闭补偿：550 ns 与 5 mA 目前只是仿真名义值，实机首次试验应从 0.75 或更低增益
% 开始（见 Simulink仿真工程说明.md「死区补偿仿真」）。
%
% shuntOhm [ohm] and amplifierGain [-] are board values from the IHM16M1 reference
% project. physicalAdcCountsPerAmp [counts/A] is the real 12-bit ADC scaling while
% mcsdkNormalizedCountsPerAmp [-] is the MCSDK Q15 normalisation; the PI gain
% conversion uses the latter, and the two must not be mixed. dutyMin/dutyMax [-]
% leave minimum pulse width for bootstrap charging and the high-side driver.
% deadTimeNs [ns] is the power-board hardware dead time and
% deadTimeVoltageAtNominalBusV [V] the resulting per-phase average loss
% 2*t_dead/Ts*Vbus, about 0.1624 V at 12.3 V / 12 kHz / 550 ns. Compensation is off by
% default because 550 ns and 5 mA are still nominal simulation values.
p.inverter.shuntOhm = 0.33;
p.inverter.amplifierGain = 1.53;
p.inverter.physicalAdcCountsPerAmp = ...
    p.bus.adcFullScale*p.inverter.shuntOhm*p.inverter.amplifierGain/p.bus.adcRefV;
p.inverter.mcsdkNormalizedCountsPerAmp = ...
    65536*p.inverter.shuntOhm*p.inverter.amplifierGain/p.bus.adcRefV;
p.inverter.dutyMin = 0.03;
p.inverter.dutyMax = 0.97;
p.inverter.enableDeadTime = logical(opt.enableDeadTime);
p.inverter.deadTimeNs = opt.deadTimeNs;
p.inverter.deadTimeCompensationEnabled = logical(opt.enableDeadTimeCompensation);
p.inverter.observerDeadTimeCompensationEnabled = ...
    logical(opt.enableObserverDeadTimeCompensation);
p.inverter.deadTimeCompensationGain = opt.deadTimeCompensationGain;
p.inverter.deadTimeCompensationCurrentBandA = ...
    opt.deadTimeCompensationCurrentBandA;
p.inverter.deadTimeVoltageAtNominalBusV = ...
    2*p.inverter.deadTimeNs*1e-9/p.timing.pwmTs*p.bus.nominalV;

%% PI gains: exact formulas from foc-control/src/params.rs
% 电流环与速度环增益的来源全部是 MCSDK 生成工程里的整数增益，换算链路如下：
%   referencePwmHz [Hz]          MCSDK 参考载波 30 kHz，用于把 Ki 保持为连续时间增益；
%   referenceVoltageUtilization [-] MCSDK 的 0.95 调制利用率；
%   referenceMaxVoltage [V]      13 V*0.95/sqrt(3) ≈ 7.13 V，SVPWM 线性区相电压峰值；
%   voltsPerNormalizedCount [V/count] 每 Q15 归一化计数对应的电压；
%   currentGainScale [-]         countsPerAmp*voltsPerNormalizedCount，把整数增益换成 SI。
% currentRawKp/currentRawKiPerTick 与 speedRawKp/speedRawKiPerTick 都是 [ST] 原始整数
% 增益（ADC/电流、归一化电压/Q15 单位），不能直接当 SI 增益使用。
% idKp/iqKp [V/A]、idKi/iqKi [V/(A*s)]；currentOutMin/Max [V] 为 ±referenceMaxVoltage。
%
% Every gain here is derived from the integer gains of the generated MCSDK project:
% 30 kHz reference carrier (so Ki stays a continuous-time gain), 0.95 modulation
% utilisation, 7.13 V linear-range phase peak, and a counts-per-amp to volts-per-count
% scale that converts the integer gains to SI. currentRawKp/currentRawKiPerTick and
% speedRawKp/speedRawKiPerTick are [ST] raw integer gains and must not be used as SI
% gains. idKp/iqKp are in [V/A] and idKi/iqKi in [V/(A*s)].
referencePwmHz = 30000;
referenceVoltageUtilization = 0.95;
referenceMaxVoltage = p.motor.nominalBusV*referenceVoltageUtilization/sqrt(3);
voltsPerNormalizedCount = referenceMaxVoltage/32767;
currentGainScale = p.inverter.mcsdkNormalizedCountsPerAmp*voltsPerNormalizedCount;
p.control.currentRawKp = 3378/1024;
p.control.currentRawKiPerTick = 2252/4096;
p.control.idKp = p.control.currentRawKp*currentGainScale;
p.control.idKi = p.control.currentRawKiPerTick*currentGainScale*referencePwmHz;
p.control.iqKp = p.control.idKp;
p.control.iqKi = p.control.idKi;
p.control.currentOutMin = -referenceMaxVoltage;
p.control.currentOutMax = referenceMaxVoltage;
% 两个利用率含义不同，不要合并：0.95 只用于从 MCSDK 整数增益反推 SI 增益（保持与
% 参考工程一致的电压基准）；0.90 是 applications/main.c 运行时的圆限幅系数，作用于
% 实测母线。模型里电压圆限幅用 0.90，增益换算用 0.95。
p.control.voltageUtilization = 0.90; % applications/main.c runtime override

speedUnitsPerRadS = 10/(2*pi);
% 速度环换算说明 / Speed-loop scaling:
%   MCSDK 的 SPEED_UNIT 取 U_01HZ（10 units/Hz），speedUnitsPerRadS [units/(rad/s)]
%   把它换成角速度单位；speedRawKp [ST] 和 speedRawKiPerTick [ST] 为原始整数增益。
%   speedKp [A/(rad/s)]、speedKi [A/rad]（Ki 在 1 kHz 速度环里按 ki*ts*err 使用）；
%   speedOutMin/Max [A] = ±ratedCurrentA，即速度环输出就是 Iq 指令限幅。
% The MCSDK SPEED_UNIT is U_01HZ (10 units/Hz), so speedUnitsPerRadS converts it to
% angular velocity; the raw integer gains are [ST]. speedKp is in [A/(rad/s)] and
% speedKi in [A/rad] (applied as ki*ts*err at 1 kHz). speedOutMin/Max are ±rated
% current because the speed PI output is the Iq reference itself.
p.control.speedRawKp = 2730/256;
p.control.speedRawKiPerTick = 562/16384;
p.control.speedKp = p.control.speedRawKp*speedUnitsPerRadS/ ...
                    p.inverter.mcsdkNormalizedCountsPerAmp;
p.control.speedKi = p.control.speedRawKiPerTick*speedUnitsPerRadS/ ...
                    p.inverter.mcsdkNormalizedCountsPerAmp* ...
                    p.timing.speedLoopFrequencyHz;
p.control.speedOutMin = -p.motor.ratedCurrentA;
p.control.speedOutMax = p.motor.ratedCurrentA;

%% Rev-up and closed-loop handoff
% 启动接管时序 [FW]（与 Rust RevUpConfig::default 一致）:
%   alignmentS [s]      强制角对齐 1.0 s，先建立确定的转子位置；
%   rampS [s]           开环升速 1.164 s 到 finalSpeedRpm；
%   transitionS [s]     接管过渡 25 ms，沿最短角差把强制角渐变到观察角；
%   finalSpeedRpm [rpm] 升速终点 582，必须高于观察器可靠速度门 524 rpm；
%   finalCurrentA [A]   升速与保持段的 Iq 0.8（额定电流）；
%   closedLoopEnable [-]
%                       1 才允许观察器接管（对应固件 closed_loop_enable）。
% 保护与限速 / Protection and slew limits:
%   observerAcquisitionTimeoutS [s] 0.5：保持段内观察器始终不可靠则锁存故障 bit2；
%   observerLossTimeoutS [s] 0.05：接管后连续失锁 50 ms 锁存故障 bit3；
%   closedLoopSpeedRampRpmPerS [rpm/s] 500：闭环速度参考上升率；
%   speedPiPreloadRatio [-] 0：与 MCSDK PID_SPEED_INTEGRAL_INIT_DIV=0 一致，速度 PI
%     积分从零开始，不做无扰预装；
%   closedLoopCurrentSlewAPerS [A/s] 32：Iq 限速器，消除进入速度环时的单周期阶跃。
%
% Start-up handoff timing [FW], matching Rust RevUpConfig::default: 1.0 s forced-angle
% alignment, 1.164 s open-loop ramp to 582 rpm, then a 25 ms shortest-path angle
% transition. Timing values are in [s], speeds in [rpm], currents in [A], limits in
% [rpm/s] and [A/s], and the preload ratio is dimensionless (0 matches MCSDK
% PID_SPEED_INTEGRAL_INIT_DIV=0, so the speed integrator starts at zero).
p.startup.alignmentS = 1.0;
p.startup.rampS = 1.164;
p.startup.transitionS = 0.025;
p.startup.finalSpeedRpm = 582.0;
p.startup.finalCurrentA = 0.8;
p.startup.closedLoopEnable = double(logical(opt.closedLoop));
p.startup.observerAcquisitionTimeoutS = 0.5;
p.startup.observerLossTimeoutS = 0.05;
p.startup.closedLoopSpeedRampRpmPerS = 500.0;
p.startup.speedPiPreloadRatio = 0.0;
p.startup.closedLoopCurrentSlewAPerS = 32.0;

%% Observer
% 为什么保留两个剖面 / Why two profiles exist:
%   firmware —— 复现实机行为的剖面。[FW] 与 Rust SmoPllTuning::for_motor 一致：
%     kSlideV = nominalBusV*(4/13) = 4.0 V（13 V 额定母线）、boundaryA =
%     ratedCurrentA*0.20 = 0.16 A、emfAlpha = 0.05、PLL 80/1000。它的作用是让仿真
%     出现与实机相同的现象（观察速度均值接近指令、但角度不可独立信任），而不是给出
%     一组"好用"的整定。
%   legacy-optimal（'optimal' 是兼容别名）—— [OPT] 1.5 V / 0.064 A / 0.8，
%     PLL 220/12000。它能让观察器在仿真里锁定，用来演示"修正后的整定"长什么样；
%     但产生它的旧扫描用了错误的电流 PI 换算和 4 ms 可靠性窗口（见 Simulink仿真工程说明.md
%     「旧扫描结果」），所以只用于复现该轮扫描，不能整组照搬到硬件。
%   只有两个剖面都保留，才能把"固件现状"和"修正方向"分开对比，避免把两者混为一谈。
%
% 上一轮调查的两条结论 / Findings from the earlier investigation:
%   1) EMF 提取滤波系数是主导因素。alpha=0.05 在 12 kHz 下等效一阶低通截止约 98 Hz
%      （小系数近似 alpha/(2*pi*Ts) 约 95 Hz）、群延迟约 Ts/alpha ≈ 1.7 ms；582 rpm、
%      7 对极时电频率约 68 Hz，该滤波在电频率处已造成约 35° 相位滞后，SMO 反电势和
%      观察角明显滞后于真实反电势。
%   2) 滑模增益按母线电压取值过大：首轮按额定母线比例取 11.7 V（≈0.9 × 13 V，配 PLL
%      220/12000，见 docs/仿真实机相关性验证.md §5），比下面 legacy-optimal
%      的 1.5 V 大约 8 倍；固件现行比例已收紧到 Vbus*4/13（= 4.0 V @ 13 V）。
%
% `firmware` reproduces the flashed tuning (4.0 V / 0.16 A / 0.05, PLL 80/1000), while
% `legacy-optimal` (alias `optimal`) demonstrates the corrected tuning that actually
% lets the observer lock in simulation (1.5 V / 0.064 A / 0.8, PLL 220/12000). Keeping
% both separates "what the firmware does" from "where the fix points". The earlier
% investigation found the EMF filter coefficient to be the dominant factor (alpha=0.05
% gives roughly a 98 Hz cutoff, about a 95 Hz value under the small-alpha approximation
% alpha/(2*pi*Ts), and 1.7 ms group delay, about 35 degrees of lag at the 68 Hz
% electrical frequency of 582 rpm with 7 pole pairs) and the bus-derived sliding
% gain to be roughly 8x too large: the first round took 11.7 V (about 0.9 x the 13 V
% nominal bus) against the usable 1.5 V, and the firmware ratio has since been tightened
% to Vbus*4/13.
switch p.meta.profile
    case 'firmware'
        % 单位 / Units: kSlideV [V]、boundaryA [A]、emfAlpha [-]、
        % pllKp [1/s]、pllKi [1/s^2]（PLL 以 [rad] 误差产生 [rad/s] 电角速度）。
        % kSlideV 在固件里按额定母线比例给出，这里写成 13 V 下的绝对值；
        % boundaryA 是饱和函数的线性边界；emfAlpha 是反电势低通系数（见上）。
        p.observer.kSlideV = 4.0;
        p.observer.boundaryA = 0.16;
        p.observer.emfAlpha = 0.05;
        p.observer.pllKp = 80.0;
        p.observer.pllKi = 1000.0;
    case {'legacy-optimal','optimal'}
        % Retained only to reproduce the obsolete 2026-09-22 sweep. It is not
        % the current firmware profile and must not be transferred to hardware.
        % [OPT] 该剖面来自本目录的旧参数扫描（216 点那一轮的数据文件
        % data/optimize_grid.csv、data/optimal_observer.json），对应单位与上面相同。
        % 它的价值是：在同一个 plant 上证明"把 alpha 提到 0.8、把滑模增益按母线比例
        % 降到 1.5 V 量级"确实能让观察器锁定；但那一轮扫描的电流 PI 换算和可靠性窗口
        % 是错的，所以这批数值只能当方向性证据，不能整组上板。
        % [OPT] comes from the obsolete 216-point sweep stored in data/. Units are the
        % same as above. Its value is showing on one plant that raising alpha and
        % reducing the bus-derived sliding gain let the observer lock; the sweep's
        % current-PI scaling and reliability window were wrong, so treat these numbers
        % as directional evidence only, never as a hardware configuration.
        p.meta.profile = 'legacy-optimal';
        p.observer.kSlideV = 1.5;
        p.observer.boundaryA = 0.064;
        p.observer.emfAlpha = 0.8;
        p.observer.pllKp = 220.0;
        p.observer.pllKi = 12000.0;
    otherwise
        error('init_foc_params:profile', ...
              'observerProfile must be firmware or legacy-optimal.');
end
% 观察器模型与可靠性门 / Observer model and reliability gate:
%   Rs [ohm]、Ls [H] 直接复用电机标称值（SMO 用 d 轴电感；本电机 Ld=Lq）；
%   pllOmegaMin/pllOmegaMax [rad/s] 是 PLL 积分出来的电角速度钳位，防止失锁时飞车；
%   reliabilityFifoLength [samples]、reliabilityDecimatorHz [Hz]、reliabilityDecimator [-]
%     决定可靠性窗口：先按 1 kHz 抽取，再存 64 点，即 64 ms 窗口（早期版本误按
%     16 kHz 存 64 点，只有 4 ms，这是被修正过的错误）；
%   reliabilityMinMeanRpm [rpm] = maxSpeedRpm/3 = 524，低于此速度不认为可靠；
%   reliabilityMinEmfV [V] 0.25 是反电势幅值门（判据里比较的是平方）；
%   reliabilityMaxVarRatio [-] 0.01 要求速度方差小于均值平方的 1%；
%   reliabilityWindows [-] 2 是连续满足次数，用于抑制单点毛刺误判。
% Rs in [ohm] and Ls in [H] reuse the motor nameplate values. The PLL speed clamp is
% in [rad/s]. The reliability window is decimated to 1 kHz and then 64 samples long,
% i.e. 64 ms (an earlier revision used 64 samples at 16 kHz and therefore only 4 ms,
% which is a corrected bug). The remaining gates are [rpm], [V], [-] and [-].
p.observer.Rs = p.motor.Rs;
p.observer.Ls = p.motor.Ld;
p.observer.pllOmegaMin = -2000.0;
p.observer.pllOmegaMax = 2000.0;
p.observer.reliabilityFifoLength = 64;
p.observer.reliabilityDecimatorHz = 1000;
p.observer.reliabilityDecimator = ...
    p.timing.controlFrequencyHz/p.observer.reliabilityDecimatorHz;
p.observer.reliabilityMinMeanRpm = 524.0;
p.observer.reliabilityMinEmfV = 0.25;
p.observer.reliabilityMaxVarRatio = 0.01;
p.observer.reliabilityWindows = 2;

%% Limits and current hardware references
% 保护门限 / Protection limits:
%   softwareTripA [A] 1.15 是平台层软件过流跳闸（额定 0.8 A 的 1.44 倍，高于正常
%     峰值、低于堵转）[HW]；
%   isrDeadlineCycles [cycles] 12500 是 ISR 截止周期，为 12 kHz 周期 14167 cycles 的
%     88%，留出裕量 [FW]。
% 实机参考量 / Hardware references（全部 [HW]，来自 582 rpm、12.3 V、12 kHz 开环
% 采集）: openLoopReliableSamples [samples]、hardwareObserverMeanRpm 与 StdRpm [rpm]、
% hardwareIqRmseA/hardwareIdRmseA [A]、hardwarePeakPhaseCurrentA 与
% hardwareLatchedPeakCurrentA [A]。
% hasMeasuredShaftSpeed=false 是本目录最重要的边界：实机没有编码器/测速仪真值，
% 上面这些 rpm 是观察器估算，不能当作轴端真值，也不能用于验证观察角精度。
%
% softwareTripA is in [A] and isrDeadlineCycles in [cycles]. All reference figures are
% [HW], captured open-loop at 582 rpm / 12.3 V / 12 kHz: samples in [samples], speeds
% in [rpm], currents in [A]. hasMeasuredShaftSpeed=false is the key boundary here -
% there is no encoder or tachometer truth on the rig, so those rpm numbers are observer
% estimates, not shaft speed, and cannot validate observer angle accuracy.
p.limits.softwareTripA = 1.15;
p.limits.isrDeadlineCycles = 12500;
p.reference.trace = '../simulation/results/hardware_582rpm_12v3_12khz_openloop_run1_20260923.csv';
p.reference.openLoopReliableSamples = 126;
p.reference.hardwareObserverMeanRpm = 582.230158730159;
p.reference.hardwareObserverStdRpm = 31.4006783871719;
p.reference.hardwareIqRmseA = 0.009186;
p.reference.hardwareIdRmseA = 0.008869;
p.reference.hardwarePeakPhaseCurrentA = 0.838;
p.reference.hardwareLatchedPeakCurrentA = 0.903;
p.reference.hasMeasuredShaftSpeed = false;

% 最后一步必须重新打包：Simulink 只看到 controllerVector/plantVector，任何在
% 命名字段上的覆盖（本函数入参、run_foc_sim 的 tuning 覆盖）都要在这里落到向量里，
% 否则仿真会静默使用旧向量。
% The pack must run last: Simulink only ever sees controllerVector/plantVector, so any
% named override made above must be folded into the vectors here or the simulation would
% silently keep using stale values.
p = foc_refresh_vectors(p);
end
