function [duty, obsSpeedRpm, state, reliable, ...
          idRef, iqRef, idMeas, iqMeas, vdCmd, vqCmd, ...
          forcedAngle, obsAngle, simTime, phaseCurrents, ...
          controlAngle, speedReferenceRpm, faultFlags] = ...
          controller_core(vbus, iabc, targetRpm, cfg)
%CONTROLLER_CORE Current STM32G431 Rust FOC control chain for Simulink.
% The configuration vector is produced by init_foc_params.m. Tunables stay on
% an input port so simulations never rewrite this source file.
% State: 3 alignment, 4 ramp, 5 hold, 6 transition, 7 closed, 8 fault.
%
% FluxRT —— Simulink 侧完整控制链（观察器 → 启动 → 速度环 → 电流环 → 调制）。
% FluxRT - the full Simulink-side control chain (observer -> start-up -> speed loop ->
% current loop -> modulation).
%
% 职责 / Responsibility:
%   - 按 12 kHz 单步执行：SMO+PLL 观察器、可靠性判据、观察器门控的开环升速与接管、
%     速度 PI、电流 PI、电压圆限幅、相域死区前馈、SVPWM 和故障锁存；
%   - 本文件是 Simulink「ControllerCore」MATLAB Function 块的源码，由
%     make_foc_model.m 注入模型并在保存时固化；它对应固件侧
%     rust/crates/foc-control 与 foc-algorithm 的实现。
%   - One 12 kHz step of SMO+PLL observer, reliability gate, observer-gated rev-up and
%     handoff, speed PI, current PI, voltage circle limit, phase-domain dead-time
%     feed-forward, SVPWM and fault latching. This file is the source of the
%     "ControllerCore" MATLAB Function block, injected by make_foc_model.m and frozen
%     into the saved model; it mirrors rust/crates/foc-control + foc-algorithm.
%
% 参数入口 / Parameters: 所有可调量都从 cfg 向量读入（49 个元素），本文件自身不保存
% 整定常数。cfg 的字段顺序与长度就是协议，必须与 foc_refresh_vectors.m 保持一致。
% Every tunable comes from the cfg vector (49 elements); no tuning constant lives in this
% file. The cfg field order and length are a protocol shared with foc_refresh_vectors.m.
%
% 关键设计（已修复的建模陷阱）/ Critical design, a fixed modelling trap:
%   观察角只有在 CLOSED_LOOP_ENABLE 打开且观察器通过可靠性判据时才允许混入换相角
%   （见 ctrlAngle 一行）。无条件混合会让转子丢转矩：观察角不可信时 Id/Iq 被投影到
%   错误的轴上，电流环看上去仍然"稳定"，但输出的是错误的转矩分量。
%   The observer angle is blended into the commutation angle only when CLOSED_LOOP_ENABLE
%   is set AND the observer is reliable (see the ctrlAngle line). Blending it
%   unconditionally makes the rotor lose torque: with an untrustworthy angle Id/Iq are
%   projected onto the wrong axes, so the loop still looks stable while producing the
%   wrong torque component.
%
% 单位约定 / Units: 端口量纲见各输出声明；占空比 duty 在 [0,1]，本块内部不使用标幺值。
% 状态码 3..8 与固件一致，故障位定义见 faultFlags 处。
% Port units are given at each output; duty stays in [0,1] and no per-unit values are used
% inside the block. States 3..8 match the firmware; fault bits are defined at faultFlags.
%
% 实时约束 / Timing: 本块按固定步长 TS 调用，不得依赖工作区状态；persistent 变量在
% 首次调用时显式初始化，模型重建/清空后必须重新初始化，不能跨仿真实例复用。
% Called at the fixed step TS with no workspace dependency; every persistent is
% initialised on the first call, so a rebuilt or cleared model always restarts clean.
%
% 参考 / Reference: docs/无感闭环接管.md, simulink/Simulink仿真工程说明.md

%%#codegen
persistent init idInt iqInt speedInt
persistent obsTheta obsOmega obsPllInteg obsCurA obsCurB obsEmfA obsEmfB
persistent obsFifo obsFifoIdx obsValid obsGood obsSum obsSquareSum obsDecim
persistent prevDa prevDb prevDc elapsed forcedAngleSt
persistent transitionElapsed transitionStarted closedLoop
persistent handoffSpeedRpm handoffStartIq handoffEndIq
persistent observerWait observerLoss
persistent speedInitialized speedRef speedIqCommand speedCounter appliedIq
persistent faultLatched faultBits

if isempty(init)
    % 首次调用的冷启动：所有积分器、PLL 状态、观察器状态和故障锁存清零。
    % 初始占空比取 0.5，即三相 50%，相电压为零（零矢量），保证未使能时不出力。
    % First-call cold start: every integrator, PLL and observer state and the fault latch
    % is cleared. The initial duty is 0.5 on all phases, i.e. the zero vector, so nothing
    % is applied before the state machine runs.
    init = 1;
    idInt = 0; iqInt = 0; speedInt = 0;
    obsTheta = 0; obsOmega = 0; obsPllInteg = 0;
    obsCurA = 0; obsCurB = 0; obsEmfA = 0; obsEmfB = 0;
    obsFifo = zeros(1,64); obsFifoIdx = 1;
    obsValid = 0; obsGood = 0; obsSum = 0; obsSquareSum = 0; obsDecim = 0;
    prevDa = 0.5; prevDb = 0.5; prevDc = 0.5;
    elapsed = 0; forcedAngleSt = 0;
    transitionElapsed = 0; transitionStarted = 0; closedLoop = 0;
    handoffSpeedRpm = 0; handoffStartIq = 0; handoffEndIq = 0;
    observerWait = 0; observerLoss = 0;
    speedInitialized = 0; speedRef = 0; speedIqCommand = 0;
    speedCounter = 0; appliedIq = 0;
    faultLatched = 0; faultBits = 0;
end

% Configuration layout; see init_foc_params.m::build_controller_vector.
% 配置向量解包：编号与 foc_refresh_vectors.m 的打包顺序严格一致，顺序即协议。
% 单位 / Units（按编号顺序）: PP[-] RS[ohm] LS[H] TS[s] VUTIL[-] DUTYMIN/DUTYMAX[-]
% TRIP_A[A] BUS_MIN/BUS_MAX[V] IKP[V/A] IKI[V/(A*s)] IPI_MIN/IPI_MAX[V]
% ALIGN_S/RAMP_S/TRANS_S[s] FINAL_RPM[rpm] FINAL_A[A] CLOSED_EN[-] SMO_K[V] SMO_BND[A]
% SMO_ALPHA[-] PLL_KP[1/s] PLL_KI[1/s^2] PLL_WMIN/PLL_WMAX[rad/s] REL_DECIM[-]
% REL_MIN_RPM[rpm] REL_MIN_EMF[V] REL_VAR[-] REL_GOOD[-] ACQUIRE_S/LOSS_S[s]
% SPEED_DIV[-] SPEED_KP[A/(rad/s)] SPEED_KI[A/rad] SPEED_MIN/SPEED_MAX[A]
% SPEED_RAMP[rpm/s] PRELOAD[-] IQ_SLEW[A/s] MAX_RPM[rpm] DEADTIME_NS[ns] DT_FF_EN[-]
% DT_FF_GAIN[-] DT_BAND_A[A] DT_OBS_EN[-] PWM_TS[s].
% Configuration unpack: the numbering matches the packing order in foc_refresh_vectors.m
% and the units are listed above in the same order.
PP          = cfg(1);   RS          = cfg(2);   LS          = cfg(3);
TS          = cfg(4);   VUTIL       = cfg(5);   DUTYMIN     = cfg(6);
DUTYMAX     = cfg(7);   TRIP_A      = cfg(8);   BUS_MIN     = cfg(9);
BUS_MAX     = cfg(10);  IKP         = cfg(11);  IKI         = cfg(12);
IPI_MIN     = cfg(13);  IPI_MAX     = cfg(14);  ALIGN_S     = cfg(15);
RAMP_S      = cfg(16);  TRANS_S     = cfg(17);  FINAL_RPM   = cfg(18);
FINAL_A     = cfg(19);  CLOSED_EN   = cfg(20);  SMO_K       = cfg(21);
SMO_BND     = cfg(22);  SMO_ALPHA   = cfg(23);  PLL_KP      = cfg(24);
PLL_KI      = cfg(25);  PLL_WMIN    = cfg(26);  PLL_WMAX    = cfg(27);
REL_DECIM   = cfg(28);  REL_MIN_RPM = cfg(29);  REL_MIN_EMF = cfg(30);
REL_VAR     = cfg(31);  REL_GOOD    = cfg(32);  ACQUIRE_S   = cfg(33);
LOSS_S      = cfg(34);  SPEED_DIV   = cfg(35);  SPEED_KP    = cfg(36);
SPEED_KI    = cfg(37);  SPEED_MIN   = cfg(38);  SPEED_MAX   = cfg(39);
SPEED_RAMP  = cfg(40);  PRELOAD     = cfg(41);  IQ_SLEW     = cfg(42);
MAX_RPM     = cfg(43);  DEADTIME_NS = cfg(44);  DT_FF_EN    = cfg(45);
DT_FF_GAIN  = cfg(46);  DT_BAND_A   = cfg(47);  DT_OBS_EN   = cfg(48);
PWM_TS      = cfg(49);

ia = iabc(1); ib = iabc(2); ic = iabc(3);
phaseCurrents = [ia; ib; ic];
peakPhase = max(abs([ia ib ic]));
% 保护先行 / Protection first: 任一相非数或峰值超过 TRIP_A [A]（软件过流 1.15 A，
% 额定 0.8 A 的 1.44 倍）就锁存 bit0；母线非数或超出 BUS_MIN..BUS_MAX [V] 锁存 bit1。
% 锁存后本周期余下的算法照常运行，但输出被强制回零矢量，见函数末尾。
% Any non-finite phase current or a peak above TRIP_A [A] latches bit0; a non-finite or
% out-of-window bus voltage latches bit1. The rest of the step still executes so the
% state stays observable, but the outputs are forced back to the zero vector (see the
% end of the function).
if ~isfinite(ia) || ~isfinite(ib) || ~isfinite(ic) || peakPhase > TRIP_A
    faultLatched = 1;
    faultBits = double(bitor(uint32(faultBits),uint32(1)));
end
if ~isfinite(vbus) || vbus < BUS_MIN || vbus > BUS_MAX
    faultLatched = 1;
    faultBits = double(bitor(uint32(faultBits),uint32(2)));
end

% Fixed defaults for the latched-fault path and code-generation inference.
% 这些默认值同时服务两件事：① 故障路径先给出安全输出（0.5 占空比 = 零矢量，
% state=8，reliable=0）；② 让代码生成的类型/宽度推断在每个分支上都能取到确定值。
% These defaults serve two purposes: they give the latched-fault path a safe output
% (0.5 duty = zero vector, state 8, reliable 0) and they let code-generation inference
% see a definite value on every branch.
duty = [0.5; 0.5; 0.5];
obsSpeedRpm = obsOmega*30/(pi*PP);
state = 8; reliable = 0;
idRef = 0; iqRef = 0; idMeas = 0; iqMeas = 0; vdCmd = 0; vqCmd = 0;
forcedAngle = forcedAngleSt; obsAngle = obsTheta;
controlAngle = forcedAngleSt; speedReferenceRpm = speedRef;
simTime = elapsed;

if faultLatched == 0
    %% Previous PWM -> SMO -> PLL.
    % 观察器整条链 / The whole observer chain:
    %   以前一周期的占空比重构端电压（含平均死区损失）→ 幅值不变 Clarke → 滑模电流
    %   观察器 → 反电势一阶低通提取 → atan2 得电角 → PLL 跟踪角度与电角速度。
    %   用"上一周期"而不是本周期占空比，是因为本周期占空比要到本函数末尾才算出，
    %   而 ADC 注入采样发生在载波周期开始处，控制器物理上只可能看到上一步的电压。
    % Reconstruct the terminal voltage from the previous period's duty (including the
    % average dead-time loss), take an amplitude-invariant Clarke transform, run the
    % sliding-mode current observer, extract the EMF with a first-order low-pass, get the
    % electrical angle via atan2 and track it with a PLL. The previous period is used
    % because this period's duty is only computed at the end of this function, while the
    % injected ADC sample is taken at the start of the carrier period.
    ialpha = (2*ia - ib - ic)/3;
    ibeta  = (ib - ic)/sqrt(3);
    signA = currentPolarityLocal(ia,DT_BAND_A);
    signB = currentPolarityLocal(ib,DT_BAND_A);
    signC = currentPolarityLocal(ic,DT_BAND_A);
    % 死区电压损失 [V]：一个载波周期里两段死区时间占 Ts 的比例乘以母线电压，
    % 即每相平均损失 2*t_dead/Ts*Vbus（12.3 V、550 ns、12 kHz 下约 0.1624 V）。
    % Dead-time voltage loss in [V]: two dead-time intervals per carrier period times the
    % bus voltage, i.e. 2*t_dead/Ts*Vbus per phase (about 0.1624 V at 12.3 V / 550 ns /
    % 12 kHz). Polarity comes from the banded sign function above.
    deadVoltage = (2*DEADTIME_NS*1e-9/PWM_TS)*vbus;
    % 减去三相共模后才是相对中性点的相电压：SVPWM 的 min-max 注入会给三相叠加同一
    % 个零序分量，而电机只感受相间/相对中性点的电压，共模不产生电流。
    % Removing the three-phase common mode yields the line-to-neutral phase voltage: the
    % min-max injection of SVPWM adds the same zero-sequence component to all phases, and
    % the machine only responds to the differential (line-to-neutral) part.
    common = (prevDa + prevDb + prevDc)/3;
    va = (prevDa - common)*vbus;
    vb = (prevDb - common)*vbus;
    vc = (prevDc - common)*vbus;
    % The SMO must use estimated terminal voltage, not the pre-dead-time PWM
    % command. This term mirrors the plant loss and is independently switchable
    % from the modulator feed-forward for A/B testing.
    % SMO 重构的是实际端电压，所以必须扣掉死区损失：占空比指令是死区作用之前的电压，
    % 直接用会把端电压估高约 deadVoltage [V]，使观察器的电流/反电势估计出现随电流极性
    % 翻转的偏差。DT_OBS_EN 与调制器前馈 DT_FF_EN 是两个独立开关，便于分层消融：前者
    % 修正观察器的电压源，后者修正实际施加的电压，两者不是重复补偿。
    % The observer reconstructs the actual terminal voltage, so the dead-time loss must be
    % subtracted: the duty command is the voltage before dead time acts, and using it as-is
    % overestimates the terminal voltage by about deadVoltage [V], biasing the current and EMF
    % estimates in a way that flips with current polarity. DT_OBS_EN and the modulator
    % feed-forward DT_FF_EN are separate switches on purpose, so the two effects can be
    % ablated independently.
    va = va - DT_OBS_EN*deadVoltage*signA;
    vb = vb - DT_OBS_EN*deadVoltage*signB;
    vc = vc - DT_OBS_EN*deadVoltage*signC;
    valpha = (2*va - vb - vc)/3;
    vbeta  = (vb - vc)/sqrt(3);
    errA = obsCurA - ialpha;
    errB = obsCurB - ibeta;
    % 滑模项 [V]：SMO_K [V] 乘以饱和函数，饱和边界 SMO_BND [A] 决定线性区宽度。
    % 饱和（而不是符号函数）是为了抑制开关抖振：边界越窄越接近纯符号函数，
    % 抖振和噪声放大越强；SMO_K 必须大于最大反电势，否则滑模面不可达。
    % Sliding term in [V]: SMO_K [V] times a saturation whose linear band is SMO_BND [A].
    % Saturation instead of a sign function suppresses chattering; SMO_K must exceed the
    % peak EMF or the sliding surface is not reachable.
    slideA = SMO_K*satLocal(errA/SMO_BND);
    slideB = SMO_K*satLocal(errB/SMO_BND);
    % 电流观察器为正向欧拉离散 [A]：di/dt = (v - R*i - e - u)/L，TS/LS 完成积分。
    % 显式欧拉在这里稳定：电气时间常数 LS/RS = 200 us，TS*RS/LS ≈ 0.417，离散极点
    % |1 - TS*RS/LS| ≈ 0.583（稳定的条件是 TS*RS/LS < 2）。本电机是高阻小电感型号，
    % 时间常数只有 2.4 个采样周期，所以这是本模型里最需要留意数值稳定性的一步。
    % Forward-Euler current observer in [A]: di/dt = (v - R*i - e - u)/L, integrated by
    % TS/LS. Explicit Euler is stable here: the electrical time constant LS/RS is 200 us,
    % TS*RS/LS is about 0.417 and the discrete pole |1 - TS*RS/LS| about 0.583 (stability needs
    % TS*RS/LS < 2). This high-resistance, low-inductance motor has a time constant of only
    % 2.4 sample periods, which makes this the most numerically delicate step in the model.
    obsCurA = obsCurA + (TS/LS)*(valpha - RS*obsCurA - obsEmfA - slideA);
    obsCurB = obsCurB + (TS/LS)*(vbeta - RS*obsCurB - obsEmfB - slideB);
    % 反电势提取就是滑模项的一阶低通 [V]，SMO_ALPHA [-] 是系数。
    % 本目录的调查结论：这个系数是观察器误差的主导因素。alpha=0.05 在 12 kHz 下等效
    % 截止约 98 Hz（小系数近似 alpha/(2*pi*Ts) 约 95 Hz）、群延迟约 Ts/alpha≈1.7 ms；
    % 582 rpm、7 对极的电频率约 68 Hz，滤波在该频率已带来约 35° 相位滞后，反电势相位
    % 落后直接变成观察角误差。
    % EMF extraction is a first-order low-pass on the sliding term, coefficient
    % SMO_ALPHA [-]. The investigation in this folder identified this coefficient as the
    % dominant error source: alpha=0.05 is roughly a 98 Hz cutoff at 12 kHz (about 95 Hz under
    % the small-alpha approximation) with a group delay of about Ts/alpha = 1.7 ms, which is
    % already about 35 degrees of lag at the 68 Hz electrical frequency of 582 rpm with 7 pole
    % pairs, and that phase lag appears directly as observer angle error.
    obsEmfA = obsEmfA + SMO_ALPHA*(slideA - obsEmfA);
    obsEmfB = obsEmfB + SMO_ALPHA*(slideB - obsEmfB);

    % 角度约定 / Angle convention: 反电势为 e_alpha = -omega*psi*sin(theta)、
    % e_beta = +omega*psi*cos(theta)（与 pmsm_plant.m 的 Park 反变换一致），因此
    % theta = atan2(-e_alpha, e_beta)。符号取反会把电角平移 pi，表现为接管后转矩
    % 反向或明显丢转矩，是这类观测器最常见的接线错误。
    % The back-EMF convention is e_alpha = -omega*psi*sin(theta), e_beta =
    % +omega*psi*cos(theta) (consistent with the inverse Park transform in pmsm_plant.m),
    % hence theta = atan2(-e_alpha, e_beta). Flipping either sign shifts the electrical
    % angle by pi and shows up as reversed or badly reduced torque after handoff.
    thetaEmf = mod(atan2(-obsEmfA, obsEmfB), 2*pi);
    pllErr = wrapPiLocal(thetaEmf - obsTheta);
    obsPllInteg = obsPllInteg + PLL_KI*TS*pllErr;
    proportional = PLL_KP*pllErr;
    rawOmega = proportional + obsPllInteg;
    obsOmega = min(max(rawOmega, PLL_WMIN), PLL_WMAX);
    % 电角速度被 PLL_WMIN..PLL_WMAX [rad/s] 钳位时，反算积分器使 PI 输出与钳位后一致，
    % 避免钳位期间积分器继续累积（失锁时尤其重要）。
    % When the electrical speed saturates at PLL_WMIN..PLL_WMAX [rad/s] the integrator is
    % back-calculated so the PI output matches the clamped value instead of winding up.
    if rawOmega ~= obsOmega
        obsPllInteg = obsOmega - proportional;
    end
    obsTheta = mod(obsTheta + obsOmega*TS, 2*pi);
    obsSpeedRpm = obsOmega*30/(pi*PP);

    % Decimate first, then insert: 64 samples at 1 kHz = 64 ms.
    % 先抽取再入队：1 kHz 下 64 点 = 64 ms 窗口。早期版本按 16 kHz 存 64 点，
    % 窗口只有 4 ms，速度方差门形同虚设（已修正）。这里 FIFO 长度硬编码 64，
    % p.observer.reliabilityFifoLength 只出现在参数文件里，改那个字段不会影响本块。
    % Decimate first, then insert: 64 samples at 1 kHz gives a 64 ms window. An earlier
    % revision stored 64 samples at 16 kHz, i.e. only 4 ms, which made the variance gate
    % meaningless. The FIFO length is hard-coded to 64 here while
    % p.observer.reliabilityFifoLength lives in the parameter file only.
    obsDecim = obsDecim + 1;
    if obsDecim >= REL_DECIM
        obsDecim = 0;
        oldest = obsFifo(obsFifoIdx);
        obsSum = obsSum + obsSpeedRpm - oldest;
        obsSquareSum = obsSquareSum + obsSpeedRpm*obsSpeedRpm - oldest*oldest;
        obsFifo(obsFifoIdx) = obsSpeedRpm;
        obsFifoIdx = mod(obsFifoIdx,64) + 1;
        obsValid = min(obsValid + 1,64);
        if obsValid < 64
            obsGood = 0;
        else
            % 可靠性判据 / Reliability criteria（全部要同时成立）:
            %   平均观察转速 meanRpm 在 REL_MIN_RPM [rpm] 与 1.10*MAX_RPM [rpm] 之间
            %     （低于最低可靠速度时反电势太小，观察角没有意义）；
            %   反电势平方和 > REL_MIN_EMF^2，即幅值超过 REL_MIN_EMF [V]；
            %   速度方差 < 均值平方的 REL_VAR（无量纲比值），抑制"数值在动但不可信"。
            % 连续 REL_GOOD 次满足才置 reliable，避免单点毛刺触发接管。
            % All gates must hold together: mean speed within REL_MIN_RPM and
            % 1.10*MAX_RPM [rpm] (below that the EMF is too small to trust), EMF
            % magnitude above REL_MIN_EMF [V], and speed variance below REL_VAR times the
            % squared mean (a dimensionless ratio). REL_GOOD consecutive passes are needed
            % to set reliable, which keeps single-sample glitches from triggering handoff.
            meanRpm = obsSum/64;
            variance = max(obsSquareSum/64 - meanRpm*meanRpm,0);
            emfSq = obsEmfA*obsEmfA + obsEmfB*obsEmfB;
            stable = isfinite(obsSpeedRpm) && meanRpm > REL_MIN_RPM && ...
                     meanRpm < MAX_RPM*1.10 && ...
                     emfSq > REL_MIN_EMF*REL_MIN_EMF && ...
                     variance < meanRpm*meanRpm*REL_VAR;
            if stable
                obsGood = min(obsGood + 1,65535);
            else
                obsGood = 0;
            end
        end
    end
    reliable = double(obsGood >= REL_GOOD);

    %% Observer-gated rev-up and shortest-path handoff.
    % 启动状态机 / Start-up state machine（时间单位 [s]，转速 [rpm]，电流 [A]）:
    %   3 alignment：强制角固定，Iq 从 0 线性升到 FINAL_A，用于把转子拉到已知位置；
    %   4 ramp：强制角按 openSpeedRpm 积分旋转，升速斜率由 FINAL_RPM/RAMP_S 决定；
    %   5 hold：保持 FINAL_RPM 与 FINAL_A，等待观察器通过可靠性判据；
    %   6 transition：在 TRANS_S 内沿最短角差把强制角渐变到观察角，同时 Iq 从接管
    %     瞬间实测的 observerIq 平滑过渡，避免阶跃激励；
    %   7 closed：完全由观察角换相，速度环开始接管 Iq。
    % In states 3..7: alignment holds the forced angle while Iq ramps to FINAL_A, ramp
    % spins the forced angle at the rev-up speed, hold waits for observer reliability,
    % transition blends the forced angle into the observer angle along the shortest path
    % while Iq slews from the measured observer-frame Iq, and closed hands commutation
    % over to the observer completely.
    elapsed = elapsed + TS;
    alignmentComplete = elapsed >= ALIGN_S;
    rampProgress = min(max((elapsed - ALIGN_S)/RAMP_S,0),1);
    openSpeedRpm = FINAL_RPM*rampProgress;
    if ~alignmentComplete
        openPhase = 3; openIq = FINAL_A*elapsed/ALIGN_S;
    elseif rampProgress < 1
        openPhase = 4; openIq = FINAL_A;
    else
        openPhase = 5; openIq = FINAL_A;
    end

    handoffReady = CLOSED_EN ~= 0 && reliable ~= 0;
    % 接管必须同时满足两个条件：固件允许闭环（CLOSED_EN）且观察器已可靠。缺少任一条
    % 都继续开环拖行，绝不允许把不可信的观察角送进换相。
    % Handoff needs both the firmware's closed-loop permission (CLOSED_EN) and a reliable
    % observer; otherwise the drive keeps running open-loop rather than commutating on an
    % untrustworthy angle.
    transitionJustStarted = 0;
    if transitionStarted == 0 && closedLoop == 0 && alignmentComplete && handoffReady
        transitionStarted = 1;
        transitionJustStarted = 1;
        transitionElapsed = 0;
        handoffSpeedRpm = openSpeedRpm;
        handoffStartIq = openIq;
        observerIq = -ialpha*sin(obsTheta) + ibeta*cos(obsTheta);
        handoffEndIq = min(max(observerIq,-FINAL_A),FINAL_A);
    end

    if closedLoop ~= 0
        phase = 7; startupIq = handoffEndIq;
        forcedSpeedRpm = handoffSpeedRpm; transition = 1;
    elseif transitionStarted ~= 0
        if transitionJustStarted == 0
            transitionElapsed = transitionElapsed + TS;
        end
        if TRANS_S > 0
            transition = min(max(transitionElapsed/TRANS_S,0),1);
        else
            transition = 1;
        end
        startupIq = handoffStartIq + transition*(handoffEndIq - handoffStartIq);
        forcedSpeedRpm = handoffSpeedRpm;
        if transition >= 1
            closedLoop = 1; phase = 7; startupIq = handoffEndIq;
        else
            phase = 6;
        end
    else
        phase = openPhase; startupIq = openIq;
        forcedSpeedRpm = openSpeedRpm; transition = 0;
    end

    forcedAngleSt = mod(forcedAngleSt + forcedSpeedRpm*pi/30*PP*TS,2*pi);
    angleErr = wrapPiLocal(obsTheta - forcedAngleSt);
    % 关键门控 / The critical gate: transition 只有在接管状态机里才非零，而接管本身又
    % 需要 CLOSED_EN 且 reliable。因此观察角只在"允许闭环 + 观察器可信"时才会真正
    % 影响换相角；无条件混合会让转子丢转矩（见文件头）。
    % transition is non-zero only inside the handoff state machine, which itself requires
    % CLOSED_EN and reliable. The observer angle therefore reaches the commutation angle
    % only when the closed loop is permitted and the observer is trustworthy; blending it
    % unconditionally would make the rotor lose torque (see the file header).
    ctrlAngle = mod(forcedAngleSt + transition*angleErr,2*pi);
    observerControls = phase == 6 || phase == 7;

    % 两种失锁保护 / Two loss-of-lock protections:
    %   bit2：只允许闭环时停在状态 5 等待观察器，若 ACQUIRE_S [s]（0.5 s）内始终不可靠，
    %     说明观察器根本没收敛或参数不对，锁存故障而不是无限拖行；
    %   bit3：已经由观察角换相后，连续 LOSS_S [s]（50 ms）不可靠即锁存故障。
    %   两者都不直接操作硬件，仿真里只把 duty 归零矢量并把 state 置 8。
    % bit2 covers the closed-loop-allowed case stuck in state 5: if the observer is still
    % unreliable after ACQUIRE_S [s] it never converged, so the fault is latched instead of
    % retrying forever. bit3 covers loss of lock after the observer took over: LOSS_S [s]
    % of continuous unreliability latches the fault. Neither path touches hardware; in
    % simulation they only force the zero vector and state 8.
    if CLOSED_EN ~= 0 && phase == 5
        observerWait = observerWait + TS;
        if observerWait >= ACQUIRE_S
            faultLatched = 1;
            faultBits = double(bitor(uint32(faultBits),uint32(4)));
        end
    else
        observerWait = 0;
    end
    if observerControls
        if reliable ~= 0
            observerLoss = 0;
        else
            observerLoss = observerLoss + TS;
            if observerLoss >= LOSS_S
                faultLatched = 1;
                faultBits = double(bitor(uint32(faultBits),uint32(8)));
            end
        end
    else
        observerLoss = 0;
    end

    %% Closed-loop speed PI and Iq slew.
    % 速度环按 SPEED_DIV 分频运行（12 kHz/12 = 1 kHz）；速度量单位 [rad/s]，参考由
    % targetRpm [rpm] 经 *pi/30 转换，反馈是 obsOmega/PP（机械角速度）。
    % 闭环首拍 / First closed-loop tick: speedRef 被置为当前观察速度，所以初始速度误差为 0；
    % PRELOAD=0 时 desired=0，比例项和积分项都从 0 开始（与 MCSDK 参考工程
    % PID_SPEED_INTEGRAL_INIT_DIV=0 一致，不做无扰预装）。实际 Iq 由 appliedIq = startupIq
    % 起步，随后只按 IQ_SLEW [A/s]（32 A/s）逼近 PI 输出，因此不会出现单周期 Iq 阶跃——
    % 消阶跃的是限速器，不是预装；接管瞬间 Iq 会先按限速器向 PI 输出靠拢。
    % SPEED_RAMP [rpm/s] 限制参考斜率，IQ_SLEW [A/s] 限制实际 Iq 变化率，两者共同约束
    % 转矩冲击。
    % The speed loop runs every SPEED_DIV ticks (1 kHz). Speeds are in [rad/s]: the reference
    % comes from targetRpm [rpm] (*pi/30) and the feedback is obsOmega/PP (mechanical). On the
    % first closed-loop tick speedRef takes the measured speed so the initial error is zero,
    % and with PRELOAD=0 both the proportional and integral terms start at zero (matching
    % MCSDK PID_SPEED_INTEGRAL_INIT_DIV=0 with no bumpless preload). The applied Iq starts at
    % the handoff value and only approaches the PI output through the IQ_SLEW [A/s] rate limit,
    % so the single-tick step is removed by the slew limiter rather than by preload.
    % SPEED_RAMP [rpm/s] limits the reference slope and IQ_SLEW [A/s] the applied Iq slew.
    if phase == 7 && faultLatched == 0
        if speedInitialized == 0
            speedRef = obsSpeedRpm;
            desired = startupIq*PRELOAD;
            speedError = speedRef*pi/30 - obsOmega/PP;
            speedIqCommand = min(max(desired,SPEED_MIN),SPEED_MAX);
            speedInt = min(max(speedIqCommand - SPEED_KP*speedError,SPEED_MIN),SPEED_MAX);
            speedIqCommand = min(max(SPEED_KP*speedError + speedInt,SPEED_MIN),SPEED_MAX);
            appliedIq = startupIq;
            speedCounter = 0;
            speedInitialized = 1;
        elseif speedCounter == 0
            maxSpeedStep = SPEED_RAMP*TS*SPEED_DIV;
            speedRef = moveTowardsLocal(speedRef,targetRpm,maxSpeedStep);
            [speedIqCommand,speedInt] = piStepLocal( ...
                speedRef*pi/30,obsOmega/PP,SPEED_KP,SPEED_KI, ...
                TS*SPEED_DIV,speedInt,SPEED_MIN,SPEED_MAX);
        end
        speedCounter = mod(speedCounter + 1,SPEED_DIV);
        appliedIq = moveTowardsLocal(appliedIq,speedIqCommand,IQ_SLEW*TS);
        iqRef = appliedIq;
    else
        speedInitialized = 0; speedInt = 0; speedCounter = 0;
        speedRef = forcedSpeedRpm; speedIqCommand = startupIq;
        appliedIq = startupIq; iqRef = startupIq;
    end
    idRef = 0;
    % d 轴电流参考固定为 0：本电机是表贴式（Ld=Lq）且不做 MTPA/弱磁，Id=0 即最小铜耗
    % 换最大转矩；一旦引入凸极或弱磁需求，这里与下面的电压圆限幅都要同步修改。
    % The d-axis reference is fixed at zero: the machine is surface-mounted (Ld=Lq) with no
    % MTPA or field weakening, so Id=0 gives maximum torque per amp. Introducing saliency
    % or field weakening would require changing this and the circle limit together.

    %% Current PI, circle limit and SVPWM.
    % 电流环 / Current loop: idMeas/iqMeas [A] 由 ctrlAngle 的 Park 变换得到（与调制所用
    % 角度一致，避免角度不一致引入交叉耦合）；两个 PI 输出 vdCmd/vqCmd [V]，
    % 抗积分饱和由 piStepLocal 处理。
    % idMeas/iqMeas [A] come from the Park transform at ctrlAngle, the same angle used for
    % modulation, so no angle mismatch is introduced. Both PI outputs are in [V] and
    % anti-windup is handled inside piStepLocal.
    cosT = cos(ctrlAngle); sinT = sin(ctrlAngle);
    idMeas = ialpha*cosT + ibeta*sinT;
    iqMeas = -ialpha*sinT + ibeta*cosT;
    [vdCmd,idInt] = piStepLocal(idRef,idMeas,IKP,IKI,TS,idInt,IPI_MIN,IPI_MAX);
    [vqCmd,iqInt] = piStepLocal(iqRef,iqMeas,IKP,IKI,TS,iqInt,IPI_MIN,IPI_MAX);
    vCircle = VUTIL*vbus/sqrt(3);
    % 圆限幅半径 [V]：VUTIL*vbus/sqrt(3) 是 SVPWM 线性区的相电压峰值。除以 sqrt(3) 是
    % 因为 min-max 注入把可用线电压峰值提高到 vbus，对应相电压峰值 vbus/sqrt(3)；
    % 超圆的 (vd,vq) 按等比缩放，只改变幅值不改变方向，等价于保持电流环的相位。
    % Circle-limit radius in [V]: VUTIL*vbus/sqrt(3) is the phase-voltage peak of the SVPWM
    % linear range. The sqrt(3) appears because min-max injection raises the usable line
    % peak to vbus. An out-of-circle (vd,vq) is scaled isotropically, which preserves its
    % direction and therefore the current-loop phase.
    magnitude = sqrt(vdCmd*vdCmd + vqCmd*vqCmd);
    if magnitude > vCircle && magnitude > 0
        vdCmd = vdCmd*vCircle/magnitude;
        vqCmd = vqCmd*vCircle/magnitude;
    end
    vAlpha = vdCmd*cosT - vqCmd*sinT;
    vBeta = vdCmd*sinT + vqCmd*cosT;
    % Phase-domain dead-time feed-forward. The plant subtracts the same
    % polarity-dependent voltage, so the controller adds it before SVPWM.
    % A narrow linear band avoids a discontinuous sign flip at current zero.
    % 与 pmsm_plant.m 的死区损失严格同形（同样的 2*t_dead/Ts*vbus 与各相电流极性），
    % 因此理想情况下两者相消。带线性过零区是因为电流过零时真实损失连续变化，纯符号
    % 函数会在零点抖动；DT_FF_GAIN [-] 是保守缩放，实机首次试验建议从 0.75 或更低开始。
    % Same form as the plant's dead-time loss (same 2*t_dead/Ts*vbus and per-phase current
    % polarity), so in the ideal case the two cancel. The linear zero-crossing band exists
    % because the real loss varies continuously through zero current while a bare sign
    % function chatters. DT_FF_GAIN [-] is a conservative scale factor, and the first
    % hardware trial should start at 0.75 or lower.
    compA = DT_FF_EN*DT_FF_GAIN*deadVoltage*signA;
    compB = DT_FF_EN*DT_FF_GAIN*deadVoltage*signB;
    compC = DT_FF_EN*DT_FF_GAIN*deadVoltage*signC;
    vAlpha = vAlpha + (2*compA - compB - compC)/3;
    vBeta = vBeta + (compB - compC)/sqrt(3);
    totalMagnitude = sqrt(vAlpha*vAlpha + vBeta*vBeta);
    % 前馈之后必须再限幅一次：否则叠加的补偿电压会把指令推出线性区，反而产生失真。
    % The command must be limited again after feed-forward, otherwise the added
    % compensation pushes it out of the linear range and creates distortion instead.
    if totalMagnitude > vCircle && totalMagnitude > 0
        vAlpha = vAlpha*vCircle/totalMagnitude;
        vBeta = vBeta*vCircle/totalMagnitude;
    end
    [da,db,dc] = svpwmLocal(vAlpha,vBeta,vbus);
    % 占空比窗口 [0,1] 内再钳到 DUTYMIN/DUTYMAX（0.03..0.97）：给自举电容充电和高侧
    % 驱动留出最小脉宽，避免上管长时间常通导致自举掉电。
    % Within [0,1] the duty is further clamped to DUTYMIN..DUTYMAX (0.03..0.97) to leave
    % minimum pulse width for bootstrap charging and to keep the bootstrap capacitor from
    % discharging through a permanently on high side.
    da = min(max(da,DUTYMIN),DUTYMAX);
    db = min(max(db,DUTYMIN),DUTYMAX);
    dc = min(max(dc,DUTYMIN),DUTYMAX);
    if ~isfinite(da) || ~isfinite(db) || ~isfinite(dc)
        % 调制输出非数说明上游已经发散（例如除零或数值溢出），锁存 bit4 并停止出力。
        % A non-finite modulation output means the chain upstream has diverged (division by
        % zero or overflow); bit4 is latched and the drive stops producing output.
        faultLatched = 1;
        faultBits = double(bitor(uint32(faultBits),uint32(16)));
    end
    if faultLatched == 0
        % 只有本周期没有故障时才更新"上一周期占空比"：观察器用它重构端电压，故障后
        % 输出已是零矢量，继续用旧值会让观察器电压源与真实端电压不符。
        % The previous duty is updated only in a fault-free tick because the observer
        % reconstructs its terminal voltage from it; after a fault the applied duty is the
        % zero vector, so keeping the old value would desynchronise the two.
        prevDa = da; prevDb = db; prevDc = dc;
        duty = [da; db; dc]; state = phase; controlAngle = ctrlAngle;
    end
    forcedAngle = forcedAngleSt; obsAngle = obsTheta;
    speedReferenceRpm = speedRef; simTime = max(elapsed - TS,0);
end

if faultLatched ~= 0
    % 锁存故障后的最终输出：三相 0.5 即零矢量（相电压为零），state=8、reliable=0；
    % 固件侧对应关闭栅极并锁存故障标志，仿真侧只是停止出力，不接触任何硬件。
    % Final output after a latched fault: 0.5 on all phases is the zero vector (no phase
    % voltage) with state 8 and reliable 0. On the target this corresponds to disabling the
    % gates and latching the fault flag; in simulation it only stops producing output.
    duty = [0.5;0.5;0.5];
    prevDa = 0.5; prevDb = 0.5; prevDc = 0.5;
    state = 8; reliable = 0;
end
faultFlags = double(faultBits);
end

% ---------------------------------------------------------------------------
function y = satLocal(x)
% 饱和到 [-1,1]，用作滑模的连续近似，抑制抖振。
% Saturates to [-1,1] as a continuous approximation of the sign function to limit
% chattering.
y = min(max(x,-1),1);
end

function s = currentPolarityLocal(current,band)
% 带线性过零区的电流极性 [-]，输入电流 [A]、band [A]。
% band>0 时在 ±band 内线性过渡，band==0 时退化为纯符号函数；
% 死区补偿必须用它而不是 sign()，否则电流过零处补偿电压会跳变。
% Banded current polarity [-]: current in [A], band in [A]. A positive band gives a
% linear transition within ±band, while band==0 degenerates to a bare sign function.
% Dead-time compensation needs this instead of sign() or the compensation voltage jumps
% at current zero crossing.
if band > 0
    s = min(max(current/band,-1),1);
elseif current > 0
    s = 1;
elseif current < 0
    s = -1;
else
    s = 0;
end
end

function y = wrapPiLocal(x)
% 把角度 [rad] 折到 (-pi, pi]，用于最短角差（PLL 相位误差、接管角度差）。
% 直接相减两个 [0,2*pi) 的角度会得到跨越 ±2*pi 的错误误差，必须先用本函数归一化。
% Wraps an angle [rad] into (-pi, pi] to obtain the shortest angular difference (PLL
% phase error, handoff angle error). Subtracting two [0,2*pi) angles directly would give
% an error that wraps by ±2*pi, so it must be normalised first.
y = mod(x + pi,2*pi) - pi;
end

function y = moveTowardsLocal(value,target,maximumStep)
% 以 maximumStep 为上限向 target 逼近（值/步长同量纲，速度用 [rpm]、Iq 用 [A]）。
% 用"每周期最多变多少"而不是一阶滤波，是为了让限速值有明确的物理含义（rpm/s、A/s），
% 便于和固件配置逐项对照。
% Moves value towards target by at most maximumStep (same unit for all three: [rpm] for
% the speed ramp, [A] for the Iq slew). A hard rate limit instead of a first-order filter
% keeps the slew physically interpretable in rpm/s or A/s.
delta = target - value;
if delta > maximumStep
    y = value + maximumStep;
elseif delta < -maximumStep
    y = value - maximumStep;
else
    y = target;
end
end

function [y,integ] = piStepLocal(ref,meas,kp,ki,ts,integ,outMin,outMax)
% 一步离散 PI（带条件反算抗积分饱和），电流环与速度环共用。
% One discrete PI update with conditional back-calculation anti-windup, shared by the
% current and speed loops.
%
% 参数 / Parameters: ref/meas 同量纲（电流 [A] 或角速度 [rad/s]），kp [输出/输入]、
% ki [输出/(输入*s)]、ts [s]、integ 为积分器状态（与输出同量纲）、
% outMin/outMax 同时是输出限幅和积分器限幅。
% ref/meas share a unit (current [A] or angular speed [rad/s]); kp is [out/in] and ki is
% [out/(in*s)]; ts is in [s]; integ carries the output unit; outMin/outMax clamp both the
% output and the integrator.
%
% 抗饱和 / Anti-windup: 先把积分器钳进 outMin..outMax，再把 kp*err+integ 钳一次；
% 只有"和"被钳掉时才把积分器反算成 剩余限额（y - proportional），避免饱和期间积分
% 继续累积、退出饱和时产生超调。
% The integrator is clamped into outMin..outMax and the sum kp*err+integ is clamped again;
% only when the sum is actually clipped is the integrator back-calculated to the remaining
% headroom (y - proportional), so it cannot keep accumulating while saturated.
err = ref - meas;
proportional = kp*err;
integ = min(max(integ + ki*ts*err,outMin),outMax);
unclamped = proportional + integ;
y = min(max(unclamped,outMin),outMax);
if unclamped ~= y
    integ = min(max(y - proportional,outMin),outMax);
end
end

function [da,db,dc] = svpwmLocal(alpha,beta,vbus)
% 最小-最大（零序注入）SVPWM：alpha/beta [V]，vbus [V]，输出三相占空比 [0,1]。
% Min-max (zero-sequence injection) SVPWM: alpha/beta in [V], vbus in [V], three-phase
% duty in [0,1].
%
% 与 +foc/svpwm.m、rust/crates/foc-algorithm/src/modulation.rs::svpwm_update 同形，
% 区别是这里把 vbus 下限钳到 1e-6 而不是提前返回 0.5：MATLAB Function 块里必须给出
% 确定的输出宽度，且这里已经在 fault 逻辑里处理了母线超窗，所以不需要第二道保护。
% Same form as +foc/svpwm.m and modulation.rs::svpwm_update, except that vbus is floored at
% 1e-6 instead of returning 0.5 early: the MATLAB Function block needs a definite output
% width, and the bus window is already handled by the fault logic.
vbusSafe = max(vbus,1e-6);
va = alpha;
vb = -0.5*alpha + 0.5*sqrt(3)*beta;
vc = -0.5*alpha - 0.5*sqrt(3)*beta;
offset = 0.5*(max([va vb vc]) + min([va vb vc]));
da = min(max(0.5 + (va - offset)/vbusSafe,0),1);
db = min(max(0.5 + (vb - offset)/vbusSafe,0),1);
dc = min(max(0.5 + (vc - offset)/vbusSafe,0),1);
end
