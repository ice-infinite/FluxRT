function [iabc, trueSpeedRpm, trueAngle, torqueE, idPlant, iqPlant] = ...
         pmsm_plant(duty, vbus, loadTorque, cfg)
%PMSM_PLANT Discrete PMSM plant matching rust/crates/foc-sim.
% An optional averaged dead-time voltage loss is retained for hardware
% correlation. Disable it for a strict comparison with the Rust host plant.
%
% FluxRT —— Simulink 侧被控对象（离散 dq PMSM + 机械负载）。
% FluxRT - the Simulink-side plant: discrete dq PMSM plus mechanical load.
%
% 职责 / Responsibility:
%   - 三相占空比 → 相电压 → dq 电压 → 定子电流状态方程 → 电磁转矩 → 机械方程 →
%     电角度 → 三相电流，全部用正向欧拉按 TS 单步推进，对应 rust/crates/foc-sim；
%   - 可选"平均死区电压损失"用于和实机趋势对拍；做严格逐拍对拍时必须关闭。
%   - Phase duty -> phase voltage -> dq voltage -> stator current state equations ->
%     electromagnetic torque -> mechanical equations -> electrical angle -> phase
%     currents, all with forward Euler at step TS, mirroring rust/crates/foc-sim. The
%     optional averaged dead-time loss is for hardware correlation; disable it when
%     comparing tick-by-tick against the Rust host plant.
%
% 单位 / Units: duty [0,1]；vbus [V]；loadTorque [N*m]；iabc [A]；trueSpeedRpm [rpm]；
% trueAngle [rad]（电角）；torqueE [N*m]；idPlant/iqPlant [A]；cfg 见下方解包注释。
% 角度约定与 controller_core.m 一致（Park 变换用 cos/sin(elecAngle)）。
% duty in [0,1], vbus in [V], loadTorque in [N*m], currents in [A], trueSpeedRpm in [rpm],
% trueAngle in [rad] electrical, torqueE in [N*m]. The angle convention matches
% controller_core.m (Park transform with cos/sin(elecAngle)).
%
% 模型边界 / Model limits（不要外推）/ do not extrapolate:
%   平均逆变器（无开关纹波）、无 ADC 量化与噪声、无母线动态、无温升、无齿槽和结构
%   声学；因此它只能验证控制流程和低频趋势，不能预测稳定性裕量或可听噪声。
%   Average-value inverter with no switching ripple, ADC quantisation or noise, no bus
%   dynamics, no thermal model, no cogging or structural acoustics: valid for control-flow
%   and low-frequency trends only, not for stability margins or audible noise.
%
% 未辨识参数 / Unidentified parameters: cfg 里的 Rs/Ld/Lq/磁链/转动惯量/摩擦都继承 ST
% Workbench 数据库，尚未在实物电机上辨识；惯量与摩擦是残差误差的主要来源之一。
% Rs, Ld, Lq, flux linkage, inertia and friction in cfg are inherited from the ST Workbench
% database and are not identified on the real motor; inertia and friction are a known
% source of residual model error.
%
% 参考 / Reference: rust/crates/foc-sim/src/lib.rs, simulink/Simulink仿真工程说明.md

%%#codegen
persistent init id iq mechSpeed mechAngle prevIa prevIb prevIc
if isempty(init)
    init = 1;
    id = 0; iq = 0; mechSpeed = 0; mechAngle = 0;
    prevIa = 0; prevIb = 0; prevIc = 0;
end

RS = cfg(1); LD = cfg(2); LQ = cfg(3); FLUX = cfg(4); PP = cfg(5);
J = cfg(6); B = cfg(7); TS = cfg(8);
DEAD_ENABLE = cfg(9); DEADTIME_NS = cfg(10);
% cfg 编号与 foc_refresh_vectors.m 的 plantVector 打包顺序一一对应（顺序即协议）。
% 单位 / Units: RS[ohm] LD/LQ[H] FLUX[Wb] PP[-] J[kg*m^2] B[N*m*s] TS[s]
% DEAD_ENABLE[-] DEADTIME_NS[ns]。
% The cfg numbering matches the plantVector packing in foc_refresh_vectors.m; the order is
% the protocol. Units: RS [ohm], LD/LQ [H], FLUX [Wb], PP [-], J [kg*m^2], B [N*m*s],
% TS [s], DEAD_ENABLE [-], DEADTIME_NS [ns].

da = duty(1); db = duty(2); dc = duty(3);
common = (da + db + dc)/3;
va = (da - common)*vbus;
vb = (db - common)*vbus;
vc = (dc - common)*vbus;
% 死区损失与 controller_core.m 的观察器补偿同形（2*t_dead/Ts*Vbus [V]），但极性用纯
% 符号函数 prevIa 而不是带线性带：真实器件在电流过零处确实会跳变，控制器那侧的线性带
% 是"故意保守"的近似，所以分层消融时两边不会完全抵消。
% The dead-time loss has the same form as the observer compensation in controller_core.m
% (2*t_dead/Ts*Vbus [V]) but derives its polarity from a bare sign of the previous phase
% current: the real device does switch abruptly near current zero, while the controller's
% linear band is a deliberately conservative approximation, so the two do not cancel
% exactly during ablation.
deadLoss = DEAD_ENABLE*(2*DEADTIME_NS*1e-9/TS)*vbus;
va = va - deadLoss*sgnLocal(prevIa);
vb = vb - deadLoss*sgnLocal(prevIb);
vc = vc - deadLoss*sgnLocal(prevIc);

valpha = (2*va - vb - vc)/3;
vbeta = (vb - vc)/sqrt(3);
elecAngle = mod(PP*mechAngle,2*pi);
cosT = cos(elecAngle); sinT = sin(elecAngle);
vd = valpha*cosT + vbeta*sinT;
vq = -valpha*sinT + vbeta*cosT;
elecSpeed = PP*mechSpeed;
% dq 状态方程 / dq state equations: 含交叉耦合项（dq 系下的旋转反电势）和永磁磁链项。
% 单位 [V]/[H]/[ohm]/[A]/[rad/s]/[Wb]；因为 Ld=Lq，磁阻转矩项为零，但保留通用形式。
% Includes the cross-coupling (motional EMF in the dq frame) and the permanent-magnet flux
% term, in [V]/[H]/[ohm]/[A]/[rad/s]/[Wb]. Ld=Lq here so the reluctance term vanishes, but
% the general form is kept.
did = (vd - RS*id + elecSpeed*LQ*iq)/LD;
diq = (vq - RS*iq - elecSpeed*(LD*id + FLUX))/LQ;
id = id + TS*did;
iq = iq + TS*diq;

% 转矩 [N*m]：1.5*PP*(flux*iq + (Ld-Lq)*id*iq)。系数 1.5 来自幅值不变 Clarke/Park
% 约定（三相合成转矩 = 3/2*p*psi*iq）；若改成幅值不变以外的约定，控制器和 plant 必须
% 同时改，否则电流环增益整定会整体偏离。
% Torque in [N*m]: 1.5*PP*(flux*iq + (Ld-Lq)*id*iq). The 1.5 factor belongs to the
% amplitude-invariant Clarke/Park convention (three-phase torque = 3/2*p*psi*iq); changing
% the convention requires changing both plant and controller or every gain tuning shifts.
torqueE = 1.5*PP*(FLUX*iq + (LD - LQ)*id*iq);
% 机械方程 / Mechanical equation: J*dw/dt = Te - Tload - B*w，w 为机械角速度 [rad/s]，
% 正向欧拉离散。loadTorque [N*m] 由 Simulink 的 Tload 常量块给出（默认 0 = 空载）。
% J*dw/dt = Te - Tload - B*w with mechanical speed in [rad/s], discretised by forward
% Euler; loadTorque in [N*m] comes from the Tload constant block (default 0, no load).
acceleration = (torqueE - loadTorque - B*mechSpeed)/J;
mechSpeed = mechSpeed + TS*acceleration;
mechAngle = mod(mechAngle + TS*mechSpeed,2*pi);
elecAngle = mod(PP*mechAngle,2*pi);

% 反 Park 变换得到三相电流 [A]：ia = id*cos - iq*sin，b/c 相分别移 ±120°。
% 这正是 SMO 反电势角度约定 atan2(-e_alpha, e_beta) 的来源（见 controller_core.m）。
% Inverse Park transform to phase currents in [A]: ia = id*cos - iq*sin with the b and c
% phases shifted by ±120 degrees. This is exactly where the SMO angle convention
% atan2(-e_alpha, e_beta) in controller_core.m comes from.
ia = id*cos(elecAngle) - iq*sin(elecAngle);
ib = id*cos(elecAngle - 2*pi/3) - iq*sin(elecAngle - 2*pi/3);
ic = id*cos(elecAngle + 2*pi/3) - iq*sin(elecAngle + 2*pi/3);
iabc = [ia;ib;ic];
prevIa = ia; prevIb = ib; prevIc = ic;
trueSpeedRpm = mechSpeed*30/pi;
trueAngle = elecAngle;
idPlant = id;
iqPlant = iq;
end

function s = sgnLocal(x)
% 纯符号函数 [-]，用于死区损失的电流极性：与控制器侧的带线性带版本刻意不同（见上）。
% Bare sign function [-] for the dead-time current polarity; deliberately different from
% the controller's banded version (see above).
if x > 0
    s = 1;
elseif x < 0
    s = -1;
else
    s = 0;
end
end
