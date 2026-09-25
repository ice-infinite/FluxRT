function out = run_foc_validation(varargin)
%RUN_FOC_VALIDATION Exercise current firmware-equivalent open/closed paths.
%
% FluxRT —— 模型级回归：参数漂移断言 + 开环 + 观察器接管后闭环。
% FluxRT - model-level regression: parameter-drift asserts, then an open-loop run, then the
% observer handoff and closed-loop run.
%
% 职责 / Responsibility:
%   - 先断言参数文件与固件默认值一致（增益、观察器四值、可靠性窗口、向量长度、两级
%     死区补偿默认关闭）。参数文件是本目录的唯一来源，一旦漂移必须立刻失败而不是
%     继续跑出一条"看起来正常"的曲线；
%   - 再用 enableDeadTime=false 做开环回归，断言末态停在状态 5 且无故障；最后跑闭环，
%     断言经过状态 6 到达状态 7 且无故障。
%   - First asserts that the parameter file still matches the firmware defaults (gains,
%     observer values, reliability window, vector length, both dead-time compensations off),
%     then runs the open loop with dead time disabled and requires state 5 with no fault, and
%     finally the closed loop and requires state 6 followed by state 7 with no fault.
%
% 为什么关掉死区 / Why dead time is disabled:
%   enableDeadTime=false 对应 rust/crates/foc-sim 的理想平均逆变器，用于逐拍对拍；
%   带死区的模型是另一条曲线，两者不能混在同一张"精确一致"表里。
%   enableDeadTime=false corresponds to the ideal average-value inverter of
%   rust/crates/foc-sim for tick-by-tick comparison; the dead-time model is a different
%   curve and the two must not be mixed in one "exact match" table.
%
% 边界 / Boundary（末尾也会打印）: 这只是模型/主机级验证，不是新的实机证明；本函数不
% 访问串口、不使能功率级，末尾关闭模型释放内存。
% This is model/host-level validation only, not new hardware proof. It never touches the
% serial port or the power stage, and it closes the model at the end.
%
% 参考 / Reference: simulink/Simulink仿真工程说明.md

ip = inputParser;
ip.addParameter('openDuration',5,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('closedDuration',10,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('rebuild',false,@(x)islogical(x)||isnumeric(x));
ip.parse(varargin{:});
o = ip.Results;

fprintf('=== current firmware parameter check ===\n');
p = init_foc_params();
% 每一条断言对应的量与量纲 / What each assert pins down:
%   idKp [V/A] 7.197815、idKi [V/(A*s)] 35989.077 —— MCSDK 整数增益换算出的电流环增益；
%   kSlideV [V] 4.0、boundaryA [A] 0.16 —— 当前固件的 SMO 默认值；
%   pllKp [-] 80、pllKi [1/s] 1000 —— PLL 默认值；
%   reliabilityWindows [-] 2、reliabilityMaxVarRatio [-] 0.01 —— 可靠性门；
%   numel(controllerVector)==49 —— 与 controller_core.m 的 cfg 解包长度对齐的协议断言；
%   两级死区补偿默认关闭、过零线性带 0.005 A —— 补偿默认不参与仿真。
% idKp [V/A] and idKi [V/(A*s)] are the converted current-loop gains, kSlideV [V] and
% boundaryA [A] the current SMO defaults, pllKp/pllKi the PLL defaults, the reliability
% gates are dimensionless/ratio, the vector length is the protocol assert, and both
% dead-time compensation layers are off by default with a 0.005 A zero-crossing band.
checks = [ ...
    abs(p.control.idKp-7.197815315)<1e-6; ...
    abs(p.control.idKi-35989.076576)<1e-3; ...
    abs(p.observer.kSlideV-4.0)<1e-9; ...
    abs(p.observer.boundaryA-0.16)<1e-9; ...
    abs(p.observer.pllKp-80)<1e-9; ...
    abs(p.observer.pllKi-1000)<1e-9; ...
    p.observer.reliabilityWindows==2; ...
    abs(p.observer.reliabilityMaxVarRatio-0.01)<1e-12; ...
    numel(p.controllerVector)==49; ...
    ~p.inverter.deadTimeCompensationEnabled; ...
    ~p.inverter.observerDeadTimeCompensationEnabled; ...
    abs(p.inverter.deadTimeCompensationCurrentBandA-0.005)<1e-12];
assert(all(checks),'MATLAB parameters drifted from the current Rust defaults.');
fprintf('parameters: PASS\n');

fprintf('\n=== open-loop telemetry run ===\n');
% 开环回归 / Open-loop regression: 明确要求末态是 5（OpenLoopHold）且没有任何故障位。
% 这是"控制器不会自己跑飞"的最小保证：启动状态机走完 alignment→ramp→hold 后停住等待，
% 而不是进入 6/7（闭环需 CLOSED_EN）或 8（故障）。
% The open-loop regression requires state 5 (OpenLoopHold) with no fault bits. It is the
% minimal "the controller does not run away" guarantee: the state machine completes
% alignment, ramp and hold and then waits, rather than entering 6/7 or latching a fault.
openRun = run_foc_sim('closedLoop',false,'duration',o.openDuration, ...
    'enableDeadTime',false,'rebuild',o.rebuild);
fprintf('final true/observer : %.3f / %.3f rpm\n', ...
    openRun.trueSpeedRpm(end),openRun.obsSpeedRpm(end));
fprintf('final state/faults  : %.0f / 0x%X\n', ...
    openRun.state(end),uint32(max(openRun.faultFlags)));
fprintf('reliable samples    : %d / %d\n', ...
    sum(openRun.reliable~=0),numel(openRun.reliable));
assert(openRun.state(end)==5 && max(openRun.faultFlags)==0, ...
    'Open-loop regression did not end safely in OpenLoopHold.');

fprintf('\n=== observer handoff and closed-loop run ===\n');
% 闭环回归 / Closed-loop regression: 必须出现状态 6（渐变接管）和状态 7（闭环），并且
% 末态是 7 且无故障。rebuild=false 是有意的：模型在这一步之前刚由开环运行保证是最新的。
% Must reach state 6 (blended handoff) and state 7 (closed loop) and finish in 7 with no
% fault. rebuild=false is intentional: the open-loop run immediately above already ensured
% the model is current.
if bdIsLoaded('foc_bringup'), close_system('foc_bringup',0); end
closedRun = run_foc_sim('closedLoop',true,'duration',o.closedDuration, ...
    'enableDeadTime',false,'rebuild',false);
i6 = find(closedRun.state==6,1);
i7 = find(closedRun.state==7,1);
assert(~isempty(i6)&&~isempty(i7),'Closed-loop run never reached transition/state 7.');
assert(closedRun.state(end)==7 && max(closedRun.faultFlags)==0, ...
    'Closed-loop run ended outside state 7 or latched a fault.');
fprintf('transition/closed   : %.6f / %.6f s\n', ...
    closedRun.time(i6),closedRun.time(i7));
fprintf('final true/observer : %.3f / %.3f rpm\n', ...
    closedRun.trueSpeedRpm(end),closedRun.obsSpeedRpm(end));
fprintf('final Iq reference  : %.6f A\n',closedRun.iqRef(end));
fprintf('max raw Iq step     : %.6f A\n',max(abs(diff(closedRun.iqRef))));

out = struct('parameters',p,'open',openRun,'closed',closedRun);
% 这句总结是硬边界：通过只说明模型和 Rust 主机实现自洽，不构成实机安全或精度证明。
% This closing line is a hard boundary: passing only means the model is self-consistent with
% the Rust host implementation, which is not hardware safety or accuracy proof.
fprintf('\nVALIDATION PASS (model/host level only; not new hardware proof).\n');
if bdIsLoaded('foc_bringup'), close_system('foc_bringup',0); end
end
