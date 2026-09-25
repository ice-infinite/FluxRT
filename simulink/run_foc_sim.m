function out = run_foc_sim(varargin)
%RUN_FOC_SIM Run the parameterized Simulink FOC model without rewriting code.
%
% out = run_foc_sim()                       % current firmware, open loop
% out = run_foc_sim('closedLoop',true,'duration',10)
% out = run_foc_sim('smoSlide',4.5,'emfAlpha',0.06)
%
% FluxRT —— 单次仿真的统一入口（参数 → 过期检查 → 重建/加载 → 仿真 → 结果结构体）。
% FluxRT - the single-run simulation entry point: parameters, staleness check,
% rebuild/load, simulate, return a result struct.
%
% 职责 / Responsibility:
%   - 用 init_foc_params 组装参数，把观察器整定覆盖写到命名字段上，再由
%     foc_refresh_vectors 重新打包成向量；因此**本函数不改写任何 .m 源文件**，
%     controller_core.m / pmsm_plant.m 始终只作为 MATLAB Function 块的源文本被读取；
%   - 需要时重建模型（见下面的过期检查），把 p 和 foc_iabc 放进 base 工作区供
%     Constant 块解析，然后 sim()，最后把 To Workspace 记录整理成名值结构体。
%   - Assembles parameters, applies observer tuning overrides to the named fields and
%     re-packs them into vectors; it therefore never rewrites a .m source file. It rebuilds
%     the model when needed, publishes p and foc_iabc into the base workspace for the
%     Constant blocks, runs sim() and reshapes the To Workspace logs into a struct.
%
% 重建必要性 / Why a rebuild matters here:
%   .slx 里保存的是 controller_core.m / pmsm_plant.m 的文本快照。改过这两个文件却直接
%   仿真，模型仍然跑旧副本，而且不会报错。下面的过期检查比较这三个源文件与 .slx 的
%   修改时间正是为了兜住这一点：本目录曾有一整轮 216 点扫描因为模型过期而对每个点给出
%   完全相同的结果，看似"参数不敏感"，其实参数根本没进模型。
%   The .slx stores a text snapshot of the two chart sources, so simulating after editing
%   them silently runs the old copy. The staleness check below compares the modification
%   times of those three sources against the .slx for exactly that reason: a full 216-point
%   sweep once returned identical results for every point because the model was stale, which
%   looked like parameter insensitivity when in fact the parameters never reached the model.
%
% 边界 / Boundary: 纯 PC 仿真，不打开串口、不使能功率级；观察器整定是否可用仍需实机
% 独立真值验证（实机没有编码器/测速仪真值）。
% Pure PC simulation: no serial port, no power stage. Whether a tuning is usable still needs
% independent hardware truth, which the rig currently does not have.
%
% 参考 / Reference: simulink/Simulink仿真工程说明.md, docs/仿真实机相关性验证.md

ip = inputParser;
ip.addParameter('observerProfile','firmware',@(s)ischar(s)||isstring(s));
ip.addParameter('closedLoop',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('modelName','foc_bringup',@(s)ischar(s)||isstring(s));
ip.addParameter('targetRpm',582,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('busVoltage',12.3,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('duration',5.0,@(x)isnumeric(x)&&isscalar(x)&&x>0);
ip.addParameter('pwmFrequencyHz',12000,@(x)isnumeric(x)&&isscalar(x)&&x>0&&x==fix(x));
ip.addParameter('controlFrequencyHz',12000,@(x)isnumeric(x)&&isscalar(x)&&x>0&&x==fix(x));
ip.addParameter('actuationDelayPwmTicks',1, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x==fix(x));
ip.addParameter('deadTimeNs',550,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('loadTorqueNm',0,@(x)isnumeric(x)&&isscalar(x));
ip.addParameter('enableDeadTime',true,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('enableObserverDeadTimeCompensation',false,@(x)islogical(x)||isnumeric(x));
ip.addParameter('deadTimeCompensationGain',1.0,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('deadTimeCompensationCurrentBandA',0.005, ...
    @(x)isnumeric(x)&&isscalar(x)&&x>=0);
ip.addParameter('smoSlide',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('smoBoundary',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('emfAlpha',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('pllKp',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('pllKi',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('minimumSpeedRpm',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('minimumBemfV',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('varianceRatio',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('consecutiveSamples',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)));
ip.addParameter('rebuild',false,@(x)islogical(x)||isnumeric(x));
ip.parse(varargin{:});
o = ip.Results;

here = fileparts(mfilename('fullpath'));
addpath(here);
p = init_foc_params( ...
    'observerProfile',o.observerProfile,'closedLoop',o.closedLoop, ...
    'targetRpm',o.targetRpm,'busVoltage',o.busVoltage, ...
    'duration',o.duration,'deadTimeNs',o.deadTimeNs, ...
    'pwmFrequencyHz',o.pwmFrequencyHz, ...
    'controlFrequencyHz',o.controlFrequencyHz, ...
    'actuationDelayPwmTicks',o.actuationDelayPwmTicks, ...
    'loadTorqueNm',o.loadTorqueNm,'enableDeadTime',o.enableDeadTime, ...
    'enableDeadTimeCompensation',o.enableDeadTimeCompensation, ...
    'enableObserverDeadTimeCompensation',o.enableObserverDeadTimeCompensation, ...
    'deadTimeCompensationGain',o.deadTimeCompensationGain, ...
    'deadTimeCompensationCurrentBandA',o.deadTimeCompensationCurrentBandA);
if ~isempty(o.smoSlide), p.observer.kSlideV = o.smoSlide; end
% 观察器整定覆盖 / Observer tuning overrides：这里只改命名字段，所有量纲与参数文件一致
% （kSlideV [V]、boundaryA [A]、emfAlpha [-]、pllKp/pllKi、可靠性门限 [rpm]/[V]/[-]）。
% 覆盖之后必须重新打包成向量，模型才看得到——这是扫描能在不改源码的情况下生效的原因。
% These only touch named fields with the same units as the parameter file (kSlideV [V],
% boundaryA [A], emfAlpha [-], pllKp/pllKi, reliability gates in [rpm]/[V]/[-]). The
% vectors must be re-packed afterwards for the model to see them, which is why a sweep can
% vary tuning without touching any source file.
if ~isempty(o.smoBoundary), p.observer.boundaryA = o.smoBoundary; end
if ~isempty(o.emfAlpha), p.observer.emfAlpha = o.emfAlpha; end
if ~isempty(o.pllKp), p.observer.pllKp = o.pllKp; end
if ~isempty(o.pllKi), p.observer.pllKi = o.pllKi; end
if ~isempty(o.minimumSpeedRpm), p.observer.reliabilityMinMeanRpm = o.minimumSpeedRpm; end
if ~isempty(o.minimumBemfV), p.observer.reliabilityMinEmfV = o.minimumBemfV; end
if ~isempty(o.varianceRatio), p.observer.reliabilityMaxVarRatio = o.varianceRatio; end
if ~isempty(o.consecutiveSamples), p.observer.reliabilityWindows = o.consecutiveSamples; end
p = foc_refresh_vectors(p);
% 模型里的 Constant 块在工作区解析 p 与 foc_iabc，因此必须放进 base 而不是 caller。
% foc_iabc 是控制器第一拍看到的初始电流向量 [A]，默认零相电流。
% The model's Constant blocks resolve p and foc_iabc from the workspace, so they go into
% base rather than the caller. foc_iabc is the initial current vector [A] seen on the first
% controller tick.
assignin('base','p',p);
assignin('base','foc_iabc',[0;0;0]);

mdl = char(o.modelName);
slxPath = fullfile(here,[mdl '.slx']);
% 过期检查 / Staleness check: 显式要求重建、或 .slx 不存在、或三个源文件中有任何一个比
% .slx 新，就重建。覆盖范围只有这三个文件——参数改动不需要重建（走向量进入模型），
% +foc/ 包也不参与（模型不使用它）。这是"改完代码忘了重建"的唯一自动防线。
% Rebuild when explicitly requested, when the .slx is missing, or when any of the three
% source files is newer than the .slx. Parameter edits do not need a rebuild (they enter
% through the vectors) and the +foc/ package is not involved because the model never calls
% it. This is the only automatic guard against "edited the code, forgot to rebuild".
needBuild = logical(o.rebuild) || ~exist(slxPath,'file');
if ~needBuild
    sources = {'controller_core.m','pmsm_plant.m','make_foc_model.m'};
    modelStamp = dir(slxPath).datenum;
    for k = 1:numel(sources)
        if dir(fullfile(here,sources{k})).datenum > modelStamp
            needBuild = true;
        end
    end
end
if needBuild
    make_foc_model(mdl);
end
if ~bdIsLoaded(mdl)
    load_system(slxPath);
end

% The saved model initializes itself for direct GUI use. Scripted runs already
% supplied p, so disable the callback only in memory for this simulation.
% 已保存的模型自带 InitFcn（p = foc_workspace_init()）以便 GUI 直接运行；脚本调用已经
% 用覆盖后的 p 赋值过，若让 InitFcn 再跑一次就会把观察器整定覆盖回去。因此只在内存里
% 清空 InitFcn，并用 onCleanup 在退出（含异常）时还原，不修改磁盘上的模型。
% StopTime 取 duration-Ts：最后一次采样落在 duration-Ts 上，使 simTime 的末值与
% p.timing.duration 对应的窗长一致，避免多出一拍。
% The saved model carries an InitFcn for direct GUI use; a scripted run has already
% published its overridden p, and letting InitFcn run again would silently revert the
% tuning. It is cleared in memory only and restored by onCleanup on every exit path,
% including errors, without touching the file on disk. StopTime is duration-Ts so the last
% sample lands on duration-Ts and the logged window matches p.timing.duration exactly.
savedInit = get_param(mdl,'InitFcn');
savedStop = get_param(mdl,'StopTime');
cleanup = onCleanup(@() restore_model(mdl,savedInit,savedStop));
set_param(mdl,'InitFcn','');
set_param(mdl,'StopTime',sprintf('%.17g',max(p.timing.duration-p.timing.pwmTs,0)));
simOut = sim(mdl);
clear cleanup

out = struct();
out.mdl = mdl; out.p = p;
% 结果字段与其量纲 / Result fields and units。取数一律按 To Workspace 的变量名，
% 因此日志块改名会在这里表现为取数失败（快速失败好过静默取错列）。
% iqMeas/idMeas 是控制器在 ctrlAngle 下算出的 dq 电流 [A]；
% vdCmd/vqCmd 是电流 PI 输出 [V]；idPlant/iqPlant 是 plant 内部真值 [A]；
% angleErrorRad 是观察角减 plant 真值并折到 (-pi,pi] 的差 [rad]；
% speedErrorRpm 是观察转速减 plant 真值 [rpm]。注意实机没有这些真值，两个误差量只在
% 仿真里存在，不能拿来宣称实机精度。
% All fields are fetched by To Workspace variable name, so renaming a logging block fails
% loudly here instead of silently picking the wrong column. iqMeas/idMeas are dq currents
% [A] in the controller frame, vdCmd/vqCmd are PI voltages [V], idPlant/iqPlant are plant
% truths [A], angleErrorRad is the wrapped angle difference [rad] and speedErrorRpm the
% speed difference [rpm]. Those two error signals exist only in simulation: the rig has no
% independent speed or angle truth.
getv = @(n)simOut.get(n).signals.values;
out.time = getv('simTime');
out.trueSpeedRpm = aligned_values(simOut,'trueSpeedRpm',out.time);
out.obsSpeedRpm = getv('obsSpeedRpm');
out.trueAngle = aligned_values(simOut,'trueAngle',out.time);
out.obsAngle = getv('obsAngle');
out.forcedAngle = getv('forcedAngle');
out.controlAngle = getv('controlAngle');
out.speedReferenceRpm = getv('speedReferenceRpm');
out.state = getv('state');
out.reliable = getv('reliable');
out.faultFlags = getv('faultFlags');
out.idRef = getv('idRef'); out.iqRef = getv('iqRef');
out.idMeas = getv('idMeas'); out.iqMeas = getv('iqMeas');
out.vdCmd = getv('vdCmd'); out.vqCmd = getv('vqCmd');
out.torqueE = aligned_values(simOut,'torqueE',out.time);
out.idPlant = aligned_values(simOut,'idPlant',out.time);
out.iqPlant = aligned_values(simOut,'iqPlant',out.time);
out.phaseCurrents = getv('phaseCurrents');
out.angleErrorRad = mod(out.obsAngle-out.trueAngle+pi,2*pi)-pi;
out.speedErrorRpm = out.obsSpeedRpm-out.trueSpeedRpm;
end

function values = aligned_values(simOut,name,referenceTime)
% 把 PWM 速率的 plant trace 以零阶保持对齐到控制拍；单速率时原样返回。
signal = simOut.get(name);
values = signal.signals.values;
sourceTime = signal.time;
if numel(sourceTime) == numel(referenceTime) && ...
        all(abs(sourceTime(:)-referenceTime(:)) < 1e-12)
    return
end
indices = interp1(sourceTime(:),(1:numel(sourceTime))',referenceTime(:), ...
    'previous','extrap');
indices = max(1,min(numel(sourceTime),round(indices)));
values = values(indices,:);
end

function restore_model(mdl,initFcn,stopTime)
% 把模型的内存配置还原成磁盘上的样子；onCleanup 会保证异常路径也执行到。
% Restores the in-memory model configuration; onCleanup guarantees this runs on error paths
% as well.
if bdIsLoaded(mdl)
    set_param(mdl,'InitFcn',initFcn,'StopTime',stopTime);
end
end
