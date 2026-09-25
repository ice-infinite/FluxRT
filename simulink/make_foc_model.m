function mdl = make_foc_model(modelName, varargin)
%MAKE_FOC_MODEL  Build (or rebuild) the Simulink FOC bring-up model from code.
%
%   mdl = make_foc_model()                     % name 'foc_bringup'
%   mdl = make_foc_model('foc_bringup_fw')
%
%   The model is generated programmatically so the diagram cannot drift away
%   from the parameter set. Structure:
%
%     Vbus ---------> [ControllerCore] --duty--> [PmsmPlant] --iabc--+
%     TargetRpm ---->        |                      |               |
%                            |                      +-- trueAngle   |
%                            +-- telemetry          +-- trueSpeed   |
%                                                   |               |
%                            <--------------------------------------+
%                                (current feedback closes the loop)
%
%   ControllerCore and PmsmPlant are MATLAB Function blocks sourced from
%   controller_core.m and pmsm_plant.m, so the control law and the machine
%   equations stay reviewable as plain code while the topology is a normal
%   Simulink diagram. Every port width is declared explicitly, so a rebuild is
%   deterministic and independent of workspace state.
%
%   Requires: Simulink. No Simscape.
%
% FluxRT —— 用代码生成 / 重建 Simulink bring-up 模型。
% FluxRT - builds (or rebuilds) the Simulink bring-up model from code.
%
% 职责 / Responsibility:
%   - 用 add_block/add_line 搭出固定的拓扑，把 controller_core.m 和 pmsm_plant.m 的
%     正文注入两个 MATLAB Function 块，声明每个端口的宽度和类型，再存成 .slx；
%   - 模型结构：ControllerCore（控制器：SMO+PLL → 升速接管 → 速度/电流 PI → 圆限幅
%     → 死区前馈 → SVPWM）与 PmsmPlant（被控对象：dq PMSM + 机械负载 + 可选死区损失），
%     中间以三相占空比相连，反馈经 Unit Delay 回到控制器。
%   - Builds the topology with add_block/add_line, injects the bodies of controller_core.m
%     and pmsm_plant.m into two MATLAB Function blocks, declares every port width and type,
%     and saves the result as .slx. ControllerCore holds the control chain and PmsmPlant the
%     machine; they are connected by the three-phase duty and the feedback returns through a
%     Unit Delay.
%
% 为什么程序化生成 / Why generate the diagram instead of drawing it:
%   手画的框图会和参数集、和两个 .m 源文件逐渐漂移；这里每次生成都显式声明端口宽度，
%   所以重建结果是确定的，不依赖工作区里残留的变量。
%   A hand-drawn diagram drifts away from the parameter set and the two .m sources. Every
%   port width is declared explicitly here, so a rebuild is deterministic and independent of
%   leftover workspace state.
%
% 重建必要性 / Why a rebuild is required:
%   本函数把 controller_core.m / pmsm_plant.m 的**文本快照**写进 .slx（chart.Script）。
%   改完这两个文件后若不重建，模型里跑的仍是旧副本，而且不会有任何报错——run_foc_sim.m
%   用文件时间戳做过期检查正是为了避免这种情况（本目录曾出现过整轮 216 点扫描因为
%   模型过期而结果全部相同的定标事故）。
%   This function writes a TEXT SNAPSHOT of controller_core.m / pmsm_plant.m into the .slx
%   (chart.Script). After editing either file, simulating without a rebuild silently runs
%   the old copy. The staleness check in run_foc_sim.m exists for exactly that reason: a
%   full 216-point sweep once produced identical results for every point because the model
%   was stale.
%
% 边界 / Boundary: 只生成 .slx，不运行仿真、不访问硬件；重建会先关闭并删除同名 .slx，
% 因此未保存的手工改动会丢失（这是有意的：模型不允许手工偏离生成脚本）。
% It only writes the .slx: no simulation, no hardware access. A rebuild closes and deletes
% the existing file, so manual edits are intentionally not preserved.
%
% 参考 / Reference: simulink/Simulink仿真工程说明.md, docs/仿真实机相关性验证.md

if nargin < 1 || isempty(modelName)
    modelName = 'foc_bringup';
end
modelName = char(modelName);
here = fileparts(mfilename('fullpath'));

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
slxPath = fullfile(here, [modelName '.slx']);
if exist(slxPath, 'file')
    delete(slxPath);
end

new_system(modelName);

%% ------------------------------------------------------------ model config
% 求解器配置 / Solver configuration:
%   FixedStepDiscrete + FixedStep = p.timing.pwmTs；被控对象按载波推进，控制器由
%   SystemSampleTime=p.timing.controlTs 以整数分频执行。
%   定步长实现，用变步长求解器会掩盖真实采样时序；
%   StopTime = p.timing.duration [s]，run_foc_sim.m 会在内存里改为 duration-Ts；
%   InitFcn 调用 foc_workspace_init()，使直接点"运行"的 GUI 用法也拿到一致的参数集；
%   SaveTime 打开、SignalLogging 关闭：日志靠显式的 To Workspace 块，不靠信号记录。
%   AlgebraicLoopMsg='error' 是有意为之：本模型不允许存在代数环，出现即说明拓扑接错，
%   不允许求解器用初始条件解糊过去。
% FixedStepDiscrete at p.timing.pwmTs; the plant advances at carrier rate while the
% controller has an explicit p.timing.controlTs integer sub-rate.
% StopTime is p.timing.duration [s] and is overridden in memory by run_foc_sim.m. InitFcn
% calls foc_workspace_init() so running the model from the GUI still uses a consistent
% parameter set. Logging is done by explicit To Workspace blocks. AlgebraicLoopMsg='error'
% is deliberate: an algebraic loop here means the topology is wrong, and the solver must not
% paper over it with initial-condition solving.
set_param(modelName, ...
    'Solver',           'FixedStepDiscrete', ...
    'FixedStep',        'p.timing.pwmTs', ...
    'StopTime',         'p.timing.duration', ...
    'InitFcn',          'p = foc_workspace_init();', ...
    'SaveOutput',       'off', ...
    'SaveTime',         'on', ...
    'SignalLogging',    'off', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'AlgebraicLoopMsg', 'error');

%% ------------------------------------------------------------- input group
% 输入常量块 / Input constants: 全部直接引用 p 的命名字段（Vbus [V]、TargetRpm [rpm]、
% Tload [N*m]），或引用两个定长向量（ControllerConfig、PlantConfig）。
% 这里引用的是字段名而不是数值，所以参数文件一改、重新仿真即生效，无需重建模型；
% 只有两个 .m 源文件（本质是代码）改动才需要重建。
% Every input constant references a named field of p (Vbus [V], TargetRpm [rpm], Tload
% [N*m]) or one of the two fixed-size vectors. Because these are field references rather
% than literals, parameter edits take effect on the next simulation without a rebuild; only
% edits to the two .m sources (which are code) require one.
add_block('simulink/Sources/Constant', [modelName '/Vbus'], ...
    'Value', 'p.bus.nominalV', 'Position', [30 60 100 90]);
add_block('simulink/Sources/Constant', [modelName '/TargetRpm'], ...
    'Value', 'p.timing.targetRpm', 'Position', [30 150 100 180]);
add_block('simulink/Sources/Constant', [modelName '/Tload'], ...
    'Value', 'p.loadTorqueNm', 'Position', [30 330 100 360]);
add_block('simulink/Sources/Constant', [modelName '/ControllerConfig'], ...
    'Value', 'p.controllerVector', 'Position', [30 220 130 250]);
add_block('simulink/Sources/Constant', [modelName '/PlantConfig'], ...
    'Value', 'p.plantVector', 'Position', [390 330 480 360]);

%% ------------------------------------------------------- controller and plant
% 两个 MATLAB Function 块分别由 controller_core.m 和 pmsm_plant.m 供源：控制律和电机
% 方程保持为可 review 的普通代码，拓扑仍是标准 Simulink 框图。注入的是文本副本，
% 因此这两个文件的内容在本次生成时被"冻结"进 .slx。
% The two MATLAB Function blocks are sourced from controller_core.m and pmsm_plant.m, so
% the control law and the machine equations stay reviewable plain code while the topology
% remains an ordinary Simulink diagram. What is injected is a text copy, so those two files
% are frozen into the .slx at build time.
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/ControllerCore'], 'Position', [220 40 380 300]);
set_matlab_function_source(modelName, 'ControllerCore', ...
    fullfile(here, 'controller_core.m'));
set_param([modelName '/ControllerCore'],'SystemSampleTime','p.timing.controlTs');

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/PmsmPlant'], 'Position', [500 40 660 300]);
set_matlab_function_source(modelName, 'PmsmPlant', ...
    fullfile(here, 'pmsm_plant.m'));
set_param([modelName '/PmsmPlant'],'SystemSampleTime','p.timing.pwmTs');

%% ------------------------------------------------------ port data attributes
% Declared explicitly because MATLAB Function blocks cannot always infer widths
% (the controller keeps a 64-element FIFO in persistent state, for example).
%
% 端口宽度必须显式声明：控制器在 persistent 里保存 64 点 FIFO，端口宽度无法可靠推断；
% 输出顺序与 controller_core.m 的函数签名、pmsm_plant.m 的函数签名严格一致，改签名
% 必须同步这里，否则 Simulink 会把输出接到错误的端口上（不报错但结果错）。
% Widths are declared explicitly because inference is unreliable (the controller keeps a
% 64-element FIFO in persistent state). The order here must match the function signatures
% exactly; a signature change without a matching edit would mis-wire outputs silently.
%
% controller_core(vbus, iabc, targetRpm, cfg) -> 17 outputs
set_function_ports(modelName, 'ControllerCore', [1 3 1 49], ...
    [3 1 1 1  1 1 1 1  1 1  1 1 1 3  1 1 1]);
% pmsm_plant(duty, vbus, loadTorque, cfg) -> 6 outputs
set_function_ports(modelName, 'PmsmPlant', [3 1 1 10], [3 1 1 1 1 1]);

%% ------------------------------------------------------------------ wiring
add_line(modelName, 'Vbus/1',        'ControllerCore/1', 'autorouting', 'on');
add_line(modelName, 'TargetRpm/1',   'ControllerCore/3', 'autorouting', 'on');
add_line(modelName, 'ControllerConfig/1', 'ControllerCore/4', 'autorouting', 'on');
add_line(modelName, 'Vbus/1',        'PmsmPlant/2',     'autorouting', 'on');
add_line(modelName, 'Tload/1',       'PmsmPlant/3',     'autorouting', 'on');
add_line(modelName, 'PlantConfig/1', 'PmsmPlant/4',     'autorouting', 'on');
% 控制器输出经离散延迟后送入被控对象。TIM1 的 CCR preload 只在 UEV 装载：
% 12/12 kHz 基线延迟 1 个 PWM 拍，24/12 kHz 候选延迟 2 个 PWM 拍；两者
% 都等于一个 12 kHz 控制拍，块本身同时承担零阶保持。
add_block('simulink/Discrete/Delay', [modelName '/DutyActuationDelay'], ...
    'DelayLength', 'p.timing.actuationDelayPwmTicks', ...
    'InitialCondition', ...
        'repmat(0.5,[3,1,max(1,p.timing.actuationDelayPwmTicks)])', ...
    'SampleTime', 'p.timing.pwmTs', 'Position', [410 75 465 105]);
add_line(modelName, 'ControllerCore/1', 'DutyActuationDelay/1', 'autorouting', 'on');
add_line(modelName, 'DutyActuationDelay/1', 'PmsmPlant/1', 'autorouting', 'on');

% Current feedback closes the loop. A Unit Delay is physically required, not
% just a solver workaround: on the target the ADC injected sample that the ISR
% consumes is latched before the duty update, so the controller always acts on
% the previous carrier period's current. Without the delay the model is an
% algebraic loop, which is also why Simulink rejected it.
% 电流反馈必须经过 Unit Delay，这是物理要求而不只是求解器权宜之计：目标板上 ISR 消费
% 的 ADC 注入采样在占空比更新之前就已经锁存，控制器永远只能看到上一载波周期的电流。
% 去掉这个延迟，模型直接构成代数环（控制器输出经 plant 又立刻回到输入），与上面
% AlgebraicLoopMsg='error' 一致——Simulink 会直接报错而不是悄悄求解。
% The Unit Delay is physically required, not just a solver workaround: on the target the
% injected ADC sample consumed by the ISR is latched before the duty update, so the
% controller always acts on the previous carrier period's current. Removing it creates an
% algebraic loop, consistent with AlgebraicLoopMsg='error', and Simulink refuses it rather
% than silently solving through.
add_block('simulink/Discrete/Unit Delay', [modelName '/CurrentFeedbackDelay'], ...
    'InitialCondition', '0', 'SampleTime', 'p.timing.pwmTs', ...
    'Position', [440 200 480 240]);
add_block('simulink/Discrete/Zero-Order Hold', [modelName '/CurrentControlSample'], ...
    'SampleTime', 'p.timing.controlTs', 'Position', [395 200 430 240]);
add_line(modelName, 'PmsmPlant/1', 'CurrentFeedbackDelay/1', 'autorouting', 'on');
add_line(modelName, 'CurrentFeedbackDelay/1', 'CurrentControlSample/1', 'autorouting', 'on');
add_line(modelName, 'CurrentControlSample/1', 'ControllerCore/2', 'autorouting', 'on');

%% ----------------------------------------------------------------- logging
% One To Workspace block per logged signal, so no vector width is ever guessed.
% 每个被记录信号一个 To Workspace 块（SaveFormat='Structure With Time'），避免靠向量
% 拼接导致列顺序歧义。信号表里的量纲：obsSpeedRpm/trueSpeedRpm/speedReferenceRpm
% [rpm]、state/reliable [-]、idRef/iqRef/idMeas/iqMeas/phaseCurrents [A]、
% vdCmd/vqCmd [V]、forcedAngle/obsAngle/trueAngle/controlAngle [rad]、simTime [s]、
% torqueE [N*m]、idPlant/iqPlant [A]、faultFlags [-]。run_foc_sim.m 按名字取用这些
% 记录，因此变量名即接口，不能随意改名。
% One To Workspace block per signal ('Structure With Time') so no column order is guessed.
% Units: speeds [rpm], state/reliable [-], currents [A], voltages [V], angles [rad], simTime
% [s], torqueE [N*m], faultFlags [-]. run_foc_sim.m fetches these by name, so the variable
% names are an interface and must not be renamed casually.
logSpecs = { ...
    'obsSpeedRpm',   'ControllerCore/2',  [760  30 840  60]; ...
    'state',         'ControllerCore/3',  [760  70 840 100]; ...
    'reliable',      'ControllerCore/4',  [760 110 840 140]; ...
    'idRef',         'ControllerCore/5',  [760 150 840 180]; ...
    'iqRef',         'ControllerCore/6',  [760 190 840 220]; ...
    'idMeas',        'ControllerCore/7',  [760 230 840 260]; ...
    'iqMeas',        'ControllerCore/8',  [760 270 840 300]; ...
    'vdCmd',         'ControllerCore/9',  [760 310 840 340]; ...
    'vqCmd',         'ControllerCore/10', [760 350 840 380]; ...
    'forcedAngle',   'ControllerCore/11', [760 390 840 420]; ...
    'obsAngle',      'ControllerCore/12', [760 430 840 460]; ...
    'simTime',       'ControllerCore/13', [760 470 840 500]; ...
    'trueSpeedRpm',  'PmsmPlant/2',       [760 510 840 540]; ...
    'trueAngle',     'PmsmPlant/3',       [760 550 840 580]; ...
    'torqueE',       'PmsmPlant/4',       [760 590 840 620]; ...
    'idPlant',       'PmsmPlant/5',       [760 630 840 660]; ...
    'iqPlant',       'PmsmPlant/6',       [760 670 840 700]; ...
    'phaseCurrents', 'ControllerCore/14', [900 350 990 380]; ...
    'controlAngle',  'ControllerCore/15', [900 390 990 420]; ...
    'speedReferenceRpm','ControllerCore/16',[900 430 990 460]; ...
    'faultFlags',    'ControllerCore/17', [900 470 990 500]  ...
    };
for k = 1:size(logSpecs, 1)
    name = logSpecs{k,1};  src = logSpecs{k,2};  pos = logSpecs{k,3};
    add_block('simulink/Sinks/To Workspace', [modelName '/' name], ...
        'VariableName', name, 'SaveFormat', 'Structure With Time', ...
        'Position', pos);
    add_line(modelName, src, [name '/1'], 'autorouting', 'on');
end

%% ---------------------------------------------------------------- metadata
% Locale-independent timestamp: datestr(now, fmt) throws on Windows locales
% where the format string is not recognised, which used to abort model
% generation on some machines.
% 时间戳用 datetime+Format 生成而不是 datestr(now,fmt)：后者在部分 Windows locale 下
% 会直接抛错，曾导致模型生成中途失败；这里要求结果不依赖机器区域设置。
% The stamp uses datetime with an explicit Format rather than datestr(now,fmt), which
% throws on some Windows locales and used to abort model generation. The result must be
% independent of the machine locale.
stamp = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));
set_param(modelName, 'Description', sprintf([ ...
    'FluxRT bring-up model, generated by make_foc_model.m at %s.\n' ...
    'Controller: SMO+PLL observer -> rev-up -> Id/Iq PI -> circle limit -> SVPWM.\n' ...
    'Plant: dq PMSM + mechanical load + dead-time loss, forward Euler at PWM rate.\n' ...
    'Parameters: init_foc_params.m.  Firmware mirror: foc_platform_stm32g431.c'], ...
    stamp));

save_system(modelName, slxPath);
mdl = modelName;
end

% ---------------------------------------------------------------------------
function set_matlab_function_source(modelName, blockName, sourceFile)
%SET_MATLAB_FUNCTION_SOURCE  Inject a .m file body into a MATLAB Function block.
% 把 .m 文件正文注入 MATLAB Function 块，相当于"以文件为源"。
% Injects a .m file body into a MATLAB Function block, i.e. file-backed source.
%
% 参数 / Parameters: sourceFile 必须存在，否则抛 make_foc_model:missingSource；
% chart.Script 直接接收整个文件文本，所以文件里的注释也会一并进入模型。
% sourceFile must exist or make_foc_model:missingSource is raised. chart.Script takes the
% whole file text, so the comments travel into the model as well.
if ~exist(sourceFile, 'file')
    error('make_foc_model:missingSource', 'Missing source file: %s', sourceFile);
end
txt   = fileread(sourceFile);
chart = find_chart(modelName, blockName);
chart.Script = txt;
end

% ---------------------------------------------------------------------------
function set_function_ports(modelName, blockName, inDims, outDims)
%SET_FUNCTION_PORTS  Declare dimensions and data types on a MATLAB Function block.
% 逐个端口声明宽度 [n x 1] 与数据类型（double），端口顺序即函数签名顺序。
% Declares width ([n x 1]) and data type (double) per port; the port order is the function
% signature order. inDims/outDims are element counts, one per port: the controller input is
% [1 3 1 49] (vbus scalar, three phase currents, target speed, 49-element config) and the
% plant input is [3 1 1 10].
chart = find_chart(modelName, blockName);
for k = 1:numel(inDims)
    p = chart.Inputs(k);
    p.DataType = 'double';
    p.Props.Array.Size = sprintf('[%d 1]', inDims(k));
end
for k = 1:numel(outDims)
    p = chart.Outputs(k);
    p.DataType = 'double';
    p.Props.Array.Size = sprintf('[%d 1]', outDims(k));
end
end

% ---------------------------------------------------------------------------
function chart = find_chart(modelName, blockName)
% 取回 MATLAB Function 块背后的 Stateflow.EMChart 句柄；找不到就报错，避免静默生成一个
% 没有源的块（那样模型能存盘但内容是空的，是最难查的一类失败）。
% Returns the Stateflow.EMChart behind a MATLAB Function block, or errors if absent: a
% block without a chart would still save but contain nothing, which is the hardest kind of
% failure to notice.
sf    = sfroot;
chart = sf.find('-isa', 'Stateflow.EMChart', ...
                '-and', 'Path', [modelName '/' blockName]);
if isempty(chart)
    error('make_foc_model:noChart', ...
          'MATLAB Function block %s/%s exposes no EMChart.', modelName, blockName);
end
end
