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
set_param(modelName, ...
    'Solver',           'FixedStepDiscrete', ...
    'FixedStep',        'p.timing.ts', ...
    'StopTime',         'p.timing.duration', ...
    'InitFcn',          'p = foc_workspace_init();', ...
    'SaveOutput',       'off', ...
    'SaveTime',         'on', ...
    'SignalLogging',    'off', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'AlgebraicLoopMsg', 'error');

%% ------------------------------------------------------------- input group
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
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/ControllerCore'], 'Position', [220 40 380 300]);
set_matlab_function_source(modelName, 'ControllerCore', ...
    fullfile(here, 'controller_core.m'));

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/PmsmPlant'], 'Position', [500 40 660 300]);
set_matlab_function_source(modelName, 'PmsmPlant', ...
    fullfile(here, 'pmsm_plant.m'));

%% ------------------------------------------------------ port data attributes
% Declared explicitly because MATLAB Function blocks cannot always infer widths
% (the controller keeps a 64-element FIFO in persistent state, for example).
%
% controller_core(vbus, iabc, targetRpm, cfg) -> 17 outputs
set_function_ports(modelName, 'ControllerCore', [1 3 1 48], ...
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
add_line(modelName, 'ControllerCore/1', 'PmsmPlant/1',  'autorouting', 'on');

% Current feedback closes the loop. A Unit Delay is physically required, not
% just a solver workaround: on the target the ADC injected sample that the ISR
% consumes is latched before the duty update, so the controller always acts on
% the previous carrier period's current. Without the delay the model is an
% algebraic loop, which is also why Simulink rejected it.
add_block('simulink/Discrete/Unit Delay', [modelName '/CurrentFeedbackDelay'], ...
    'InitialCondition', '0', 'SampleTime', 'p.timing.ts', ...
    'Position', [440 200 480 240]);
add_line(modelName, 'PmsmPlant/1', 'CurrentFeedbackDelay/1', 'autorouting', 'on');
add_line(modelName, 'CurrentFeedbackDelay/1', 'ControllerCore/2', 'autorouting', 'on');

%% ----------------------------------------------------------------- logging
% One To Workspace block per logged signal, so no vector width is ever guessed.
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
stamp = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));
set_param(modelName, 'Description', sprintf([ ...
    'FluxRT bring-up model, generated by make_foc_model.m at %s.\n' ...
    'Controller: SMO+PLL observer -> rev-up -> Id/Iq PI -> circle limit -> SVPWM.\n' ...
    'Plant: dq PMSM + mechanical load + dead-time loss, forward Euler at 12 kHz.\n' ...
    'Parameters: init_foc_params.m.  Firmware mirror: foc_platform_stm32g431.c'], ...
    stamp));

save_system(modelName, slxPath);
mdl = modelName;
end

% ---------------------------------------------------------------------------
function set_matlab_function_source(modelName, blockName, sourceFile)
%SET_MATLAB_FUNCTION_SOURCE  Inject a .m file body into a MATLAB Function block.
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
sf    = sfroot;
chart = sf.find('-isa', 'Stateflow.EMChart', ...
                '-and', 'Path', [modelName '/' blockName]);
if isempty(chart)
    error('make_foc_model:noChart', ...
          'MATLAB Function block %s/%s exposes no EMChart.', modelName, blockName);
end
end
