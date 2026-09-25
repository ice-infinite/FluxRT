function p = foc_workspace_init(varargin)
%FOC_WORKSPACE_INIT  Populate the base workspace for the Simulink model.
%
%   Called from the model InitFcn, so `sim('foc_bringup')` always uses a
%   consistent parameter set. Also callable directly:
%
%     p = foc_workspace_init('observerProfile','firmware');
%     out = sim('foc_bringup');
%
%   The struct `p` is assigned into the base workspace for the Constant blocks;
%   controller/plant tunables enter the MATLAB Function blocks through vectors.
%
% FluxRT —— 给 Simulink 模型准备 base 工作区的薄封装。
% FluxRT - a thin wrapper that prepares the base workspace for the Simulink model.
%
% 职责 / Responsibility:
%   - 被模型的 InitFcn 调用，保证 `sim('foc_bringup')` 这种直接运行也拿到一套一致的参数；
%   - 同时把控制器第一拍看到的初始三相电流 foc_iabc [A] 放进 base 工作区。
%   - Called from the model's InitFcn so a direct `sim('foc_bringup')` also gets a consistent
%     parameter set, and publishes the initial three-phase current foc_iabc [A] seen on the
%     first controller tick.
%
% 为什么是薄封装 / Why it stays thin: 参数全部来自 init_foc_params.m（唯一来源），这里
% 只做工作区发布，不派生新数值，避免出现"第二套默认值"。脚本调用时 run_foc_sim.m 已经
% 自己赋过 p，并会在内存里临时清空 InitFcn，防止这里的默认值把覆盖后的整定改回去。
% All values come from init_foc_params.m and nothing new is derived here, so no second set of
% defaults can appear. A scripted run has already published p and temporarily clears InitFcn
% in memory so these defaults cannot revert the overridden tuning.
%
% 安全默认 / Safe default: 默认 closedLoop=false，即只有观察器遥测和强制角开环，功率级
% 语义仍保持关闭；仿真本身不接触硬件。
% closedLoop defaults to false: observer telemetry with forced-angle open loop only, matching
% the firmware's power-stage-disabled default. Simulation never touches hardware.

if nargin == 0
    p = init_foc_params();
else
    p = init_foc_params(varargin{:});
end

% Initial current vector seen by the controller on the first tick.
% 初始电流向量 [A]：控制器第一拍用它做 Clarke/Park，取零表示"上电瞬间无电流"，
% 与实机 ADC 零点校准后的状态一致。
% Initial current vector in [A] seen by the controller on the first tick: zero represents the
% de-energised state right after power-up.
foc_iabc = [0; 0; 0];

assignin('base', 'p', p);
assignin('base', 'foc_iabc', foc_iabc);
end
