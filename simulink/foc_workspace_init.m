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

if nargin == 0
    p = init_foc_params();
else
    p = init_foc_params(varargin{:});
end

% Initial current vector seen by the controller on the first tick.
foc_iabc = [0; 0; 0];

assignin('base', 'p', p);
assignin('base', 'foc_iabc', foc_iabc);
end
