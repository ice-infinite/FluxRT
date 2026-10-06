function r = lab23_other_motors(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L23'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

s=linspace(-1,1,2001)';sm=.2;T=2*s*sm./(s.^2+sm^2);
% 步进静态平衡：Te=Tmax*sin(Nr*(command-position))。
Nr=50;micro=16;cmd=(0:micro)'*(pi/2/micro)/Nr;load=.3;Tmax=1;position=cmd-asin(load/Tmax)/Nr;
r.data=[s,T];r.columns={'slip','normalized_induction_torque'};
r.metrics.microstep_mechanical_deg=(pi/2/micro)/Nr*180/pi;r.metrics.static_load_error_deg=asin(load/Tmax)/Nr*180/pi;
mc_plot(outdir,'L23_induction',s,T,{'Kloss-type approximation'},'Slip','Normalized torque','Induction-motor torque-slip shape; no real machine parameters');
mc_plot(outdir,'L23_stepper',cmd*180/pi,[cmd,position]*180/pi,{'Command','Loaded equilibrium'},'Command (mechanical deg)','Position (mechanical deg)','Finer microstep resolution does not remove static load deflection');

mc_save(outdir, 'L23', r);
end
