function r = lab08_windup_delay(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L08'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_current_loop(500,0,false,2);b=mc_current_loop(500,0,true,2);
c=mc_current_loop(800,0,true,1);d=mc_current_loop(800,4,true,1);
r.data=[a(:,1:3),b(:,3),a(:,5),b(:,5),c(:,3),d(:,3)];
r.columns={'time_s','reference_A','noAW_A','AW_A','integral_noAW_V','integral_AW_V','delay0_A','delay4_A'};
r.metrics.windup_peak=max(abs(a(:,5)));r.metrics.aw_integral_peak=max(abs(b(:,5)));
r.metrics.delay4_rms=sqrt(mean((d(end-400:end,3)-5).^2));
mc_plot(outdir,'L08_aw',a(:,1)*1000,[a(:,2:3),b(:,3)],{'Reference','No anti-windup','Back-calculation'},'Time (ms)','Current (A)','Unreachable 30 A request: plant limit is 4/0.2=20 A');
mc_plot(outdir,'L08_integral',a(:,1)*1000,[a(:,5),b(:,5)],{'No AW','AW'},'Time (ms)','Integrator state (V)','Saturation recovery is a state-management problem');
mc_plot(outdir,'L08_delay',c(:,1)*1000,[c(:,2:3),d(:,3)],{'Reference','No extra delay','4-sample measurement delay'},'Time (ms)','Current (A)','The same PI gains with additional feedback delay');

mc_save(outdir, 'L08', r);
end
