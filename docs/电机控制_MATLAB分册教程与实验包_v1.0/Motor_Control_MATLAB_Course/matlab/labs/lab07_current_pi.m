function r = lab07_current_pi(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L07'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_current_loop(200,0,true,1);b=mc_current_loop(800,0,true,1);
r.data=[a(:,1:3),b(:,3),a(:,4),b(:,4)];r.columns={'time_s','reference_A','i_fc200_A','i_fc800_A','v_fc200_V','v_fc800_V'};
r.metrics.final_error_slow=abs(a(end,3)-5);r.metrics.final_error_fast=abs(b(end,3)-5);
mc_plot(outdir,'L07_response',a(:,1)*1000,[a(:,2),a(:,3),b(:,3)],{'Reference','fc=200Hz','fc=800Hz'},'Time (ms)','Current (A)','PI bandwidth change with identical RL plant and voltage limit');
mc_plot(outdir,'L07_voltage',a(:,1)*1000,[a(:,4),b(:,4)],{'fc=200Hz','fc=800Hz'},'Time (ms)','Voltage (V)','Faster reference response demands more voltage');
assert(r.metrics.final_error_slow<1e-4 && r.metrics.final_error_fast<1e-4);

mc_save(outdir, 'L07', r);
end
