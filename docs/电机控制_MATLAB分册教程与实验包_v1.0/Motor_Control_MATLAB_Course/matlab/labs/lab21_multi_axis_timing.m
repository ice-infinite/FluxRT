function r = lab21_multi_axis_timing(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L21'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

axes=(1:8)';period=50;peraxis=8;management=6;jitter_budget=4;
worst=axes*peraxis+management+jitter_budget;util=worst/period;
% 每轴每毫秒一帧命令+一帧反馈；150bit/帧只是显式预算假设，不是标准最坏证明。
can_load=axes*2*1000*150/1e6;
r.data=[axes,worst,util,can_load];r.columns={'axes','budget_us','CPU_period_fraction','assumed_CAN_bus_fraction'};
r.metrics.four_axis_budget_us=worst(4);r.metrics.four_axis_CAN_fraction=can_load(4);
mc_plot(outdir,'L21_budget',axes,[util,can_load,ones(size(axes))],{'CPU period budget','Assumed CAN occupancy','100% boundary'},'Number of axes','Fraction','Compute timing and communication are separate feasibility checks');

mc_save(outdir, 'L21', r);
end
