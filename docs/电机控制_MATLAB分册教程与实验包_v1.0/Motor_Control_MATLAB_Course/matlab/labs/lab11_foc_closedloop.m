function r = lab11_foc_closedloop(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L11'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_foc_sim(0);b=mc_foc_sim(1);c=mc_foc_sim(2);
r.data=[a,b(:,3:5),c(:,3:5)];
r.columns={'time_s','speed_ref_rad_s','speed_rad_s','id_A','iq_A','iq_ref_A','voltage_norm_V','Te_Nm','load_Nm','dutyA','dutyB','dutyC','offset_speed_rad_s','offset_id_A','offset_iq_A','bias_speed_rad_s','bias_id_A','bias_iq_A'};
r.metrics.final_speed_error=abs(a(end,3)-a(end,2));r.metrics.max_voltage=max(a(:,7));r.metrics.max_current=max(hypot(a(:,4),a(:,5)));r.metrics.offset_final_id=b(end,4);
mc_plot(outdir,'L11_speed',a(:,1),a(:,2:3),{'Reference','Measured'},'Time (s)','Speed (rad/s)','Full average-value FOC: start, load step, speed step');
mc_plot(outdir,'L11_current',a(:,1),[a(:,4:5),a(:,6)],{'True id','True iq','iq reference'},'Time (s)','Current (A)','Current loops inside the speed loop');
mc_plot(outdir,'L11_voltage',a(:,1),a(:,7),{'Applied dq norm'},'Time (s)','Voltage (V)','Circular voltage limit and actuator demand');
mc_plot(outdir,'L11_fault_effect',a(:,1),[a(:,4),b(:,4),c(:,4)],{'Normal true id','15deg encoder error','0.3A phase-A sensing bias'},'Time (s)','True rotor-frame id (A)','Apparently rotating is not equivalent to correctly calibrated');
assert(r.metrics.final_speed_error<2 && r.metrics.max_voltage<=.95*48/sqrt(3)+1e-8);

mc_save(outdir, 'L11', r);
end
