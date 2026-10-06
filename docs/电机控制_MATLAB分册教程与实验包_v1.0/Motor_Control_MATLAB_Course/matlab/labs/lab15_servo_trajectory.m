function r = lab15_servo_trajectory(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L15'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

a=mc_servo_sim(false);b=mc_servo_sim(true);r.data=[a,b(:,3:6)];
r.columns={'time_s','position_ref_rad','position_noFF_rad','speed_noFF_rad_s','torque_cmd_noFF_Nm','torque_noFF_Nm','load_Nm','position_FF_rad','speed_FF_rad_s','torque_cmd_FF_Nm','torque_FF_Nm'};
sel=a(:,1)>.05 & a(:,1)<.35;
r.metrics.tracking_RMS_noFF=sqrt(mean((a(sel,3)-a(sel,2)).^2));r.metrics.tracking_RMS_FF=sqrt(mean((b(sel,3)-b(sel,2)).^2));
mc_plot(outdir,'L15_position',a(:,1),[a(:,2:3),b(:,3)],{'Quintic reference','No feedforward','Velocity/acceleration FF'},'Time (s)','Position (rad)','Position-speed cascade with a 1 ms torque actuator');
mc_plot(outdir,'L15_error',a(:,1),[a(:,3)-a(:,2),b(:,3)-b(:,2)],{'No feedforward','With feedforward'},'Time (s)','Tracking error (rad)','Tracking lag and rejection of unknown load torque');

mc_save(outdir, 'L15', r);
end
