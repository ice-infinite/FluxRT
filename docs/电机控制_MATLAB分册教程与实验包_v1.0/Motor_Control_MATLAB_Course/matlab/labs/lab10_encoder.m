function r = lab10_encoder(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L10'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Ts=50e-6;t=(0:Ts:.08)';wm=100*2*pi/60;th=6+wm*t;
q12=round(th/(2*pi/4096))*(2*pi/4096);q18=round(th/(2*pi/2^18))*(2*pi/2^18);
w12=[0;mc_wrap(diff(q12))/Ts];w18=[0;mc_wrap(diff(q18))/Ts];
a=exp(-2*pi*60*Ts);wf=zeros(size(w12));
for k=2:numel(t),wf(k)=a*wf(k-1)+(1-a)*w12(k);end
r.data=[t,mod(th,2*pi),mod(q12,2*pi),w12,w18,wf];r.columns={'time_s','true_angle_rad','angle12_rad','speed12_rad_s','speed18_rad_s','filtered_speed12_rad_s'};
r.metrics.rms_speed12=sqrt(mean((w12(2:end)-wm).^2));r.metrics.rms_speed18=sqrt(mean((w18(2:end)-wm).^2));r.metrics.highspeed_delay_deg=7*(6000*2*pi/60)*20e-6*180/pi;
mc_plot(outdir,'L10_angle',t,mod(q12,2*pi),{'12-bit single-turn angle'},'Time (s)','Angle (rad)','Angle wrap must not become a speed spike');
sel=t<.02;
mc_plot(outdir,'L10_speed',t(sel),[wm*ones(sum(sel),1),w12(sel),w18(sel),wf(sel)],{'True','12-bit difference','18-bit difference','12-bit filtered'},'Time (s)','Speed (rad/s)','Resolution, differentiation noise, and filter lag');

mc_save(outdir, 'L10', r);
end
