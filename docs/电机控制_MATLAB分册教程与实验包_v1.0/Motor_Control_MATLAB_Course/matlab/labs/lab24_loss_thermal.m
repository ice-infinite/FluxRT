function r = lab24_loss_thermal(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L24'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=.02;t=(0:h:180)';N=numel(t);Ta=25;Rth=8;Cth=8;Tj=Ta;D=zeros(N,5);
for k=1:N
 Irms=20+20*(t(k)>=60);Iedge=Irms;V=48;fpwm=20000;R25=.004;alpha=.006;trtf=60e-9;
 Pcond=Irms^2*R25*(1+alpha*(Tj-25));Psw=.5*V*Iedge*trtf*fpwm;
 Pgate=30e-9*10*fpwm;D(k,:)=[Tj,Pcond,Psw,Pgate,Irms];
 Tj=Tj+h*(Pcond+Psw-(Tj-Ta)/Rth)/Cth;
end
r.data=[t,D];r.columns={'time_s','junction_C','conduction_W','switching_W','gate_drive_W','device_rms_A'};
r.metrics.final_temperature_C=D(end,1);r.metrics.gate_drive_W=D(end,4);
mc_plot(outdir,'L24_temperature',t,D(:,1),{'One-pole thermal estimate'},'Time (s)','Temperature (degC)','Per-device RMS current step: 20A to 40A');
mc_plot(outdir,'L24_losses',t,D(:,2:4),{'Conduction','Switching overlap','Gate-drive supply'},'Time (s)','Power (W)','Gate-drive loss is shown separately, not added to MOSFET junction heat');

mc_save(outdir, 'L24', r);
end
