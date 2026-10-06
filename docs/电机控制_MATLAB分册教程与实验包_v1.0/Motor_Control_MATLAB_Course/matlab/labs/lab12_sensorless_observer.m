function r = lab12_sensorless_observer(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L12'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

% 恒速、外力强制旋转的SPMSM输入。SMO不是完整无感启动闭环。
p=mc_params();h=10e-6;t=(0:h:.08)';we=400;th=we*t;I=2;
i=[-I*sin(th),I*cos(th)];di=[-I*we*cos(th),-I*we*sin(th)];
e=p.psi*we*[-sin(th),cos(th)];v=p.R*i+p.Ld*di+e;
hat=[0 0];ef=[0 0];Eh=zeros(size(i));K=12;boundary=.12;fc=1200;af=exp(-2*pi*fc*h);
for k=1:numel(t)
 inj=K*tanh((hat-i(k,:))/boundary);
 hat=hat+h*(-p.R*hat+v(k,:)-inj)/p.Ld;
 ef=af*ef+(1-af)*inj;Eh(k,:)=ef;
end
angle=atan2(-Eh(:,1),Eh(:,2));rawerr=mc_wrap(angle-th);
% 仅离线诊断使用已知恒速。真实无感补偿应改用可靠的估计速度。
comp=mc_wrap(angle+atan(we/(2*pi*fc)));err=mc_wrap(comp-th);
% 同一个测量电压扰动，比较低速和高速反电动势的可观测条件。
noise=[.08*sin(2*pi*1700*t),.06*cos(2*pi*1300*t)];
low=.2*[-sin(th),cos(th)]+noise;high=8*[-sin(th),cos(th)]+noise;
lowerr=mc_wrap(atan2(-low(:,1),low(:,2))-th);higherr=mc_wrap(atan2(-high(:,1),high(:,2))-th);
r.data=[t,e,Eh,rawerr,err,lowerr,higherr];r.columns={'time_s','emf_alpha_V','emf_beta_V','estimated_alpha_V','estimated_beta_V','raw_angle_error_rad','diagnostic_comp_error_rad','low_emf_error_rad','high_emf_error_rad'};
sel=t>.03;r.metrics.smo_comp_rms_deg=sqrt(mean(err(sel).^2))*180/pi;r.metrics.low_emf_rms_deg=sqrt(mean(lowerr.^2))*180/pi;r.metrics.high_emf_rms_deg=sqrt(mean(higherr.^2))*180/pi;
mc_plot(outdir,'L12_emf',t,[e(:,1),Eh(:,1)],{'True e-alpha','SMO + LPF'},'Time (s)','Back EMF (V)','SMO supplied with synthesized measured voltage/current');
mc_plot(outdir,'L12_angle',t,[rawerr,err]*180/pi,{'Uncompensated','Known-speed diagnostic correction'},'Time (s)','Angle error (deg)','Observer lag; correction is not an autonomous startup solution');
mc_plot(outdir,'L12_low_speed',t,[lowerr,higherr]*180/pi,{'0.2 V EMF','8 V EMF'},'Time (s)','Angle error (deg)','Identical voltage disturbance, different EMF signal levels');
assert(r.metrics.low_emf_rms_deg>r.metrics.high_emf_rms_deg);

mc_save(outdir, 'L12', r);
end
