function r = lab06_svpwm(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L06'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Vdc=48;th=linspace(0,2*pi,2001)';A=.98*Vdc/sqrt(3);N=numel(th);D=zeros(N,3);re=zeros(N,2);
for k=1:N
 ref=A*[cos(th(k));sin(th(k))];[d,v]=mc_svpwm(ref,Vdc);D(k,:)=d';re(k,:)=(v-ref)';
end
% 一段独立开关示例，20 kHz 三角载波，参考角近似恒定。
h=1e-7;tt=(0:h:150e-6)';carrier=1-abs(2*mod(tt*20000,1)-1);
[d,~]=mc_svpwm(A*[cos(.4);sin(.4)],Vdc);g=double(carrier<d');vab=Vdc*(g(:,1)-g(:,2));
r.data=[th,D,re];r.columns={'theta_e_rad','duty_a','duty_b','duty_c','alpha_error_V','beta_error_V'};
r.metrics.reconstruction_error=max(abs(re(:)));r.metrics.duty_min=min(D(:));r.metrics.duty_max=max(D(:));
mc_plot(outdir,'L06_duty',th*180/pi,D,{'dA','dB','dC'},'Electrical angle (deg)','Duty ratio','Centered zero-sequence injection (linear SVPWM equivalent)');
mc_plot(outdir,'L06_switching',tt*1e6,vab,{'vAB'},'Time (us)','Line voltage (V)','Ideal switched line voltage; no dead time or parasitics');
assert(r.metrics.reconstruction_error<1e-10 && min(D(:))>=0 && max(D(:))<=1);

mc_save(outdir, 'L06', r);
end
