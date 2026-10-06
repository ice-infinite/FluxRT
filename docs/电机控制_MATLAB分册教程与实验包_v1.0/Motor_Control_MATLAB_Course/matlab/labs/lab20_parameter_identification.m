function r = lab20_parameter_identification(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L20'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_params();h=50e-6;t=(0:h:.12)';N=numel(t);u=2*(2*mod(floor(t/.001),2)-1)+.5*sin(2*pi*131*t);i=zeros(N,1);
a=exp(-p.R*h/p.Ld);b=(1-a)/p.R;
for k=1:N-1,i(k+1)=a*i(k)+b*u(k);end
im=i+.003*sin(2*pi*1573*t)+.002*cos(2*pi*2331*t);
Phi=[im(1:end-1),u(1:end-1)];coef=Phi\im(2:end);ae=coef(1);be=coef(2);
assert(ae>0 && ae<1 && be>0);Rest=(1-ae)/be;Lest=-Rest*h/log(ae);
ip=zeros(N,1);for k=1:N-1,ip(k+1)=ae*ip(k)+be*u(k);end
r.data=[t,u,i,im,ip];r.columns={'time_s','voltage_V','true_current_A','measured_current_A','fitted_model_current_A'};
r.metrics.R_est_Ohm=Rest;r.metrics.L_est_H=Lest;r.metrics.R_relative_error=abs(Rest-p.R)/p.R;r.metrics.L_relative_error=abs(Lest-p.Ld)/p.Ld;r.metrics.condition_Phi=cond(Phi);
mc_plot(outdir,'L20_fit',t,[im,ip],{'Synthetic measurement','Identified RL model'},'Time (s)','Current (A)','Locked-rotor RL identification with persistent excitation');
mc_plot(outdir,'L20_residual',t,im-ip,{'Measurement minus prediction'},'Time (s)','Residual (A)','Residual is needed; fitted parameters alone do not validate a model');
assert(r.metrics.R_relative_error<.05 && r.metrics.L_relative_error<.05);

mc_save(outdir, 'L20', r);
end
