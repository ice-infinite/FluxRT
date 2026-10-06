function r = lab05_pmsm_model(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L05'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_params();we=400;h=10e-6;t=(0:h:.02)';N=numel(t);X=zeros(N,2);
vd=-we*p.Lq*4;vq=p.R*4+we*p.psi;
f=@(x)[(vd-p.R*x(1)+we*p.Lq*x(2))/p.Ld;(vq-p.R*x(2)-we*(p.Ld*x(1)+p.psi))/p.Lq];
for k=1:N-1,X(k+1,:)=mc_rk4(f,X(k,:)',h)';end
Te=1.5*p.p*(p.psi*X(:,2)+(p.Ld-p.Lq)*X(:,1).*X(:,2));
res=zeros(N,1);
for k=1:N
 dx=f(X(k,:)');pe=1.5*(vd*X(k,1)+vq*X(k,2));pcu=1.5*p.R*sum(X(k,:).^2);
 dW=1.5*(p.Ld*X(k,1)*dx(1)+p.Lq*X(k,2)*dx(2));
 res(k)=pe-pcu-dW-Te(k)*(we/p.p);
end
r.data=[t,X,Te,res];r.columns={'time_s','id_A','iq_A','torque_Nm','power_balance_residual_W'};
r.metrics.power_residual_max=max(abs(res));r.metrics.final_iq=X(end,2);
mc_plot(outdir,'L05_dq',t,X,{'id','iq'},'Time (s)','Current (A)','Coupled PMSM dq dynamics at imposed constant speed');
mc_plot(outdir,'L05_energy',t,res,{'pe-pcu-dW-Te*wm'},'Time (s)','Residual (W)','Instantaneous energy-balance check');
assert(r.metrics.power_residual_max<1e-8);

mc_save(outdir, 'L05', r);
end
