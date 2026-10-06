function r = lab02_rl_mechanics(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L02'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_params();h=10e-6;t=(0:h:.02)';u=2;N=numel(t);i=zeros(N,1);w=i;
a=exp(-p.R*h/p.Ld);b=(1-a)/p.R;
for k=1:N-1
 i(k+1)=a*i(k)+b*u;
 % 电流由堵转 RL 教学对象给定，此机械积分是独立的转矩-惯量示例，非完整电机闭环。
 Te=p.Kt*i(k);w(k+1)=w(k)+h*(Te-p.B*w(k))/p.J;
end
exact=u/p.R*(1-exp(-p.R*t/p.Ld));
r.data=[t,i,exact,w];r.columns={'time_s','current_A','analytic_A','separate_mechanics_rad_s'};
r.metrics.RL_max_error=max(abs(i-exact));r.metrics.time_constant_s=p.Ld/p.R;
mc_plot(outdir,'L02_rl',t,[i,exact],{'Exact-ZOH recurrence','Analytical curve'},'Time (s)','Current (A)','RL step response: tau=L/R');
mc_plot(outdir,'L02_mechanical',t,w,{'Ideal torque-driven inertia'},'Time (s)','Speed (rad/s)','Separate mechanical example; back EMF not coupled');
assert(r.metrics.RL_max_error<1e-9);

mc_save(outdir, 'L02', r);
end
