function r = lab22_fixedpoint(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L22'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

th=linspace(-pi,pi,4001)';x=.9*cos(th);y=.9*sin(th);
q=@(v)min(32767,max(-32768,round(v*32768)));
xq=q(x);yq=q(y);cq=q(cos(th));sq=q(sin(th));
dq=(xq.*cq+yq.*sq)/32768^2;qq=(-xq.*sq+yq.*cq)/32768^2;
% 上式用double精确承载Q15整数乘加，展示量化误差；并非已执行MCU定点指令。
dtrue=x.*cos(th)+y.*sin(th);qtrue=-x.*sin(th)+y.*cos(th);
r.data=[th,dtrue,dq,qtrue,qq];r.columns={'theta_rad','d_float','d_Q15_inputs','q_float','q_Q15_inputs'};
r.metrics.d_max_error=max(abs(dq-dtrue));r.metrics.q_max_error=max(abs(qq-qtrue));
mc_plot(outdir,'L22_quantization',th,[dq-dtrue,qq-qtrue],{'d error','q error'},'Angle (rad)','Normalized error','Q15 input/trigonometric quantization with wide accumulation');
assert(max([r.metrics.d_max_error,r.metrics.q_max_error])<2e-4);

mc_save(outdir, 'L22', r);
end
