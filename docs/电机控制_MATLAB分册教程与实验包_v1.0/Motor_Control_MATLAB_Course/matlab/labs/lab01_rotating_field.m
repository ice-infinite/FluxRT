function r = lab01_rotating_field(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L01'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

t=linspace(0,.04,2001)'; fe=50; I=6; th=2*pi*fe*t;
a=I*cos(th);b=I*cos(th-2*pi/3);c=I*cos(th+2*pi/3);
alpha=(2/3)*(a-.5*b-.5*c);beta=(b-c)/sqrt(3);
mag=hypot(alpha,beta);
r.data=[t,a,b,c,alpha,beta,mag];
r.columns={'time_s','ia_A','ib_A','ic_A','alpha_A','beta_A','magnitude_A'};
r.metrics.sum_abc_max=max(abs(a+b+c));r.metrics.magnitude_error=max(abs(mag-I));
mc_plot(outdir,'L01_abc',t,[a,b,c],{'ia','ib','ic'},'Time (s)','Current (A)','Three-phase currents: 120-degree time shifts');
mc_plot(outdir,'L01_vector',alpha,beta,{},'Alpha (A)','Beta (A)','Current space-vector locus (not magnetic flux in tesla)');
assert(r.metrics.sum_abc_max<1e-10 && r.metrics.magnitude_error<1e-10);

mc_save(outdir, 'L01', r);
end
