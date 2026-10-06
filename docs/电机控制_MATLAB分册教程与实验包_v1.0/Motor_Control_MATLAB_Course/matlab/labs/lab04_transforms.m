function r = lab04_transforms(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L04'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

th=linspace(0,4*pi,2001)';id0=0;iq0=6;
alpha=id0*cos(th)-iq0*sin(th);beta=id0*sin(th)+iq0*cos(th);
a=alpha;b=-.5*alpha+sqrt(3)/2*beta;c=-.5*alpha-sqrt(3)/2*beta;
al=(2/3)*(a-.5*b-.5*c);be=(b-c)/sqrt(3);
id=al.*cos(th)+be.*sin(th);iq=-al.*sin(th)+be.*cos(th);
delta=15*pi/180;id_bad=al.*cos(th+delta)+be.*sin(th+delta);iq_bad=-al.*sin(th+delta)+be.*cos(th+delta);
r.data=[th,a,b,c,id,iq,id_bad,iq_bad];r.columns={'theta_e_rad','ia_A','ib_A','ic_A','id_A','iq_A','id_15deg_A','iq_15deg_A'};
r.metrics.dq_error=max(abs([id-id0;iq-iq0]));r.metrics.id_offset_mean=mean(id_bad);r.metrics.iq_offset_mean=mean(iq_bad);
mc_plot(outdir,'L04_dq',th,[id,iq,id_bad,iq_bad],{'id correct','iq correct','id with +15deg','iq with +15deg'},'Electrical angle (rad)','Current (A)','A fixed encoder offset rotates the measured dq vector');
assert(r.metrics.dq_error<1e-10);

mc_save(outdir, 'L04', r);
end
