function r = lab17_joint_impedance(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L17'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=1e-4;t=(0:h:.8)';N=numel(t);Kset=[5,30];Y=zeros(N,4);wall=.6;
for j=1:2
 theta=0;w=0;
 for k=1:N
  cmd=min(8,max(-8,Kset(j)*(1-theta)-1*w));
  if theta>wall,contact=max(0,150*(theta-wall)+.4*w);else,contact=0;end
  Y(k,[2*j-1,2*j])=[theta,contact];
  w=w+h*(cmd-contact-.02*w)/.02;theta=theta+h*w;
 end
end
r.data=[t,Y];r.columns={'time_s','soft_position_rad','soft_contact_Nm','stiff_position_rad','stiff_contact_Nm'};
r.metrics.peak_contact_soft=max(Y(:,2));r.metrics.peak_contact_stiff=max(Y(:,4));
mc_plot(outdir,'L17_position',t,[Y(:,1),Y(:,3),wall*ones(N,1)],{'K=5 Nm/rad','K=30 Nm/rad','Wall angle'},'Time (s)','Joint angle (rad)','Impedance against a unilateral compliant contact');
mc_plot(outdir,'L17_contact',t,Y(:,[2 4]),{'Soft controller','Stiff controller'},'Time (s)','Contact torque (Nm)','Higher command stiffness can increase contact loads');

mc_save(outdir, 'L17', r);
end
