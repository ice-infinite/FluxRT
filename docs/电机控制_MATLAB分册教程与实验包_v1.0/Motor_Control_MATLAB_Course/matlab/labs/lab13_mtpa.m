function r = lab13_mtpa(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L13'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_ipm_params();T=linspace(0,.8,81)';idgrid=linspace(-p.Imax,0,4001)';D=zeros(numel(T),5);
for k=1:numel(T)
 iq=T(k)./(1.5*p.p*(p.psi+(p.Ld-p.Lq)*idgrid));I2=idgrid.^2+iq.^2;
 I2(I2>p.Imax^2)=Inf;[val,j]=min(I2);assert(isfinite(val));
 D(k,:)=[idgrid(j),iq(j),sqrt(val),T(k)/(1.5*p.p*p.psi),T(k)];
end
r.data=[T,D(:,1:4)];r.columns={'target_torque_Nm','id_MTPA_A','iq_MTPA_A','I_MTPA_A','I_id0_A'};
r.metrics.max_current_saving_A=max(D(:,4)-D(:,3));r.metrics.recomputed_torque_error=max(abs(1.5*p.p*(p.psi+(p.Ld-p.Lq)*D(:,1)).*D(:,2)-T));
mc_plot(outdir,'L13_currents',T,D(:,1:2),{'id','iq'},'Torque (Nm)','Current (A)','MTPA current references found by a bounded grid search');
mc_plot(outdir,'L13_saving',T,D(:,3:4),{'MTPA','id=0'},'Torque (Nm)','Current-vector magnitude (A)','Same requested torque, different required current');
assert(r.metrics.recomputed_torque_error<1e-10 && all(D(:,3)<=D(:,4)+1e-5));

mc_save(outdir, 'L13', r);
end
