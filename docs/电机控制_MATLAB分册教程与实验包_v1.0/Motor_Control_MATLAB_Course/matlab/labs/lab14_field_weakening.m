function r = lab14_field_weakening(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L14'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

p=mc_ipm_params();[ID,IQ]=ndgrid(linspace(-p.Imax,0,241),linspace(0,p.Imax,241));
T=1.5*p.p*(p.psi+(p.Ld-p.Lq)*ID).*IQ;cur=ID.^2+IQ.^2<=p.Imax^2+1e-9;
wm=linspace(0,1200,121)';D=nan(numel(wm),5);vlim=p.Vdc/sqrt(3);
for k=1:numel(wm)
 we=p.p*wm(k);VD=p.R*ID-we*p.Lq*IQ;VQ=p.R*IQ+we*(p.Ld*ID+p.psi);
 mask=cur & hypot(VD,VQ)<=vlim;Tc=T;Tc(~mask)=-Inf;
 [best,idx]=max(Tc(:));
 if isfinite(best),D(k,1:4)=[best,ID(idx),IQ(idx),hypot(VD(idx),VQ(idx))];end
 mask0=mask & abs(ID)<1e-12;vals=T(mask0);if ~isempty(vals),D(k,5)=max(vals);end
end
r.data=[wm,D];r.columns={'speed_rad_s','max_torque_Nm','id_A','iq_A','voltage_V','max_torque_id0_Nm'};
r.metrics.current_max=max(hypot(D(:,2),D(:,3)));r.metrics.voltage_max=max(D(:,4));r.metrics.highspeed_torque=D(end,1);
mc_plot(outdir,'L14_envelope',wm*60/(2*pi),D(:,[1 5]),{'Optimized under I/V limits','id=0 only'},'Mechanical speed (rpm)','Feasible maximum torque (Nm)','Steady-state grid envelope, not a dynamic weakening controller');
mc_plot(outdir,'L14_idiq',wm*60/(2*pi),D(:,2:3),{'id','iq'},'Mechanical speed (rpm)','Current (A)','Negative id trades current capacity for voltage headroom');
assert(r.metrics.current_max<=p.Imax+1e-8 && r.metrics.voltage_max<=vlim+1e-8);

mc_save(outdir, 'L14', r);
end
