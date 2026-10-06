function r = lab18_regeneration(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L18'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=1e-4;t=(0:h:.4)';J=2e-4;w0=300;Tb=.25;C=.0022;V0=48;Rb=20;N=numel(t);
w=w0*max(0,1-t/Tb);Preg=J*w*(w0/Tb);E0=.5*C*V0^2;Ea=E0;Eb=E0;on=false;D=zeros(N,5);diss=0;
for k=1:N
 va=sqrt(2*Ea/C);vb=sqrt(2*Eb/C);
 if vb>52,on=true;elseif vb<50,on=false;end
 Pbr=double(on)*vb^2/Rb;D(k,:)=[va,vb,Preg(k),Pbr,diss];
 if k<N
  Ea=Ea+h*Preg(k);Eb=Eb+h*(Preg(k)-Pbr);diss=diss+h*Pbr;
 end
end
r.data=[t,w,D];r.columns={'time_s','speed_rad_s','bus_no_sink_V','bus_brake_V','regen_W','brake_W','dissipated_J'};
r.metrics.analytic_final_no_sink=sqrt(V0^2+J*w0^2/C);r.metrics.numeric_final_no_sink=D(end,1);r.metrics.max_braked_voltage=max(D(:,2));r.metrics.initial_mechanical_J=.5*J*w0^2;
mc_plot(outdir,'L18_bus',t,D(:,1:2),{'No energy sink','Hysteretic brake resistor'},'Time (s)','DC bus voltage (V)','Finite capacitor energy: ideal braking energy balance');
mc_plot(outdir,'L18_power',t,D(:,3:4),{'Regeneration','Resistor pulse power'},'Time (s)','Power (W)','Pulse power rating and total dissipated energy are different');
assert(abs(r.metrics.numeric_final_no_sink-r.metrics.analytic_final_no_sink)<.1 && r.metrics.max_braked_voltage<54);

mc_save(outdir, 'L18', r);
end
