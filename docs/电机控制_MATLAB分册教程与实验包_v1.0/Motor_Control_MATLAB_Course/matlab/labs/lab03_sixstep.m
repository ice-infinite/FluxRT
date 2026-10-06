function r = lab03_sixstep(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L03'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

th=linspace(0,2*pi,3601)';I=3;e=[cos(th),cos(th-2*pi/3),cos(th+2*pi/3)];
% 选最大反电势相注入电流、最小相抽出，其余悬空：每60度换一次。
cur=zeros(size(e));
for k=1:numel(th)
 [~,hi]=max(e(k,:));[~,lo]=min(e(k,:));cur(k,hi)=I;cur(k,lo)=-I;
end
% 两种电流总RMS平方相同：sum(sixstep.^2)=2I^2，正弦峰值=2I/sqrt(3)。
isin=(2*I/sqrt(3))*e;power6=sum(e.*cur,2);powersin=sum(e.*isin,2);
r.data=[th,cur,power6,powersin];r.columns={'theta_e_rad','ia_A','ib_A','ic_A','sixstep_normalized_power','sine_normalized_power'};
r.metrics.sixstep_relative_ripple=(max(power6)-min(power6))/mean(power6);
r.metrics.sine_relative_ripple=(max(powersin)-min(powersin))/mean(powersin);
mc_plot(outdir,'L03_commutation',th*180/pi,cur,{'ia','ib','ic'},'Electrical angle (deg)','Current (A)','Ideal six-step current, no finite commutation dynamics');
mc_plot(outdir,'L03_power',th*180/pi,[power6,powersin],{'Six-step on sinusoidal EMF','Sinusoidal current'},'Electrical angle (deg)','Normalized conversion power','Same copper-loss proxy; not a universal BLDC-vs-FOC verdict');

mc_save(outdir, 'L03', r);
end
