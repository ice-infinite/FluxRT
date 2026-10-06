function r = lab09_current_sensing(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L09'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Ts=50e-6;k=(0:399)';ta=(k+.15)*Ts;tb=(k+.5)*Ts;
truth=@(t)8*sin(2*pi*200*t);
% 合成开关尖峰：只用于隔离“采样位置”效应，不是某运放模型。
spike=@(t)4*exp(-((mod(t/Ts,1)-.15)/.025).^2);
rawa=truth(ta)+spike(ta);rawb=truth(tb)+spike(tb);
Rs=.005;G=20;Vref=3.3;Vbias=1.65;
quant=@(x)(min(4095,max(0,round((Vbias+Rs*G*x)/Vref*4096)))*Vref/4096-Vbias)/(Rs*G);
ia=quant(rawa);ib=quant(rawb);
r.data=[ta,tb,truth(ta),truth(tb),ia,ib];r.columns={'edge_sample_time_s','center_sample_time_s','truth_at_edge_A','truth_at_center_A','edge_sample_A','center_sample_A'};
r.metrics.edge_RMS_error=sqrt(mean((ia-truth(ta)).^2));r.metrics.center_RMS_error=sqrt(mean((ib-truth(tb)).^2));r.metrics.LSB_A=Vref/4096/(Rs*G);
mc_plot(outdir,'L09_samples',tb,[truth(tb),ia,ib],{'Truth at center','Edge sample (own time)','Center sample'},'Time (s)','Current (A)','ADC samples at contaminated vs settled parts of PWM');
mc_plot(outdir,'L09_error',tb,[ia-truth(ta),ib-truth(tb)],{'Edge error','Center error'},'Time (s)','Error (A)','Errors compared against truth at each actual sample instant');
assert(r.metrics.center_RMS_error<.01 && r.metrics.edge_RMS_error>3);

mc_save(outdir, 'L09', r);
end
