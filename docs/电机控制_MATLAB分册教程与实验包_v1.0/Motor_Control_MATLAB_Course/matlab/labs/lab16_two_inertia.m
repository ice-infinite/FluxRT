function r = lab16_two_inertia(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L16'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

Jm=.002;Jl=.01;K=40;C=.01;Bm=.002;Bl=.003;f=logspace(-1,3,1200)';
fr=sqrt(K*(1/Jm+1/Jl))/(2*pi);wn=2*pi*fr;G=zeros(size(f));H=G;
for k=1:numel(f)
 s=1i*2*pi*f(k);A=[Jm*s^2+(Bm+C)*s+K,-C*s-K;-C*s-K,Jl*s^2+(Bl+C)*s+K];
 x=A\[1;0];G(k)=s*x(1);H(k)=(s^2+2*.04*wn*s+wn^2)/(s^2+2*.4*wn*s+wn^2);
end
r.data=[f,20*log10(abs(G)),20*log10(abs(G.*H)),unwrap(angle(G))*180/pi];r.columns={'frequency_Hz','plant_dB','input_notched_dB','plant_phase_deg'};
r.metrics.resonance_undamped_Hz=fr;r.metrics.antiresonance_undamped_Hz=sqrt(K/Jl)/(2*pi);
mc_plot(outdir,'L16_resonance',log10(f),r.data(:,2:3),{'Plant','Plant times input notch'},'log10 frequency (Hz)','Magnitude (dB re 1 (rad/s)/Nm)','Two-inertia FRF; filtered input is not a stability proof');

mc_save(outdir, 'L16', r);
end
