function p=mc_params()
% 教学用 SPMSM 参数，不对应用户的真实电机。
p.R=0.2; p.Ld=0.4e-3; p.Lq=0.4e-3; p.psi=0.02; p.p=4;
p.J=2e-4; p.B=1e-4; p.Vdc=48; p.Imax=8; p.Ts=50e-6;
p.Kt=1.5*p.p*p.psi;
end
