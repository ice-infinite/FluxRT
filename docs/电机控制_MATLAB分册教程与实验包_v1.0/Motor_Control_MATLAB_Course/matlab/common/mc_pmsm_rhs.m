function dx=mc_pmsm_rhs(x,vab,load,p)
% x=[id iq mechanical_speed electrical_angle]，vab在一个控制周期内保持。
c=cos(x(4));s=sin(x(4));vd=c*vab(1)+s*vab(2);vq=-s*vab(1)+c*vab(2);we=p.p*x(3);
Te=1.5*p.p*(p.psi*x(2)+(p.Ld-p.Lq)*x(1)*x(2));
dx=[(vd-p.R*x(1)+we*p.Lq*x(2))/p.Ld;
(vq-p.R*x(2)-we*(p.Ld*x(1)+p.psi))/p.Lq;
(Te-load-p.B*x(3))/p.J;we];
end
