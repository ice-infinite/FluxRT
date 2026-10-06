function y=mc_foc_sim(mode)
% 完整平均值FOC。非开关器件仿真；母线固定，可吸收回灌。
% mode 0正常；1在0.12s注入15度电角误差；2在A相测量注入0.3A偏置。
p=mc_params();h=p.Ts;t=(0:h:.3)';N=numel(t);x=zeros(4,1);z=zeros(2,1);zs=0;iqref=0;
wc=2*pi*600;kp=[p.Ld;p.Lq]*wc;ki=p.R*wc;wn=2*pi*20;
kps=2*.9*wn*p.J/p.Kt;kis=wn^2*p.J/p.Kt;
y=zeros(N,12);
for k=1:N
 ref=100*(t(k)>=.02)+50*(t(k)>=.20);load=.15*(t(k)>=.12);
 off=(mode==1 && t(k)>=.12)*pi/12;bias=(mode==2 && t(k)>=.12)*.3;
 theta=x(4)+off;c=cos(x(4));s=sin(x(4));
 ab=[c*x(1)-s*x(2);s*x(1)+c*x(2)];
 abc=[ab(1)+bias;-.5*ab(1)+sqrt(3)/2*ab(2);-.5*ab(1)-sqrt(3)/2*ab(2)];
 abm=[(2/3)*(abc(1)-.5*abc(2)-.5*abc(3));(abc(2)-abc(3))/sqrt(3)];
 cm=cos(theta);sm=sin(theta);idm=[cm*abm(1)+sm*abm(2);-sm*abm(1)+cm*abm(2)];
 if mod(k-1,10)==0
  es=ref-x(3);raw=kps*es+zs;iqref=min(p.Imax,max(-p.Imax,raw));
  zs=zs+(10*h)*(kis*es+wn*(iqref-raw));
 end
 er=[0;iqref]-idm;we=p.p*x(3);ff=[-we*p.Lq*idm(2);we*(p.Ld*idm(1)+p.psi)];
 un=kp.*er+z+ff;vlim=.95*p.Vdc/sqrt(3);usat=un*min(1,vlim/max(norm(un),1e-12));
 z=z+h*(ki*er+wc*(usat-un));
 % 电压零阶保持区间的中点角预测；不包含真实硬件计算/串行延迟。
 thv=theta+we*h/2;vab=[cos(thv)*usat(1)-sin(thv)*usat(2);sin(thv)*usat(1)+cos(thv)*usat(2)];
 [duty,vab_actual]=mc_svpwm(vab,p.Vdc);
 Te=1.5*p.p*(p.psi*x(2)+(p.Ld-p.Lq)*x(1)*x(2));
 y(k,:)=[t(k),ref,x(3),x(1),x(2),iqref,norm(usat),Te,load,duty'];
 if k<N
  for j=1:2,x=mc_rk4(@(xx)mc_pmsm_rhs(xx,vab_actual,load,p),x,h/2);end
 end
 assert(all(isfinite(x)) && norm(x(1:2))<100 && abs(x(3))<3000,'FOC model exceeded teaching guard.');
end
end
