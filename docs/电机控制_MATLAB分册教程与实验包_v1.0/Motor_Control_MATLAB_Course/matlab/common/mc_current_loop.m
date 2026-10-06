function r=mc_current_loop(fc,delay,aw,profile)
p=mc_params();h=p.Ts;t=(0:h:.08)';N=numel(t);i=zeros(N,1);u=i;integ=i;ref=i;z=0;
Kp=p.Ld*2*pi*fc;Ki=p.R*2*pi*fc;vmax=4;
a=exp(-p.R*h/p.Ld);b=(1-a)/p.R;
for k=1:N-1
 if profile==1,ref(k)=5*(t(k)>=.002);else,ref(k)=30*(t(k)>=.002 && t(k)<.025)+5*(t(k)>=.025);end
 e=ref(k)-i(max(1,k-delay));un=Kp*e+z;u(k)=min(vmax,max(-vmax,un));
 dz=Ki*e;
 if aw,dz=dz+2*pi*fc*(u(k)-un);end
 z=z+h*dz;integ(k)=z;i(k+1)=a*i(k)+b*u(k);
end
ref(end)=ref(end-1);u(end)=u(end-1);integ(end)=integ(end-1);
r=[t,ref,i,u,integ];
end
