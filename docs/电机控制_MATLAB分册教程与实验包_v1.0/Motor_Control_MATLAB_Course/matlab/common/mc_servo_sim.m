function y=mc_servo_sim(feedforward)
h=2e-4;t=(0:h:1.2)';N=numel(t);theta=0;w=0;torque=0;z=0;J=.01;B=.03;y=zeros(N,7);
for k=1:N
 u=min(1,max(0,(t(k)-.05)/.3));r=10*u^3-15*u^4+6*u^5;
 dr=(30*u^2-60*u^3+30*u^4)/.3;ddr=(60*u-180*u^2+120*u^3)/.3^2;
 wr=min(20,max(-20,8*(r-theta)+feedforward*dr));e=wr-w;
 raw=.8*e+z+feedforward*(J*ddr+B*dr);cmd=min(1,max(-1,raw));z=z+h*(10*e+40*(cmd-raw));
 load=.2*(t(k)>=.6);y(k,:)=[t(k),r,theta,w,cmd,torque,load];
 torque=torque+h*(cmd-torque)/.001;w=w+h*(torque-load-B*w)/J;theta=theta+h*w;
end
end
