function [duty,vab]=mc_svpwm(vab,Vdc)
% 输入输出 alpha,beta 电压。圆限幅+居中零序注入，理想平均值模型。
assert(Vdc>0 && all(isfinite(vab))); vlim=Vdc/sqrt(3);
if norm(vab)>vlim,vab=vab*(vlim/norm(vab));end
vp=[vab(1);-.5*vab(1)+sqrt(3)/2*vab(2);-.5*vab(1)-sqrt(3)/2*vab(2)];
v0=-(max(vp)+min(vp))/2;
duty=.5+(vp+v0)/Vdc;
duty=min(1,max(0,duty));
% 由最终占空比重构平均相对中性点电压，而不是直接回传理想指令。
vphase=Vdc*(duty-mean(duty));
vab=[(2/3)*(vphase(1)-.5*vphase(2)-.5*vphase(3));(vphase(2)-vphase(3))/sqrt(3)];
end
