function selftest_math()
startup_course();
assert(abs(mc_wrap(2*pi))<1e-12);assert(mc_wrap(pi)==-pi);
p=mc_params();assert(abs(p.Kt-.12)<1e-12);
for th=linspace(-pi,pi,101)
 for amp=[0,5,20,48/sqrt(3)]
  v=amp*[cos(th);sin(th)];[d,vr]=mc_svpwm(v,48);
  assert(all(d>=0 & d<=1));assert(norm(v-vr)<1e-10);
 end
end
[d,v]=mc_svpwm([100;100],48);assert(norm(v)<=48/sqrt(3)+1e-9);
for th=linspace(-pi,pi,101)
 P=[cos(th),sin(th);-sin(th),cos(th)];assert(norm(P'*P-eye(2))<1e-12);
end
fprintf('Math assertions passed in the runtime shown above.\n');
end
