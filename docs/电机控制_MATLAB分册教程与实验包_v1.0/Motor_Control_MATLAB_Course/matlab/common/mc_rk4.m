function xn=mc_rk4(f,x,h)
k1=f(x); k2=f(x+h*k1/2); k3=f(x+h*k2/2); k4=f(x+h*k3);
xn=x+h*(k1+2*k2+2*k3+k4)/6;
end
