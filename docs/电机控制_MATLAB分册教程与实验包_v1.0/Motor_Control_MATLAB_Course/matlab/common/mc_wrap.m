function a=mc_wrap(a)
% 统一 [-pi,pi)，用于差角；不是多圈位置存储。
a=mod(a+pi,2*pi)-pi;
end
