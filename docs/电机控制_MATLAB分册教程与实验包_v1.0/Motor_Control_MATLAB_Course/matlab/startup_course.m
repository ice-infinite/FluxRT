function startup_course()
% 从任意当前目录调用：先 addpath 到本文件所在 matlab 文件夹。
root = fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'common'),fullfile(root,'labs'),fullfile(root,'simulink'));
fprintf('Motor control course loaded. Runtime: %s\n', version);
fprintf('Run: run_lab(4), run_lab(11), or run_all\n');
end
