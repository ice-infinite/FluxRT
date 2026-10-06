function check_environment()
fprintf('MATLAB runtime: %s\n',version);ver;
features={'Simulink','Real-Time_Workshop','RTW_Embedded_Coder','MATLAB_Coder'};
for k=1:numel(features)
 try,fprintf('%s license: %d\n',features{k},license('test',features{k}));catch,fprintf('%s: check manually\n',features{k});end
end
fprintf('Core L01-L24 do not require the toolboxes above.\n');
end
