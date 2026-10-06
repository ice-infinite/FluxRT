function model=build_rl_pi_model()
% 只依赖 MATLAB+Simulink；未在交付环境执行。运行后才会产生 .slx。
% 若同名模型已打开则停止，避免修改用户未保存工作。
startup_course();model='course_rl_pi';
assert(license('test','Simulink'),'Simulink license is required.');
assert(~bdIsLoaded(model),'Model already loaded. Save/close it before building.');
outdir=fullfile(mc_root(),'results_matlab','simulink');if ~exist(outdir,'dir'),mkdir(outdir);end
outfile=fullfile(outdir,[model '.slx']);assert(~exist(outfile,'file'),'Existing model will not be overwritten.');
new_system(model);open_system(model);
set_param(model,'Solver','ode4','SolverType','Fixed-step','FixedStep','1e-5','StopTime','.05');
add_block('simulink/Sources/Step',[model '/Iref'],'Time','.002','Before','0','After','5','Position',[30 65 60 95]);
add_block('simulink/Math Operations/Sum',[model '/Error'],'Inputs','+-','Position',[105 65 130 95]);
add_block('simulink/Math Operations/Gain',[model '/Kp'],'Gain','1.2566370614','Position',[170 35 240 65]);
add_block('simulink/Math Operations/Gain',[model '/Ki'],'Gain','628.3185307','Position',[170 125 240 155]);
add_block('simulink/Continuous/Integrator',[model '/Integral'],'Position',[270 120 300 160]);
add_block('simulink/Math Operations/Sum',[model '/PI'],'Inputs','++','Position',[340 65 365 100]);
add_block('simulink/Continuous/Transfer Fcn',[model '/RL'],'Numerator','[1]','Denominator','[0.0004 0.2]','Position',[410 60 505 100]);
add_block('simulink/Sinks/Scope',[model '/CurrentScope'],'Position',[555 55 595 95]);
add_block('simulink/Sinks/To Workspace',[model '/CurrentLog'],'VariableName','course_current','SaveFormat','Structure With Time','Position',[550 130 670 160]);
add_line(model,'Iref/1','Error/1');add_line(model,'Error/1','Kp/1');add_line(model,'Error/1','Ki/1');
add_line(model,'Ki/1','Integral/1');add_line(model,'Kp/1','PI/1');add_line(model,'Integral/1','PI/2');
add_line(model,'PI/1','RL/1');add_line(model,'RL/1','Error/2','autorouting','on');add_line(model,'RL/1','CurrentScope/1');add_line(model,'RL/1','CurrentLog/1');
save_system(model,outfile);
fprintf('Built %s\nThis is a CONTINUOUS unsaturated PI baseline, not the discrete saturated L07 model.\n',outfile);
sim(model);
end
