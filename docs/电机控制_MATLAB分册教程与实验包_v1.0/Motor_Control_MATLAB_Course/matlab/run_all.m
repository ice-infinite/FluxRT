function summary=run_all()
startup_course();summary=cell(24,3);
outdir=fullfile(mc_root(),'results_matlab');if ~exist(outdir,'dir'),mkdir(outdir);end
for k=1:24
 try
  r=run_lab(k);summary(k,:)={k,'PASS',r.metrics};
 catch ME
  summary(k,:)={k,'FAIL',ME.message};fprintf(2,'L%02d FAILED: %s\n',k,ME.message);
 end
end
save(fullfile(mc_root(),'results_matlab','run_summary.mat'),'summary');
disp(summary(:,1:2));
assert(~any(strcmp(summary(:,2),'FAIL')),'Some experiments failed. Inspect run_summary.mat.');
end
