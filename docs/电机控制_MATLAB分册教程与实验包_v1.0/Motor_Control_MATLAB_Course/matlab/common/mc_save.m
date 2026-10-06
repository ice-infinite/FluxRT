function mc_save(outdir,name,r)
% 每项实验统一有带列名 CSV、MAT 数据和数值指标。
assert(all(isfinite(r.data(~isnan(r.data)))),'Infinite data detected.');
if ~exist(outdir,'dir'),mkdir(outdir);end
p=fullfile(outdir,[name '_data.csv']); fid=fopen(p,'w');
assert(fid>=0,'Cannot create output file.');
clean=onCleanup(@()fclose(fid));
fprintf(fid,'%s',r.columns{1});
for j=2:numel(r.columns),fprintf(fid,',%s',r.columns{j});end
fprintf(fid,'\n');
fmt=[repmat('%.12g,',1,size(r.data,2)-1) '%.12g\n'];
fprintf(fid,fmt,r.data.'); clear clean;
save(fullfile(outdir,[name '_data.mat']),'r');
fid=fopen(fullfile(outdir,[name '_metrics.txt']),'w');
assert(fid>=0);clean=onCleanup(@()fclose(fid));
fprintf(fid,'Runtime: %s\n',version);
keys=fieldnames(r.metrics);
for k=1:numel(keys)
 v=r.metrics.(keys{k});fprintf(fid,'%s = %.12g\n',keys{k},v);
end
end
