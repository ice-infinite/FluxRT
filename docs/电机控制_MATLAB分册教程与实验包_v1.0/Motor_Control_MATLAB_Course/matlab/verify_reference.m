function report=verify_reference(id,absTol,relTol)
% 对比本地已生成的MATLAB CSV与随包Python参考；不自动声称完全等价。
% 默认阈值用于发现明显差异。跨语言差异需按模型/步长/事件规则分析。
if nargin<2,absTol=1e-7;end
if nargin<3,relTol=1e-6;end
startup_course();
assert(isscalar(id) && id==floor(id) && id>=1 && id<=24,'id must be 1..24');
assert(isscalar(absTol) && isfinite(absTol) && absTol>=0,'Invalid absolute tolerance');
assert(isscalar(relTol) && isfinite(relTol) && relTol>=0,'Invalid relative tolerance');
lab=sprintf('L%02d',id);
reference=fullfile(mc_root(),'reference_results',[lab '_data.csv']);
actual=fullfile(mc_root(),'results_matlab',lab,[lab '_data.csv']);
assert(exist(actual,'file')==2,'Run run_lab(id) first; local MATLAB CSV does not exist.');
a=readmatrix(reference,'NumHeaderLines',1);b=readmatrix(actual,'NumHeaderLines',1);
assert(isequal(size(a),size(b)),'Data dimensions differ: check time grids and output columns.');
fid=fopen(reference,'r');assert(fid>=0);clean=onCleanup(@()fclose(fid));
cols=strsplit(strtrim(fgetl(fid)),',');clear clean;
assert(numel(cols)==size(a,2),'Header/data mismatch.');
report.lab=lab;report.runtime=version;report.absTol=absTol;report.relTol=relTol;
report.columns=cols;report.max_abs_error=zeros(1,size(a,2));
report.passed=false(1,size(a,2));
for j=1:size(a,2)
 sameMissing=isequal(isnan(a(:,j)),isnan(b(:,j)));
 valid=isfinite(a(:,j)) & isfinite(b(:,j));
 noInf=~any(isinf(a(:,j))) && ~any(isinf(b(:,j)));
 if any(valid)
  delta=abs(a(valid,j)-b(valid,j));
  scale=abs(a(valid,j));
  report.max_abs_error(j)=max(delta);
  report.passed(j)=sameMissing && noInf && all(delta<=absTol+relTol*scale);
 else
  report.max_abs_error(j)=NaN;
  report.passed(j)=sameMissing && noInf && all(isnan(a(:,j)));
 end
 fprintf('%-32s max_abs=%12.5g pass=%d\n',cols{j},report.max_abs_error(j),report.passed(j));
end
report.all_passed=all(report.passed);
save(fullfile(mc_root(),'results_matlab',lab,[lab '_reference_comparison.mat']),'report');
fprintf('Overall comparison: %d. This is tolerance-based numerical comparison, not hardware validation.\n',report.all_passed);
end
