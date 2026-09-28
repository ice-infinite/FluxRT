function count = test_fluxrt_simulation_contract(projectRoot)
%TEST_FLUXRT_SIMULATION_CONTRACT Negative tests for the MATLAB loader.
% Every fixture is isolated under tempdir and removed even on failure.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
end
load_fluxrt_simulation_contract(projectRoot);
cases = {'hash','missing','version','units','identity'};
for k = 1:numel(cases)
    tempRoot = tempname;
    cleanup = onCleanup(@()remove_tree(tempRoot));
    copy_fixture(projectRoot,tempRoot);
    mutate_fixture(tempRoot,cases{k});
    failed = false;
    try
        load_fluxrt_simulation_contract(tempRoot);
    catch problem
        failed = startsWith(problem.identifier,'FluxRT:Contract:');
    end
    assert(failed,'FluxRT:Contract:Test','negative case %s did not fail closed',cases{k});
    clear cleanup
end
count = numel(cases);
fprintf('FLUXRT_SIM_CONTRACT_NEGATIVE_PASS cases=%d\n',count);
end

function copy_fixture(sourceRoot,targetRoot)
relative = { ...
    'simulation/contracts/legacy_speed_start.bundle.json', ...
    'simulation/contracts/fluxrt-simulation-v1.schema.json', ...
    'simulation/contracts/trace-schema-v1.json', ...
    'simulation/contracts/comparison-gates-v1.json', ...
    'simulation/profiles/stm32g431_gbm2804h_reference_v1.json', ...
    'simulation/scenarios/legacy_speed_start.json'};
for k = 1:numel(relative)
    destination = fullfile(targetRoot,strrep(relative{k},'/',filesep));
    parent = fileparts(destination);
    if ~exist(parent,'dir'), mkdir(parent); end
    copyfile(fullfile(sourceRoot,strrep(relative{k},'/',filesep)),destination);
end
end

function mutate_fixture(root,name)
profilePath = fullfile(root,'simulation','profiles','stm32g431_gbm2804h_reference_v1.json');
scenarioPath = fullfile(root,'simulation','scenarios','legacy_speed_start.json');
bundlePath = fullfile(root,'simulation','contracts','legacy_speed_start.bundle.json');
bundle = read_struct(bundlePath);
switch name
    case 'hash'
        profile = read_struct(profilePath);
        profile.rated_current_a = 0.7;
        write_struct(profilePath,profile); % bundle hash intentionally remains stale
        return
    case 'missing'
        profile = rmfield(read_struct(profilePath),'board_id');
        write_struct(profilePath,profile);
        bundle.profile_sha256 = file_hash(profilePath);
    case 'version'
        profile = read_struct(profilePath);
        profile.version = 99;
        write_struct(profilePath,profile);
        bundle.profile_sha256 = file_hash(profilePath);
    case 'units'
        profile = read_struct(profilePath);
        profile.units = 'rpm';
        write_struct(profilePath,profile);
        bundle.profile_sha256 = file_hash(profilePath);
    case 'identity'
        scenario = read_struct(scenarioPath);
        scenario.profile_id = 'wrong.profile';
        write_struct(scenarioPath,scenario);
        bundle.scenario_sha256 = file_hash(scenarioPath);
    otherwise
        error('FluxRT:Contract:Test','unknown mutation %s',name);
end
write_struct(bundlePath,bundle);
end

function value = read_struct(path)
value = jsondecode(fileread(path));
end

function write_struct(path,value)
text = jsonencode(value,'PrettyPrint',true);
fid = fopen(path,'wb');
assert(fid>=0,'FluxRT:Contract:IO','cannot create %s',path);
cleanup = onCleanup(@()fclose(fid));
fwrite(fid,unicode2native([text newline],'UTF-8'),'uint8');
clear cleanup
end

function hash = file_hash(path)
fid = fopen(path,'rb');
assert(fid>=0,'FluxRT:Contract:IO','cannot open %s',path);
cleanup = onCleanup(@()fclose(fid));
bytes = fread(fid,Inf,'*uint8');
clear cleanup
digest = javaMethod('getInstance','java.security.MessageDigest','SHA-256');
digest.update(typecast(uint8(bytes),'int8'));
raw = typecast(digest.digest(),'uint8');
hash = lower(reshape(dec2hex(raw,2).',1,[]));
end

function remove_tree(path)
if exist(path,'dir'), rmdir(path,'s'); end
end
