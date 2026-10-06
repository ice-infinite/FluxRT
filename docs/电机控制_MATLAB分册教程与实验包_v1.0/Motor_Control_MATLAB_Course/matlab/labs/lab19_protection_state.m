function r = lab19_protection_state(outdir)
% 教学数值实验，参数不是实测电机数据。仅依赖基础 MATLAB。
% 详见 docs 分册中的模型假设、修改任务与验证边界。
if nargin < 1, outdir = fullfile(mc_root(),'results_matlab','L19'); end
if ~exist(outdir,'dir'), mkdir(outdir); end

h=.001;t=(0:h:.32)';N=numel(t);state=0;D=zeros(N,5);
start=false(N,1);start(round([.05,.2]/h)+1)=true;
reset=false(N,1);reset(round([.13,.18]/h)+1)=true;
for k=1:N
 oc=t(k)>=.12 && t(k)<.14;lost=t(k)>=.25;active=oc||lost;
 if active
  state=4; % 故障优先于同周期start/reset，且锁存。
 elseif state==4
  if reset(k),state=2;end
 elseif state==0
  state=1;
 elseif state==1
  if t(k)>=.02,state=2;end
 elseif state==2 && start(k)
  state=3;
 end
 permit=(state==3)&&~active;D(k,:)=[state,permit,oc,lost,reset(k)];
 assert(~(active && permit));
end
r.data=[t,D];r.columns={'time_s','state_0init_1cal_2ready_3run_4fault','torque_permission','overcurrent','communication_loss','reset_event'};
r.metrics.unsafe_permission_count=sum((D(:,3)|D(:,4)) & D(:,2));
mc_plot(outdir,'L19_states',t,D(:,1:2),{'State code','Torque permission'},'Time (s)','Code / boolean','Fault latch: early reset rejected, cleared reset returns READY');
assert(r.metrics.unsafe_permission_count==0 && D(end,1)==4);

mc_save(outdir, 'L19', r);
end
