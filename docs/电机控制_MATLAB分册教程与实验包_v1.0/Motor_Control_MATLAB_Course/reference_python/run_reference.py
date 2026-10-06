"""Numerical reference implementation for the accompanying MATLAB course.
These plots are generated in Python, NOT by MATLAB/Simulink.
Requires numpy and matplotlib. No MATLAB runtime is needed for this reference.
Run from any directory: python run_reference.py
"""
from pathlib import Path
import json, csv, math, platform, time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]
FIG=ROOT/'figures'; OUT=ROOT/'reference_results'
FIG.mkdir(exist_ok=True); OUT.mkdir(exist_ok=True)
METRICS={}; PLOTS={}; DATA={}
def timeline(dt,end):return np.arange(round(end/dt)+1)*dt
def plot(n,name,x,y,labels,xlabel,ylabel,title):
    f,ax=plt.subplots(figsize=(10,5.6))
    Y=np.asarray(y);ax.plot(x,Y,linewidth=1.35)
    ax.grid(True,alpha=.3);ax.set(xlabel=xlabel,ylabel=ylabel,title=title)
    if labels:ax.legend(labels,loc='best',fontsize=9)
    if name=='L01_vector':ax.set_aspect('equal',adjustable='box')
    f.text(.5,.015,'Reference computation: Python | MATLAB / Simulink execution not performed',ha='center',fontsize=8)
    f.tight_layout(rect=[0,.035,1,1]);f.savefig(FIG/(name+'.png'),dpi=145);plt.close(f)
    PLOTS.setdefault(n,[]).append(name)
def save(n,data,columns,metrics):
    a=np.asarray(data);assert a.ndim==2 and a.shape[1]==len(columns)
    assert not np.isinf(a).any(),(n,'infinite data')
    np.savetxt(OUT/f'L{n:02d}_data.csv',a,delimiter=',',header=','.join(columns),comments='',fmt='%.12g')
    METRICS[f'L{n:02d}']={k:float(v) for k,v in metrics.items()};DATA[n]=a
    (OUT/f'L{n:02d}_metrics.json').write_text(json.dumps(METRICS[f'L{n:02d}'],indent=2),encoding='utf8')
def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi
def rk4(f,x,h):
    k1=f(x);k2=f(x+h*k1/2);k3=f(x+h*k2/2);k4=f(x+h*k3)
    return x+h*(k1+2*k2+2*k3+k4)/6
R=.2;LD=.0004;LQ=.0004;PSI=.02;PP=4;J=.0002;B=.0001;VDC=48;IMAX=8;TS=.00005;KT=.12

def svpwm(v):
    v=np.array(v,dtype=float);vm=48/np.sqrt(3)
    if np.linalg.norm(v)>vm:v*=vm/np.linalg.norm(v)
    phase=np.array([v[0],-.5*v[0]+np.sqrt(3)/2*v[1],-.5*v[0]-np.sqrt(3)/2*v[1]])
    v0=-(phase.max()+phase.min())/2;d=np.clip(.5+(phase+v0)/48,0,1)
    vp=48*(d-np.mean(d));va=(2/3)*(vp[0]-.5*vp[1]-.5*vp[2]);vb=(vp[1]-vp[2])/np.sqrt(3)
    return d,np.array([va,vb])

def L01():
    t=np.linspace(0,.04,2001);th=2*np.pi*50*t;I=6
    a=I*np.cos(th);b=I*np.cos(th-2*np.pi/3);c=I*np.cos(th+2*np.pi/3)
    al=(2/3)*(a-.5*b-.5*c);be=(b-c)/np.sqrt(3);mag=np.hypot(al,be)
    m={'sum_abc_max':np.max(abs(a+b+c)),'magnitude_error':np.max(abs(mag-I))}
    save(1,np.c_[t,a,b,c,al,be,mag],['time_s','ia_A','ib_A','ic_A','alpha_A','beta_A','magnitude_A'],m)
    plot(1,'L01_abc',t,np.c_[a,b,c],['ia','ib','ic'],'Time (s)','Current (A)','Three-phase currents: 120-degree time shifts')
    plot(1,'L01_vector',al,be,[],'Alpha (A)','Beta (A)','Current space-vector locus (not magnetic flux in tesla)')
    assert m['magnitude_error']<1e-10

def L02():
    h=1e-5;t=timeline(h,.02);i=np.zeros(t.size);w=i.copy();a=np.exp(-R*h/LD);b=(1-a)/R
    for k in range(len(t)-1):i[k+1]=a*i[k]+b*2;w[k+1]=w[k]+h*(KT*i[k]-B*w[k])/J
    exact=2/R*(1-np.exp(-R*t/LD));m={'RL_max_error':np.max(abs(i-exact)),'time_constant_s':LD/R}
    save(2,np.c_[t,i,exact,w],['time_s','current_A','analytic_A','separate_mechanics_rad_s'],m)
    plot(2,'L02_rl',t,np.c_[i,exact],['Exact-ZOH recurrence','Analytical curve'],'Time (s)','Current (A)','RL step response: tau=L/R')
    plot(2,'L02_mechanical',t,w,['Separate mechanics'],'Time (s)','Speed (rad/s)','Independent torque-driven inertia: back EMF not coupled')
    assert m['RL_max_error']<1e-9

def L03():
    th=np.linspace(0,2*np.pi,3601);I=3;e=np.c_[np.cos(th),np.cos(th-2*np.pi/3),np.cos(th+2*np.pi/3)]
    cur=np.zeros_like(e);cur[np.arange(len(th)),e.argmax(axis=1)]=I;cur[np.arange(len(th)),e.argmin(axis=1)]=-I
    isin=2*I/np.sqrt(3)*e;p6=(e*cur).sum(axis=1);ps=(e*isin).sum(axis=1)
    save(3,np.c_[th,cur,p6,ps],['theta_e_rad','ia_A','ib_A','ic_A','sixstep_normalized_power','sine_normalized_power'],{'sixstep_relative_ripple':np.ptp(p6)/np.mean(p6),'sine_relative_ripple':np.ptp(ps)/np.mean(ps)})
    plot(3,'L03_commutation',th*180/np.pi,cur,['ia','ib','ic'],'Electrical angle (deg)','Current (A)','Ideal six-step currents; finite commutation dynamics omitted')
    plot(3,'L03_power',th*180/np.pi,np.c_[p6,ps],['Six-step on sine EMF','Sine currents'],'Electrical angle (deg)','Normalized power','Equal copper-loss proxy, sinusoidal back EMF assumed')

def L04():
    th=np.linspace(0,4*np.pi,2001);al=-6*np.sin(th);be=6*np.cos(th)
    a=al;b=-.5*al+np.sqrt(3)/2*be;c=-.5*al-np.sqrt(3)/2*be;aa=(2/3)*(a-.5*b-.5*c);bb=(b-c)/np.sqrt(3)
    d=aa*np.cos(th)+bb*np.sin(th);q=-aa*np.sin(th)+bb*np.cos(th);dd=aa*np.cos(th+np.pi/12)+bb*np.sin(th+np.pi/12);qq=-aa*np.sin(th+np.pi/12)+bb*np.cos(th+np.pi/12)
    m={'dq_error':max(np.max(abs(d)),np.max(abs(q-6))),'id_offset_mean':np.mean(dd),'iq_offset_mean':np.mean(qq)}
    save(4,np.c_[th,a,b,c,d,q,dd,qq],['theta_e_rad','ia_A','ib_A','ic_A','id_A','iq_A','id_15deg_A','iq_15deg_A'],m)
    plot(4,'L04_dq',th,np.c_[d,q,dd,qq],['id correct','iq correct','id +15deg','iq +15deg'],'Electrical angle (rad)','Current (A)','Fixed angle error rotates dq measurements')
    assert m['dq_error']<1e-10

def L05():
    we=400;h=1e-5;t=timeline(h,.02);X=np.zeros((len(t),2));vd=-we*LQ*4;vq=R*4+we*PSI
    def f(x):return np.array([(vd-R*x[0]+we*LQ*x[1])/LD,(vq-R*x[1]-we*(LD*x[0]+PSI))/LQ])
    for k in range(len(t)-1):X[k+1]=rk4(f,X[k],h)
    te=1.5*PP*(PSI*X[:,1]+(LD-LQ)*X[:,0]*X[:,1]);res=np.zeros(len(t))
    for k,x in enumerate(X):
        dx=f(x);res[k]=1.5*(vd*x[0]+vq*x[1])-1.5*R*np.sum(x*x)-1.5*(LD*x[0]*dx[0]+LQ*x[1]*dx[1])-te[k]*we/PP
    m={'power_residual_max':np.max(abs(res)),'final_iq':X[-1,1]}
    save(5,np.c_[t,X,te,res],['time_s','id_A','iq_A','torque_Nm','power_balance_residual_W'],m)
    plot(5,'L05_dq',t,X,['id','iq'],'Time (s)','Current (A)','Coupled PMSM dynamics at externally imposed constant speed')
    plot(5,'L05_energy',t,res,['Energy-balance residual'],'Time (s)','Residual (W)','pe = copper loss + magnetic-energy rate + Te*wm')
    assert m['power_residual_max']<1e-8

def L06():
    th=np.linspace(0,2*np.pi,2001);A=.98*48/np.sqrt(3);D=[];E=[]
    for v in np.c_[A*np.cos(th),A*np.sin(th)]:d,vr=svpwm(v);D.append(d);E.append(vr-v)
    D=np.array(D);E=np.array(E);tt=timeline(1e-7,150e-6);carrier=1-abs(2*((tt*20000)%1)-1)
    d,_=svpwm(A*np.array([np.cos(.4),np.sin(.4)]));g=(carrier[:,None]<d).astype(float);vab=48*(g[:,0]-g[:,1])
    m={'reconstruction_error':np.max(abs(E)),'duty_min':D.min(),'duty_max':D.max()}
    save(6,np.c_[th,D,E],['theta_e_rad','duty_a','duty_b','duty_c','alpha_error_V','beta_error_V'],m)
    plot(6,'L06_duty',th*180/np.pi,D,['dA','dB','dC'],'Electrical angle (deg)','Duty ratio','Centered zero-sequence injection')
    plot(6,'L06_switching',tt*1e6,vab,['vAB'],'Time (us)','Line voltage (V)','Ideal switching voltage; dead time and parasitics omitted')
    assert m['reconstruction_error']<1e-10

def current(fc,delay,aw,profile):
    t=timeline(TS,.08);i=np.zeros(t.size);u=i.copy();zlog=i.copy();ref=i.copy();z=0;kP=LD*2*np.pi*fc;kI=R*2*np.pi*fc
    a=np.exp(-R*TS/LD);b=(1-a)/R
    for k in range(len(t)-1):
        ref[k]=5*(t[k]>=.002) if profile==1 else 30*(t[k]>=.002 and t[k]<.025)+5*(t[k]>=.025)
        err=ref[k]-i[max(0,k-delay)];un=kP*err+z;u[k]=np.clip(un,-4,4)
        z+=TS*(kI*err+(2*np.pi*fc*(u[k]-un) if aw else 0));zlog[k]=z;i[k+1]=a*i[k]+b*u[k]
    ref[-1]=ref[-2];u[-1]=u[-2];zlog[-1]=zlog[-2]
    return np.c_[t,ref,i,u,zlog]

def L07():
    a=current(200,0,True,1);b=current(800,0,True,1)
    m={'final_error_slow':abs(a[-1,2]-5),'final_error_fast':abs(b[-1,2]-5)}
    save(7,np.c_[a[:,:3],b[:,2],a[:,3],b[:,3]],['time_s','reference_A','i_fc200_A','i_fc800_A','v_fc200_V','v_fc800_V'],m)
    plot(7,'L07_response',a[:,0]*1000,np.c_[a[:,1:3],b[:,2]],['Reference','fc=200Hz','fc=800Hz'],'Time (ms)','Current (A)','PI bandwidth comparison at the same voltage limit')
    plot(7,'L07_voltage',a[:,0]*1000,np.c_[a[:,3],b[:,3]],['fc=200Hz','fc=800Hz'],'Time (ms)','Voltage (V)','Faster tracking requires greater voltage authority')
    assert max(m.values())<1e-4

def L08():
    a=current(500,0,False,2);b=current(500,0,True,2);c=current(800,0,True,1);d=current(800,4,True,1)
    m={'windup_peak':np.max(abs(a[:,4])),'aw_integral_peak':np.max(abs(b[:,4])),'delay4_rms':np.sqrt(np.mean((d[-401:,2]-5)**2))}
    save(8,np.c_[a[:,:3],b[:,2],a[:,4],b[:,4],c[:,2],d[:,2]],['time_s','reference_A','noAW_A','AW_A','integral_noAW_V','integral_AW_V','delay0_A','delay4_A'],m)
    plot(8,'L08_aw',a[:,0]*1000,np.c_[a[:,1:3],b[:,2]],['Reference','No AW','Back-calculation'],'Time (ms)','Current (A)','30 A is unreachable with 4 V and 0.2 ohm')
    plot(8,'L08_integral',a[:,0]*1000,np.c_[a[:,4],b[:,4]],['No AW','AW'],'Time (ms)','Integrator state (V)','Observe stored controller state during saturation')
    plot(8,'L08_delay',c[:,0]*1000,np.c_[c[:,1:3],d[:,2]],['Reference','No extra delay','4-sample delay'],'Time (ms)','Current (A)','Same gains, different feedback delay')

def L09():
    k=np.arange(400);ta=(k+.15)*TS;tb=(k+.5)*TS;truth=lambda t:8*np.sin(2*np.pi*200*t)
    spike=lambda t:4*np.exp(-(((t/TS)%1-.15)/.025)**2)
    quant=lambda x:(np.clip(np.floor((1.65+.005*20*x)/3.3*4096+.5),0,4095)*3.3/4096-1.65)/(.005*20)
    ia=quant(truth(ta)+spike(ta));ib=quant(truth(tb)+spike(tb))
    m={'edge_RMS_error':np.sqrt(np.mean((ia-truth(ta))**2)),'center_RMS_error':np.sqrt(np.mean((ib-truth(tb))**2)),'LSB_A':3.3/4096/.1}
    save(9,np.c_[ta,tb,truth(ta),truth(tb),ia,ib],['edge_sample_time_s','center_sample_time_s','truth_at_edge_A','truth_at_center_A','edge_sample_A','center_sample_A'],m)
    plot(9,'L09_samples',tb,np.c_[truth(tb),ia,ib],['Truth at center','Edge sample (own time)','Center sample'],'Time (s)','Current (A)','A contaminated acquisition cannot be repaired by PI tuning')
    plot(9,'L09_error',tb,np.c_[ia-truth(ta),ib-truth(tb)],['Edge error','Center error'],'Time (s)','Error (A)','Error referenced to each actual sample instant')
    assert m['center_RMS_error']<.01 and m['edge_RMS_error']>3

def L10():
    t=timeline(TS,.08);wm=100*2*np.pi/60;th=6+wm*t;q12=np.round(th/(2*np.pi/4096))*(2*np.pi/4096);q18=np.round(th/(2*np.pi/2**18))*(2*np.pi/2**18)
    w12=np.r_[0,wrap(np.diff(q12))/TS];w18=np.r_[0,wrap(np.diff(q18))/TS];wf=np.zeros(t.size);a=np.exp(-2*np.pi*60*TS)
    for k in range(1,len(t)):wf[k]=a*wf[k-1]+(1-a)*w12[k]
    m={'rms_speed12':np.sqrt(np.mean((w12[1:]-wm)**2)),'rms_speed18':np.sqrt(np.mean((w18[1:]-wm)**2)),'highspeed_delay_deg':7*(6000*2*np.pi/60)*20e-6*180/np.pi}
    save(10,np.c_[t,th%(2*np.pi),q12%(2*np.pi),w12,w18,wf],['time_s','true_angle_rad','angle12_rad','speed12_rad_s','speed18_rad_s','filtered_speed12_rad_s'],m)
    plot(10,'L10_angle',t,q12%(2*np.pi),['Single-turn angle'],'Time (s)','Angle (rad)','A 2-pi wrap is not a physical reverse rotation')
    sel=t<.02;plot(10,'L10_speed',t[sel],np.c_[np.full(sel.sum(),wm),w12[sel],w18[sel],wf[sel]],['True','12-bit','18-bit','12-bit filtered'],'Time (s)','Speed (rad/s)','Position quantization amplified by differentiation')

def rhs(x,v,load):
    c=math.cos(x[3]);s=math.sin(x[3]);vd=c*v[0]+s*v[1];vq=-s*v[0]+c*v[1];we=PP*x[2];te=KT*x[1]
    return np.array([(vd-R*x[0]+we*LQ*x[1])/LD,(vq-R*x[1]-we*(LD*x[0]+PSI))/LQ,(te-load-B*x[2])/J,we])

def foc(mode):
    t=timeline(TS,.3);x=np.zeros(4);z=np.zeros(2);zs=0;iqr=0;wc=2*np.pi*600;kp=np.array([LD,LQ])*wc;ki=R*wc;wn=2*np.pi*20;kps=2*.9*wn*J/KT;kis=wn**2*J/KT;y=np.zeros((len(t),12))
    for k,tk in enumerate(t):
        ref=100*(tk>=.02)+50*(tk>=.2);load=.15*(tk>=.12);off=(mode==1 and tk>=.12)*np.pi/12;bias=(mode==2 and tk>=.12)*.3
        theta=x[3]+off;c=math.cos(x[3]);s=math.sin(x[3]);ab=np.array([c*x[0]-s*x[1],s*x[0]+c*x[1]])
        abc=np.array([ab[0]+bias,-.5*ab[0]+np.sqrt(3)/2*ab[1],-.5*ab[0]-np.sqrt(3)/2*ab[1]])
        abm=np.array([(2/3)*(abc[0]-.5*abc[1]-.5*abc[2]),(abc[1]-abc[2])/np.sqrt(3)])
        cm=math.cos(theta);sm=math.sin(theta);idm=np.array([cm*abm[0]+sm*abm[1],-sm*abm[0]+cm*abm[1]])
        if k%10==0:
            es=ref-x[2];raw=kps*es+zs;iqr=np.clip(raw,-IMAX,IMAX);zs+=10*TS*(kis*es+wn*(iqr-raw))
        er=np.array([0,iqr])-idm;we=PP*x[2];ff=np.array([-we*LQ*idm[1],we*(LD*idm[0]+PSI)])
        un=kp*er+z+ff;vlim=.95*48/np.sqrt(3);usat=un*min(1,vlim/max(np.linalg.norm(un),1e-12));z+=TS*(ki*er+wc*(usat-un))
        thv=theta+we*TS/2;vab=np.array([math.cos(thv)*usat[0]-math.sin(thv)*usat[1],math.sin(thv)*usat[0]+math.cos(thv)*usat[1]])
        duty,va=svpwm(vab);te=KT*x[1];y[k]=[tk,ref,x[2],x[0],x[1],iqr,np.linalg.norm(usat),te,load,*duty]
        if k<len(t)-1:
            for _ in range(2):x=rk4(lambda xx:rhs(xx,va,load),x,TS/2)
        assert np.isfinite(x).all() and np.linalg.norm(x[:2])<100 and abs(x[2])<3000
    return y

def L11():
    a=foc(0);b=foc(1);c=foc(2)
    m={'final_speed_error':abs(a[-1,2]-a[-1,1]),'max_voltage':a[:,6].max(),'max_current':np.hypot(a[:,3],a[:,4]).max(),'offset_final_id':b[-1,3]}
    save(11,np.c_[a,b[:,2:5],c[:,2:5]],['time_s','speed_ref_rad_s','speed_rad_s','id_A','iq_A','iq_ref_A','voltage_norm_V','Te_Nm','load_Nm','dutyA','dutyB','dutyC','offset_speed_rad_s','offset_id_A','offset_iq_A','bias_speed_rad_s','bias_id_A','bias_iq_A'],m)
    plot(11,'L11_speed',a[:,0],a[:,1:3],['Reference','Measured'],'Time (s)','Speed (rad/s)','Full averaged FOC: startup, load step, speed step')
    plot(11,'L11_current',a[:,0],a[:,3:6],['True id','True iq','iq reference'],'Time (s)','Current (A)','Current loops operate inside the speed loop')
    plot(11,'L11_voltage',a[:,0],a[:,6],['Applied dq norm'],'Time (s)','Voltage (V)','Vector-voltage demand with circular saturation')
    plot(11,'L11_fault_effect',a[:,0],np.c_[a[:,3],b[:,3],c[:,3]],['Normal','15-degree encoder error','0.3A phase-A offset'],'Time (s)','True rotor-frame id (A)','Compare fault signatures under the same load profile')
    assert m['final_speed_error']<2 and m['max_voltage']<=.95*48/np.sqrt(3)+1e-8

def L12():
    h=1e-5;t=timeline(h,.08);we=400;th=we*t;I=2;i=np.c_[-I*np.sin(th),I*np.cos(th)];di=np.c_[-I*we*np.cos(th),-I*we*np.sin(th)]
    e=PSI*we*np.c_[-np.sin(th),np.cos(th)];v=R*i+LD*di+e;hat=np.zeros(2);ef=hat.copy();EH=np.zeros_like(i);K=12;boundary=.12;fc=1200;af=np.exp(-2*np.pi*fc*h)
    for k in range(len(t)):
        inj=K*np.tanh((hat-i[k])/boundary);hat+=h*(-R*hat+v[k]-inj)/LD;ef=af*ef+(1-af)*inj;EH[k]=ef
    angle=np.arctan2(-EH[:,0],EH[:,1]);rawerr=wrap(angle-th);err=wrap(angle+np.arctan(we/(2*np.pi*fc))-th)
    noise=np.c_[.08*np.sin(2*np.pi*1700*t),.06*np.cos(2*np.pi*1300*t)];direction=np.c_[-np.sin(th),np.cos(th)];low=.2*direction+noise;high=8*direction+noise
    le=wrap(np.arctan2(-low[:,0],low[:,1])-th);he=wrap(np.arctan2(-high[:,0],high[:,1])-th);sel=t>.03
    m={'smo_comp_rms_deg':np.sqrt(np.mean(err[sel]**2))*180/np.pi,'low_emf_rms_deg':np.sqrt(np.mean(le**2))*180/np.pi,'high_emf_rms_deg':np.sqrt(np.mean(he**2))*180/np.pi}
    save(12,np.c_[t,e,EH,rawerr,err,le,he],['time_s','emf_alpha_V','emf_beta_V','estimated_alpha_V','estimated_beta_V','raw_angle_error_rad','diagnostic_comp_error_rad','low_emf_error_rad','high_emf_error_rad'],m)
    plot(12,'L12_emf',t,np.c_[e[:,0],EH[:,0]],['True e-alpha','SMO + LPF'],'Time (s)','Back EMF (V)','SMO with synthesized current and voltage measurements')
    plot(12,'L12_angle',t,np.c_[rawerr,err]*180/np.pi,['Raw','Known-speed diagnostic compensation'],'Time (s)','Angle error (deg)','Compensation here is diagnostic, not autonomous startup')
    plot(12,'L12_low_speed',t,np.c_[le,he]*180/np.pi,['0.2V EMF','8V EMF'],'Time (s)','Angle error (deg)','Equal voltage disturbance, unequal signal-to-noise ratio')
    assert m['low_emf_rms_deg']>m['high_emf_rms_deg']

IPMR=.2;IPMLD=.0006;IPMLQ=.001;IPMPSI=.012;IPMP=4;IPMIMAX=12

def L13():
    T=np.linspace(0,.8,81);grid=np.linspace(-12,0,4001);D=np.zeros((len(T),4))
    for k,tar in enumerate(T):
        iq=tar/(1.5*4*(.012+(.0006-.001)*grid));I2=grid**2+iq**2;I2[I2>144]=np.inf;j=I2.argmin();assert np.isfinite(I2[j]);D[k]=[grid[j],iq[j],np.sqrt(I2[j]),tar/(1.5*4*.012)]
    m={'max_current_saving_A':np.max(D[:,3]-D[:,2]),'recomputed_torque_error':np.max(abs(1.5*4*(.012+(.0006-.001)*D[:,0])*D[:,1]-T))}
    save(13,np.c_[T,D],['target_torque_Nm','id_MTPA_A','iq_MTPA_A','I_MTPA_A','I_id0_A'],m)
    plot(13,'L13_currents',T,D[:,:2],['id','iq'],'Torque (Nm)','Current (A)','Numerical MTPA search for a salient IPMSM')
    plot(13,'L13_saving',T,D[:,2:4],['MTPA','id=0'],'Torque (Nm)','Current-vector magnitude (A)','Same torque with different current allocation')
    assert m['recomputed_torque_error']<1e-10

def L14():
    ID,IQ=np.meshgrid(np.linspace(-12,0,241),np.linspace(0,12,241),indexing='ij');T=1.5*4*(.012+(.0006-.001)*ID)*IQ;cur=ID**2+IQ**2<=144+1e-9
    wm=np.linspace(0,1200,121);D=np.full((len(wm),5),np.nan);vlim=48/np.sqrt(3)
    for k,w in enumerate(wm):
        we=4*w;vd=.2*ID-we*.001*IQ;vq=.2*IQ+we*(.0006*ID+.012);mask=cur&(np.hypot(vd,vq)<=vlim);tc=np.where(mask,T,-np.inf)
        # MATLAB linear indexing is column-major; use the same tie-breaking order.
        j=tc.ravel(order='F').argmax();ii=np.unravel_index(j,tc.shape,order='F');best=tc[ii]
        if np.isfinite(best):D[k,:4]=[best,ID[ii],IQ[ii],np.hypot(vd[ii],vq[ii])]
        vals=T[mask&(abs(ID)<1e-12)]
        if len(vals):D[k,4]=vals.max()
    m={'current_max':np.nanmax(np.hypot(D[:,1],D[:,2])),'voltage_max':np.nanmax(D[:,3]),'highspeed_torque':D[-1,0]}
    save(14,np.c_[wm,D],['speed_rad_s','max_torque_Nm','id_A','iq_A','voltage_V','max_torque_id0_Nm'],m)
    plot(14,'L14_envelope',wm*60/(2*np.pi),D[:,[0,4]],['Optimized I/V constraints','id=0 only'],'Mechanical speed (rpm)','Feasible torque (Nm)','Steady-state envelope, not a dynamic weakening controller')
    plot(14,'L14_idiq',wm*60/(2*np.pi),D[:,1:3],['id','iq'],'Mechanical speed (rpm)','Current (A)','Current allocation changes at the voltage constraint')
    assert m['current_max']<=12+1e-8 and m['voltage_max']<=vlim+1e-8

def servo(ff):
    h=.0002;t=timeline(h,1.2);theta=w=torque=z=0.;y=np.zeros((len(t),7))
    for k,tk in enumerate(t):
        u=np.clip((tk-.05)/.3,0,1);r=10*u**3-15*u**4+6*u**5;dr=(30*u**2-60*u**3+30*u**4)/.3;ddr=(60*u-180*u**2+120*u**3)/.3**2
        wr=np.clip(8*(r-theta)+ff*dr,-20,20);err=wr-w;raw=.8*err+z+ff*(.01*ddr+.03*dr);cmd=np.clip(raw,-1,1);z+=h*(10*err+40*(cmd-raw));load=.2*(tk>=.6)
        y[k]=[tk,r,theta,w,cmd,torque,load];torque+=h*(cmd-torque)/.001;w+=h*(torque-load-.03*w)/.01;theta+=h*w
    return y

def L15():
    a=servo(0);b=servo(1);sel=(a[:,0]>.05)&(a[:,0]<.35)
    m={'tracking_RMS_noFF':np.sqrt(np.mean((a[sel,2]-a[sel,1])**2)),'tracking_RMS_FF':np.sqrt(np.mean((b[sel,2]-b[sel,1])**2))}
    save(15,np.c_[a,b[:,2:6]],['time_s','position_ref_rad','position_noFF_rad','speed_noFF_rad_s','torque_cmd_noFF_Nm','torque_noFF_Nm','load_Nm','position_FF_rad','speed_FF_rad_s','torque_cmd_FF_Nm','torque_FF_Nm'],m)
    plot(15,'L15_position',a[:,0],np.c_[a[:,1:3],b[:,2]],['Reference','No FF','Velocity/acceleration FF'],'Time (s)','Position (rad)','Position-speed cascade, first-order torque actuator')
    plot(15,'L15_error',a[:,0],np.c_[a[:,2]-a[:,1],b[:,2]-b[:,1]],['No FF','With FF'],'Time (s)','Tracking error (rad)','Feedforward assists tracking; feedback rejects unknown load')

def L16():
    jm=.002;jl=.01;K=40;c=.01;bm=.002;bl=.003;f=np.logspace(-1,3,1200);fr=np.sqrt(K*(1/jm+1/jl))/(2*np.pi);wn=2*np.pi*fr;G=np.zeros(f.size,complex);H=G.copy()
    for k,fk in enumerate(f):
        s=1j*2*np.pi*fk;A=np.array([[jm*s*s+(bm+c)*s+K,-c*s-K],[-c*s-K,jl*s*s+(bl+c)*s+K]]);x=np.linalg.solve(A,[1,0]);G[k]=s*x[0];H[k]=(s*s+2*.04*wn*s+wn**2)/(s*s+2*.4*wn*s+wn**2)
    dat=np.c_[f,20*np.log10(abs(G)),20*np.log10(abs(G*H)),np.unwrap(np.angle(G))*180/np.pi]
    save(16,dat,['frequency_Hz','plant_dB','input_notched_dB','plant_phase_deg'],{'resonance_undamped_Hz':fr,'antiresonance_undamped_Hz':np.sqrt(K/jl)/(2*np.pi)})
    plot(16,'L16_resonance',np.log10(f),dat[:,1:3],['Plant','Plant times input notch'],'log10 frequency (Hz)','Magnitude (dB re 1 (rad/s)/Nm)','Two-inertia FRF: input filtering is not closed-loop stability')

def L17():
    h=.0001;t=timeline(h,.8);Y=np.zeros((len(t),4));wall=.6
    for j,stiff in enumerate([5,30]):
        theta=w=0.
        for k in range(len(t)):
            cmd=np.clip(stiff*(1-theta)-w,-8,8);contact=max(0,150*(theta-wall)+.4*w) if theta>wall else 0
            Y[k,2*j:2*j+2]=[theta,contact];w+=h*(cmd-contact-.02*w)/.02;theta+=h*w
    save(17,np.c_[t,Y],['time_s','soft_position_rad','soft_contact_Nm','stiff_position_rad','stiff_contact_Nm'],{'peak_contact_soft':Y[:,1].max(),'peak_contact_stiff':Y[:,3].max()})
    plot(17,'L17_position',t,np.c_[Y[:,0],Y[:,2],np.full(t.size,wall)],['K=5','K=30','Wall angle'],'Time (s)','Angle (rad)','Ideal torque control with unilateral compliant contact')
    plot(17,'L17_contact',t,Y[:,[1,3]],['Soft','Stiff'],'Time (s)','Contact torque (Nm)','Stiffness changes contact loads; no real robot safety claim')

def L18():
    h=.0001;t=timeline(h,.4);j=.0002;w0=300;tb=.25;c=.0022;v0=48;rb=20;w=w0*np.maximum(0,1-t/tb);preg=j*w*w0/tb;ea=eb=.5*c*v0*v0;on=False;diss=0;D=np.zeros((len(t),5))
    for k in range(len(t)):
        va=np.sqrt(2*ea/c);vb=np.sqrt(2*eb/c)
        if vb>52:on=True
        elif vb<50:on=False
        pbr=on*vb*vb/rb;D[k]=[va,vb,preg[k],pbr,diss]
        if k<len(t)-1:ea+=h*preg[k];eb+=h*(preg[k]-pbr);diss+=h*pbr
    m={'analytic_final_no_sink':np.sqrt(v0*v0+j*w0*w0/c),'numeric_final_no_sink':D[-1,0],'max_braked_voltage':D[:,1].max(),'initial_mechanical_J':.5*j*w0*w0}
    save(18,np.c_[t,w,D],['time_s','speed_rad_s','bus_no_sink_V','bus_brake_V','regen_W','brake_W','dissipated_J'],m)
    plot(18,'L18_bus',t,D[:,:2],['No sink','Hysteretic brake resistor'],'Time (s)','Bus voltage (V)','Capacitor-energy model during prescribed deceleration')
    plot(18,'L18_power',t,D[:,2:4],['Regeneration','Resistor pulse power'],'Time (s)','Power (W)','Pulse power and total dissipated energy are distinct ratings')
    assert abs(m['numeric_final_no_sink']-m['analytic_final_no_sink'])<.1 and m['max_braked_voltage']<54

def L19():
    h=.001;t=timeline(h,.32);N=len(t);state=0;D=np.zeros((N,5));start=np.zeros(N,bool);reset=start.copy();start[[50,200]]=True;reset[[130,180]]=True
    for k,tk in enumerate(t):
        oc=tk>=.12 and tk<.14;lost=tk>=.25;active=oc or lost
        if active:state=4
        elif state==4:
            if reset[k]:state=2
        elif state==0:state=1
        elif state==1:
            if tk>=.02:state=2
        elif state==2 and start[k]:state=3
        permit=state==3 and not active;D[k]=[state,permit,oc,lost,reset[k]];assert not(active and permit)
    m={'unsafe_permission_count':np.sum(((D[:,2]>0)|(D[:,3]>0))&(D[:,1]>0))}
    save(19,np.c_[t,D],['time_s','state_0init_1cal_2ready_3run_4fault','torque_permission','overcurrent','communication_loss','reset_event'],m)
    plot(19,'L19_states',t,D[:,:2],['State code','Torque permission'],'Time (s)','Code / boolean','Fault latch and explicit restart requirements')
    assert m['unsafe_permission_count']==0 and D[-1,0]==4

def L20():
    h=TS;t=timeline(h,.12);u=2*(2*(np.floor(t/.001)%2)-1)+.5*np.sin(2*np.pi*131*t);i=np.zeros(t.size);a=np.exp(-R*h/LD);b=(1-a)/R
    for k in range(len(t)-1):i[k+1]=a*i[k]+b*u[k]
    im=i+.003*np.sin(2*np.pi*1573*t)+.002*np.cos(2*np.pi*2331*t);phi=np.c_[im[:-1],u[:-1]];ae,be=np.linalg.lstsq(phi,im[1:],rcond=None)[0];assert 0<ae<1 and be>0
    rest=(1-ae)/be;lest=-rest*h/np.log(ae);ip=np.zeros(t.size)
    for k in range(len(t)-1):ip[k+1]=ae*ip[k]+be*u[k]
    m={'R_est_Ohm':rest,'L_est_H':lest,'R_relative_error':abs(rest-R)/R,'L_relative_error':abs(lest-LD)/LD,'condition_Phi':np.linalg.cond(phi)}
    save(20,np.c_[t,u,i,im,ip],['time_s','voltage_V','true_current_A','measured_current_A','fitted_model_current_A'],m)
    plot(20,'L20_fit',t,np.c_[im,ip],['Synthetic measurement','Identified model'],'Time (s)','Current (A)','Locked-rotor RL identification from persistently excited data')
    plot(20,'L20_residual',t,im-ip,['Residual'],'Time (s)','Current (A)','A small fitting residual is necessary, not sufficient validation')
    assert m['R_relative_error']<.05 and m['L_relative_error']<.05

def L21():
    axes=np.arange(1,9);worst=axes*8+6+4;util=worst/50;can=axes*2*1000*150/1e6
    save(21,np.c_[axes,worst,util,can],['axes','budget_us','CPU_period_fraction','assumed_CAN_bus_fraction'],{'four_axis_budget_us':worst[3],'four_axis_CAN_fraction':can[3]})
    plot(21,'L21_budget',axes,np.c_[util,can,np.ones(8)],['CPU period budget','Assumed CAN occupancy','100%'],'Number of axes','Fraction','Separate compute and communication feasibility checks')

def L22():
    th=np.linspace(-np.pi,np.pi,4001);x=.9*np.cos(th);y=.9*np.sin(th)
    # Away-from-zero rounding aligns with MATLAB round for this quantizer.
    quant=lambda v:np.clip(np.sign(v)*np.floor(abs(v)*32768+.5),-32768,32767)
    xq=quant(x);yq=quant(y);cq=quant(np.cos(th));sq=quant(np.sin(th));dq=(xq*cq+yq*sq)/32768**2;qq=(-xq*sq+yq*cq)/32768**2;dt=x*np.cos(th)+y*np.sin(th);qt=-x*np.sin(th)+y*np.cos(th)
    m={'d_max_error':np.max(abs(dq-dt)),'q_max_error':np.max(abs(qq-qt))}
    save(22,np.c_[th,dt,dq,qt,qq],['theta_rad','d_float','d_Q15_inputs','q_float','q_Q15_inputs'],m)
    plot(22,'L22_quantization',th,np.c_[dq-dt,qq-qt],['d error','q error'],'Angle (rad)','Normalized error','Q15 inputs, wide exact multiply/accumulate simulation')
    assert max(m.values())<2e-4

def L23():
    slip=np.linspace(-1,1,2001);sm=.2;torque=2*slip*sm/(slip**2+sm**2);nr=50;micro=16;cmd=np.arange(micro+1)*(np.pi/2/micro)/nr;position=cmd-np.arcsin(.3)/nr
    m={'microstep_mechanical_deg':np.pi/2/micro/nr*180/np.pi,'static_load_error_deg':np.arcsin(.3)/nr*180/np.pi}
    save(23,np.c_[slip,torque],['slip','normalized_induction_torque'],m)
    plot(23,'L23_induction',slip,torque,['Kloss-type approximation'],'Slip','Normalized torque','Qualitative induction-machine torque-slip characteristic')
    plot(23,'L23_stepper',cmd*180/np.pi,np.c_[cmd,position]*180/np.pi,['Command','Loaded equilibrium'],'Command (mechanical deg)','Position (mechanical deg)','Microstep size is not loaded positioning accuracy')

def L24():
    h=.02;t=timeline(h,180);tj=25.;D=np.zeros((len(t),5))
    for k,tk in enumerate(t):
        irms=20+20*(tk>=60);pcond=irms**2*.004*(1+.006*(tj-25));psw=.5*48*irms*60e-9*20000;pgate=30e-9*10*20000;D[k]=[tj,pcond,psw,pgate,irms];tj+=h*(pcond+psw-(tj-25)/8)/8
    save(24,np.c_[t,D],['time_s','junction_C','conduction_W','switching_W','gate_drive_W','device_rms_A'],{'final_temperature_C':D[-1,0],'gate_drive_W':D[-1,3]})
    plot(24,'L24_temperature',t,D[:,0],['One-pole thermal estimate'],'Time (s)','Temperature (degC)','Per-device RMS current 20A to 40A; assumed thermal circuit')
    plot(24,'L24_losses',t,D[:,1:4],['Conduction','Switching overlap','Gate-drive supply'],'Time (s)','Power (W)','Gate-drive power is NOT added to MOSFET junction heating')

def structural_tests():
    assert abs(wrap(2*np.pi))<1e-12 and wrap(np.pi)==-np.pi
    count=0
    for th in np.linspace(-np.pi,np.pi,101):
        P=np.array([[np.cos(th),np.sin(th)],[-np.sin(th),np.cos(th)]]);assert np.linalg.norm(P.T@P-np.eye(2))<1e-12;count+=1
        for amp in [0,5,20,48/np.sqrt(3)]:
            v=amp*np.array([np.cos(th),np.sin(th)]);d,r=svpwm(v);assert np.all((d>=0)&(d<=1)) and np.linalg.norm(v-r)<1e-10;count+=1
    return count+2

if __name__=='__main__':
    started=time.time();results=[]
    for n in range(1,25):
        try:globals()[f'L{n:02d}']();results.append({'lab':f'L{n:02d}','status':'PASS'});print(f'L{n:02d} PASS',flush=True)
        except Exception as exc:results.append({'lab':f'L{n:02d}','status':'FAIL','error':repr(exc)});print(f'L{n:02d} FAIL {exc}',flush=True);raise
    nchecks=structural_tests()
    (OUT/'metrics_all.json').write_text(json.dumps(METRICS,indent=2),encoding='utf8')
    (OUT/'plot_manifest.json').write_text(json.dumps(PLOTS,indent=2),encoding='utf8')
    report={'engine':'Python reference only','python':platform.python_version(),'numpy':np.__version__,'matplotlib':matplotlib.__version__,'matlab_executed':False,'simulink_executed':False,'cross_language_numeric_equivalence_proven':False,'experiments':results,'math_assertion_cases':nchecks,'figure_count':sum(map(len,PLOTS.values())),'elapsed_s':time.time()-started}
    (OUT/'validation_report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))
