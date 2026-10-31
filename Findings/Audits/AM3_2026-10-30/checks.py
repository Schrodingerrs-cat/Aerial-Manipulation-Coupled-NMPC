"""Audit-only checks. No controller builds, training or project-result overwrites."""
from pathlib import Path
import sys,json,hashlib,copy,ast
import numpy as np
ROOT=Path(__file__).resolve().parents[3]; CODE=ROOT/'Code'
sys.path[:0]=[str(CODE),str(CODE/'scripts')]
from ocrl.am3_episode import initial_state,Episode
from ocrl.am3_ctrl_b import BaselineB
from ocrl.am3_pred import AMPred3
from ocrl.am3_task import Task
from ocrl.am3_plant import AMPlant3,KF
from ocrl.am3_eval import passes
import mujoco
out={}
files=list((CODE/'ocrl').glob('am3_*.py'))+list((CODE/'scripts').glob('am3_*.py'))+list((CODE/'config').glob('am3_*.json'))+list((CODE/'model').glob('*.xml'))
out['snapshot']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
f=json.load(open(CODE/'config/am3_freeze_v1.json'))
out['freeze_changed']=[p for p,h in f['files'].items() if hashlib.sha256((CODE/p).read_bytes()).hexdigest()!=h]
out['syntax_errors']=[]
for p in files:
 if p.suffix=='.py':
  try:ast.parse(p.read_text())
  except SyntaxError as e:out['syntax_errors'].append([str(p),str(e)])
x,u=initial_state(); pr=AMPred3();pad=np.array(pr.pad_f(x)[0]).ravel()
# No commanded arm motion: constant EE reference, noisy closed-loop MuJoCo.
e=Episode(BaselineB(),'audit_static',free_waypoints=[pad],segment_s=30,T_max=10,sensors=True,seed=0).run()
out['stationary_B']=[]
for a,b in [(0,1),(1,3),(3,6),(6,10)]:
 X=np.array([r['x'] for r in e.rows if a<=r['t']<b]);out['stationary_B'].append(dict(window=[a,b],tilt_deg=float(np.degrees(2*np.arcsin(np.minimum(1,np.hypot(X[:,4],X[:,5])))).max()),base_mm=float(np.linalg.norm(X[:,:3]-x[:3],axis=1).max()*1000)))
s=(CODE/'scripts/am3_b_linear.py').read_text().split('if __name__')[0]; ns={'__file__':str(CODE/'scripts/am3_b_linear.py'),'__name__':'audit'};exec(s,ns)
r=ns['eigs']({});out['B_linear']={'radius':float(r[0]),'largest':r[1].tolist(),'equilibrium_residual':float(r[2])}
# Reuse independent static checker without its output-writing tail.
s=(CODE/'scripts/am3_check_pred.py').read_text().split('os.makedirs(os.path.join(HERE')[0];ns={'__file__':str(CODE/'scripts/am3_check_pred.py')};exec(s,ns);out['predictor_checks']={k:(v.item() if isinstance(v,np.generic) else v) for k,v in ns['res'].items()}
# Independent whole-assembly Newton/Euler balance from native MuJoCo body Jacobians at zero velocity.
out['newton_euler']=[]
for wall,jointstep in [(False,0),(False,.05),(True,0)]:
 p=AMPlant3();z=x.copy()
 if wall:z[0]+=.061
 p.set_state(z);m,d=p.m,p.d;d.ctrl[:4]=KF*z[19:23]**2;d.ctrl[4:]=u[4:]+np.array([0,jointstep,0]);mujoco.mj_forward(m,d)
 mass=m.body_mass.sum();com=d.subtree_com[1].copy();Fa=np.zeros(3);Ha=np.zeros(3)
 for i in range(1,m.nbody):
  Jv=np.zeros((3,m.nv));Jw=Jv.copy();mujoco.mj_jacBodyCom(m,d,Jv,Jw,i);ai=Jv@d.qacc;al=Jw@d.qacc;Ri=d.ximat[i].reshape(3,3);Iw=Ri@np.diag(m.body_inertia[i])@Ri.T
  Fa+=m.body_mass[i]*ai;Ha+=Iw@al+np.cross(d.xipos[i]-com,m.body_mass[i]*ai)
 R=d.xmat[1].reshape(3,3);Fext=mass*m.opt.gravity.copy();Mext=np.zeros(3)
 for j in range(4):
  sid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,f'rotor{j}');fj=R[:,2]*d.ctrl[j];Fext+=fj;Mext+=np.cross(d.site_xpos[sid]-com,fj)+R[:,2]*pr.P['spin_reaction_sign'][j]*pr.P['km_over_kf_m']*d.ctrl[j]
 for i in range(d.ncon):
  cc=d.contact[i]
  if {cc.geom1,cc.geom2}!={p.gpad,p.gwall}:continue
  f6=np.zeros(6);mujoco.mj_contactForce(m,d,i,f6);fw=cc.frame.reshape(3,3).T@f6[:3];fw*=(-1 if cc.geom1==p.gpad else 1);Fext+=fw;Mext+=np.cross(cc.pos-com,fw)
 out['newton_euler'].append(dict(wall=wall,shoulder_torque_increment=jointstep,force_balance_error_N=float(np.linalg.norm(Fa-Fext)),moment_balance_error_Nm=float(np.linalg.norm(Ha-Mext)),COM_accel=(Fa/mass).tolist(),base_accel=d.qacc[:3].tolist(),body_angular_accel=d.qacc[3:6].tolist(),wall_force=p.pad_force()[0],other_contacts=p.other_contacts()))
# Same count thresholds have different durations.
out['hysteresis']={}
for dt in [.005,.02]:
 t=Task('line',pad);n=0
 while not t.contact:n+=1;t.advance(dt,1,0)
 acquired=n*dt;n=0
 while t.contact:n+=1;t.advance(dt,0,.01)
 out['hysteresis'][str(dt)]=dict(acquisition_s=acquired,loss_s=n*dt)
# Evaluator loophole: a completed free task with huge EE error.
m=json.load(open(CODE/'results/am3_runs/GATE_v1_N/table.json'))
free=next(copy.deepcopy(a) for a in m if a['path']=='free');free['pad_vs_reference']={'rms_mm':1000,'max_mm':2000}
crit=json.load(open(CODE/'config/am3_acceptance_v1.json'))['criteria'];out['free_bad_tracking_passes']=passes(free,crit)
# A marker controller proves which command is used over each logged interval.
class Marker:
 period=.02;name='audit'
 def reset(self,x0,u0,task):self.u=u0.copy();self.n=0
 def compute(self,t,xm,Fm,task):self.n+=1;v=self.u.copy();v[4]+=.001*self.n;return v,{}
e=Episode(Marker(),'audit_marker',free_waypoints=[pad],segment_s=30,T_max=.061,sensors=False);used=[];step=e.pl.step
def spy(ui):
 if not used or e.pl.d.time-used[-1][0]>.01999:used.append((float(e.pl.d.time),float(ui[4])))
 step(ui)
e.pl.step=spy;e.run();out['logged_vs_interval_input']=[dict(t=r['t'],logged=float(r['u_app'][4]),actual=uu[1]) for r,uu in zip(e.rows,used)]
# Compare rate-integrated reference derivative to advertised derivative mid ramp.
t=Task('line',pad);t.phase='trace';t.contact=True;t.rho=.3;t.tau_p=.5;t.n_off=0
h=1e-6;a=t.ref();t.advance(h,1,0);b=t.ref();out['reference_derivative']={'advertised':a['v'].tolist(),'finite_difference':((b['p']-a['p'])/h).tolist(),'rho':.3}
# Re-score persisted gates under the actual frozen acceptance.
out['gate_rescore']={}
for c in ['N','B']:
 rows=json.load(open(CODE/f'results/am3_runs/GATE_v1_{c}/table.json'));out['gate_rescore'][c]={'n':len(rows),'stubs':sum(bool(r.get('stub')) for r in rows),'passes':sum(passes(r,crit)['all'] for r in rows if not r.get('stub'))}
print(json.dumps(out,indent=2))
