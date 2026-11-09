"""Audit-only counterfactual. Wall collision disabled in memory; original XML untouched."""
from pathlib import Path
import sys,json,numpy as np
root=Path(__file__).resolve().parents[3];sys.path[:0]=[str(root/'Code'),str(root/'Code/scripts')]
from ocrl.am3_episode import Episode,initial_state
from ocrl.am3_ctrl_b import BaselineB
from ocrl.am3_pred import AMPred3
from ocrl.am3_task import TASK,BASE
TASK['hover_s']=3.0
x,u=initial_state(.06);p=np.array(AMPred3().pad_f(x)[0]).ravel();wp=[p+(-1)**k*.06*np.array([0,1,.5]) for k in range(12)]
e=Episode(BaselineB(),'free',seed=0,T_max=8,free_waypoints=wp,segment_s=.25,x0=(x,u));e.pl.m.geom_contype[e.pl.gwall]=0;e.pl.m.geom_conaffinity[e.pl.gwall]=0;e.run()
t=np.array([r['t'] for r in e.rows]);X=np.array([r['x'] for r in e.rows]);tilt=np.degrees(2*np.arcsin(np.minimum(1,np.hypot(X[:,4],X[:,5]))));err=np.linalg.norm(X[:,:3]-BASE,axis=1)
old=np.load(root/'Code/results/am3_runs/AM3_BvsN_arm_oscillation_B_loses_control_2026-10-30_B/log.npz');n=min(np.sum(t<3.78),np.sum(old['t']<3.78))
out=dict(abort=e.abort,last_logged_time=float(t[-1]),max_base_error_mm=float(err.max()*1000),max_tilt_deg=float(tilt.max()),peak_pad_wall_force=float(max(e.pl.log['Fn'])),unexpected_contacts=sorted(e.pl.other_names),preimpact_state_max_difference=float(np.max(abs(X[:n]-old['x'][:n]))),preimpact_samples=int(n))
print(json.dumps(out,indent=2));np.savez_compressed(Path(__file__).with_suffix('.npz'),t=t,x=X)
