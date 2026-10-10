"""Numerical airborne trim utility. Reports residuals; does not certify flight capability."""
import numpy as np
from scipy.optimize import least_squares

def settle_trim(sim,climb_m_s=0.,turn_deg_s=0.,voltage=22.2,seconds=180.):
 if abs(turn_deg_s)>20 or abs(climb_m_s)>1:raise ValueError('This utility is scoped to ordinary flight: |turn|<=20 deg/s, |climb|<=1m/s')
 dt=.04;n=round(seconds/dt)+1;history=[]
 def command(x):
  throttle,brake=x
  return np.array([throttle,max(-brake,0)+(sim.parameters['left_offset']*1000 if brake<0 else 0),max(brake,0)])
 def run(x):
  u=command(x);y,r=sim.replay(sim.initial_state(),np.tile(u,(n,1)),voltage=voltage);v=y[:,7:10];rate=np.rad2deg(np.gradient(np.unwrap(np.arctan2(v[:,1],v[:,0])),dt));a=np.c_[-v[:,2],rate,np.linalg.norm(v[:,:2],axis=1)];mean=a[-501:].mean(axis=0);return y,r,a,mean
 def objective(x):
  y,r,a,m=run(x);err=np.array([m[0]-climb_m_s,(m[1]-turn_deg_s)/10]);history.append(dict(throttle=float(x[0]),effective_differential_mm=float(x[1]),climb=float(m[0]),turn=float(m[1])));return err
 fit=least_squares(objective,[.43,np.clip(turn_deg_s*.7+3,-30,30)],bounds=([0,-40],[.65,40]),diff_step=.002,max_nfev=18,ftol=1e-5,xtol=1e-5,gtol=1e-5);y,r,a,m=run(fit.x);state=y[-1].copy();origin=state[:3].copy();state[:3]-=origin;state[13:16]-=origin
 return dict(state=state,rpm=float(r[-1]),command=command(fit.x),achieved_climb=float(m[0]),achieved_turn=float(m[1]),horizontal_speed=float(m[2]),last20_std=a[-501:].std(axis=0),last10_minus_previous10=a[-250:].mean(axis=0)-a[-500:-250].mean(axis=0),settling_seconds=float(seconds),solver_message=fit.message,history=history)



def find_trim(sim,climb_m_s=0.,turn_deg_s=0.,voltage=22.2,seconds=75.):
 """Seed by integration, solve a relative equilibrium, then report continuation drift.
 A small equilibrium residual is not a stability certificate; perturb separately.
 """
 from equilibrium import solve
 seed=settle_trim(sim,climb_m_s,turn_deg_s,voltage,seconds)
 root=solve(sim,seed['state'],seed['command'],climb_m_s,turn_deg_s,voltage)
 if not np.isfinite(root['residual']).all() or abs(root['residual']).max()>1e-6:raise RuntimeError('Equilibrium solver did not balance the model; inspect the seed and requested operating point')
 y,r=sim.replay(root['state'],np.tile(root['command'],(1501,1)),voltage=voltage,rpm=root['rpm'])
 rate=np.rad2deg(np.gradient(np.unwrap(np.arctan2(y[:,8],y[:,7])),.04))
 a=np.c_[-y[:,9],rate,np.linalg.norm(y[:,7:9],axis=1)]
 state=root['state'].copy();origin=state[:3].copy();state[:3]-=origin;state[13:16]-=origin
 return dict(state=state,rpm=root['rpm'],command=root['command'],achieved_climb=float(a[-501:,0].mean()),achieved_turn=float(a[-501:,1].mean()),horizontal_speed=float(a[-501:,2].mean()),last20_std=a[-501:].std(axis=0),last10_minus_previous10=a[-250:].mean(axis=0)-a[-500:-250].mean(axis=0),equilibrium_residual_max=float(abs(root['residual']).max()),equilibrium_solver_success=root['success'],solver_message=root['message'],seed_settling_seconds=float(seconds),continuation_seconds=60.,history=seed['history'])
