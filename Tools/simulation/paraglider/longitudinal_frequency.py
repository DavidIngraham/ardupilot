"""Fixed-brake throttle frequency response, with nonlinear sine validation."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from profiles import make_simulator
from trim import find_trim
from equilibrium import perturb,difference
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
sim=make_simulator('trout_lake','mission');sim.substeps=4
trim=find_trim(sim);base=trim['state'];rpm=trim['rpm'];u=trim['command'];dt=.005

def obs(y):
 return np.array([R.from_quat(y[3:7]).inv().apply(y[10:13])[1],-y[9]])

def advance(x,du=0):
 y=perturb(base,x[:15],sim.geometry);command=u.copy();command[0]+=du
 yn,rn=sim.step(y,rpm+x[15],command,dt=dt)
 return np.r_[difference(base,yn),rn-rpm]

A=np.empty((16,16));C=np.zeros((2,16))
for j in range(16):
 e=np.zeros(16);e[j]=1 if j==15 else 1e-5
 A[:,j]=(advance(e)-advance(-e))/(2*e[j])
 C[:,j]=(obs(perturb(base,e[:15],sim.geometry))-obs(perturb(base,-e[:15],sim.geometry)))/(2*e[j])
B=(advance(np.zeros(16),1e-5)-advance(np.zeros(16),-1e-5))/(2e-5)
f=np.geomspace(.02,8,400)
H=np.array([C@np.linalg.solve(np.exp(2j*np.pi*ff*dt)*np.eye(16)-A,B) for ff in f])
checks=[]
for ff in [.1,.333,1.,1.5,3.]:
 for amplitude in [.001,.0005]:
  duration=max(100,30/ff);t=np.arange(0,duration+dt/2,dt)
  commands=np.tile(u,(len(t),1));commands[:,0]+=amplitude*np.sin(2*np.pi*ff*t)
  y,r=sim.replay(base,commands,rpm=rpm,dt=dt)
  q=R.from_quat(y[:,3:7]).inv().apply(y[:,10:13])[:,1];climb=-y[:,9]
  mask=t>duration/2;tt=t[mask];X=np.c_[np.sin(2*np.pi*ff*tt),np.cos(2*np.pi*ff*tt),np.ones(len(tt)),tt-tt.mean()]
  fit=np.linalg.lstsq(X,np.c_[q[mask],climb[mask]],rcond=None)[0]
  response=(fit[0]+1j*fit[1])/amplitude
  predicted=C@np.linalg.solve(np.exp(2j*np.pi*ff*dt)*np.eye(16)-A,B)
  checks.append(dict(frequency_hz=ff,amplitude=amplitude,gain=np.abs(response).tolist(),phase_deg=np.angle(response,deg=True).tolist(),relative_complex_error=(np.abs(response-predicted)/np.maximum(np.abs(predicted),1e-9)).tolist()))
  print(checks[-1],flush=True)
  if amplitude==.001:np.savez_compressed(a.output/f'sine-{ff}.npz',time=t,q=q,climb=climb,throttle=commands[:,0])
fig,axes=plt.subplots(2,2,figsize=(12,8),sharex=True)
for j,title in enumerate(['Body pitch rate / throttle (rad/s per fraction)','Climb / throttle (m/s per fraction)']):
 axes[0,j].loglog(f,np.abs(H[:,j]));axes[0,j].set_title(title)
 axes[1,j].semilogx(f,np.angle(H[:,j],deg=True));axes[1,j].set_ylabel('Phase (deg)');axes[1,j].set_xlabel('Frequency (Hz)')
 for check in checks:
  if check['amplitude']==.001:
   axes[0,j].plot(check['frequency_hz'],check['gain'][j],'o',color='tab:orange')
   axes[1,j].plot(check['frequency_hz'],check['phase_deg'][j],'o',color='tab:orange')
 for ax in axes[:,j]:ax.grid(True,which='both',alpha=.25)
fig.suptitle('Trout Lake level trim, fixed physical brakes — line: local model; dots: nonlinear sine tests')
fig.tight_layout();fig.savefig(a.output/'frequency-response.png',dpi=150)
eig=np.linalg.eigvals(A);continuous=np.log(eig.astype(complex))/dt
result=dict(trim_command=u.tolist(),rpm=float(rpm),dt=dt,equilibrium_step_residual=float(np.linalg.norm(advance(np.zeros(16)))),modes=[dict(real_per_s=float(z.real),frequency_hz=float(z.imag/(2*np.pi))) for z in continuous],validation=checks)
(a.output/'frequency-summary.json').write_text(json.dumps(result,indent=2));np.savez(a.output/'linearization.npz',A=A,B=B,C=C,frequency=f,response=H)
