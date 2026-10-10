"""Solve a relative equilibrium, then separately assess its dynamic stability."""
from pathlib import Path
import sys,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
def perturb(base,x,g):
 y=base.copy();rp=Rot.from_quat(base[3:7])*Rot.from_rotvec(x[:3]);rc=Rot.from_quat(base[16:20])*Rot.from_rotvec(x[3:6]);y[3:7]=rp.as_quat();y[16:20]=rc.as_quat();y[7:10]+=Rot.from_quat(base[3:7]).apply(x[6:9]);y[10:13]+=Rot.from_quat(base[3:7]).apply(x[9:12]);y[23:26]+=Rot.from_quat(base[16:20]).apply(x[12:15]);ap=rp.apply(g['hinge']);ac=rc.apply([0,0,.85]);y[13:16]=y[:3]+ap-ac;y[20:23]=y[7:10]+np.cross(y[10:13],ap)-np.cross(y[23:26],ac);return y

def difference(base,y):
 rp=Rot.from_quat(base[3:7]);rc=Rot.from_quat(base[16:20]);return np.r_[(rp.inv()*Rot.from_quat(y[3:7])).as_rotvec(),(rc.inv()*Rot.from_quat(y[16:20])).as_rotvec(),rp.inv().apply(y[7:10]-base[7:10]),rp.inv().apply(y[10:13]-base[10:13]),rc.inv().apply(y[23:26]-base[23:26])]



def solve(sim,base,command0,climb=0.,turn=0.,voltage=22.2):
 dt=.004;undo=Rot.from_euler('z',-np.deg2rad(turn)*dt);offset=sim.parameters['left_offset']*1000
 def command(x):return np.array([x[-2],max(-x[-1],0)+(offset if x[-1]<0 else 0),max(x[-1],0)])
 def unpack(x):
  y=perturb(base,x[:15],sim.geometry);u=command(x);rpm=743.5778222336925*voltage*max(u[0]-.0005141016498671315,0)**.9086807307118114;return y,u,rpm
 def residual(x):
  y,u,rpm=unpack(x);n,_=sim.step(y,rpm,u,voltage=voltage,dt=dt)
  for sl in [slice(3,7),slice(16,20)]:n[sl]=(undo*Rot.from_quat(n[sl])).as_quat()
  for sl in [slice(7,10),slice(10,13),slice(20,23),slice(23,26)]:n[sl]=undo.apply(n[sl])
  return np.r_[difference(y,n)/dt,(-y[9]-climb),Rot.from_quat(y[3:7]).as_euler('xyz')[2]]
 # Fix arbitrary heading before solving.
 yaw=Rot.from_quat(base[3:7]).as_euler('xyz')[2];rz=Rot.from_euler('z',-yaw);base=base.copy()
 for sl in [slice(3,7),slice(16,20)]:base[sl]=(rz*Rot.from_quat(base[sl])).as_quat()
 for sl in [slice(7,10),slice(10,13),slice(20,23),slice(23,26)]:base[sl]=rz.apply(base[sl])
 signed=command0[2]-max(command0[1]-offset,0);x0=np.r_[np.zeros(15),command0[0],signed];lo=np.r_[np.full(6,-1.),np.full(3,-3.),np.full(6,-3.),0.,-40.];hi=np.r_[np.full(6,1.),np.full(3,3.),np.full(6,3.),.65,40.]
 fit=least_squares(residual,x0,bounds=(lo,hi),x_scale=np.r_[np.ones(15),.1,10.],diff_step=1e-4,max_nfev=100,ftol=1e-10,xtol=1e-10,gtol=1e-9);y,u,rpm=unpack(fit.x)
 return dict(state=y,command=u,rpm=rpm,residual=residual(fit.x),success=bool(fit.success),message=fit.message,nfev=fit.nfev)
