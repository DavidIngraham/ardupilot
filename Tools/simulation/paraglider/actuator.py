"""Diagnostic travel-rate bound; no fitted lag or claimed load model."""
import numpy as np
class BrakeActuator:
 def __init__(self,rate_mm_s=221.522558906,position_mm=(0.,0.)):
  if not np.isfinite(rate_mm_s) or rate_mm_s<=0:raise ValueError('positive travel rate required')
  self.rate=float(rate_mm_s);self.position=np.asarray(position_mm,dtype=float).copy()
  if self.position.shape!=(2,) or not np.isfinite(self.position).all():raise ValueError('two finite servo positions required')
 def advance(self,target_mm,dt):
  target=np.asarray(target_mm,dtype=float)
  if target.shape!=(2,) or not np.isfinite(target).all() or not np.isfinite(dt) or dt<=0:raise ValueError('two finite targets and positive dt required')
  end=self.position+np.clip(target-self.position,-self.rate*dt,self.rate*dt);distance=abs(end-self.position);travel_time=distance/self.rate;average=np.where(travel_time>0,.5*(self.position+end)*travel_time/dt+end*(1-travel_time/dt),end);self.position=end;return average

def limited_commands(commands,dt=.04,rate_mm_s=221.522558906):
 commands=np.asarray(commands,float);result=commands.copy();actuator=BrakeActuator(rate_mm_s,commands[0,1:])
 for i in range(len(commands)-1):result[i,1:]=actuator.advance(commands[i,1:],dt)
 result[-1,1:]=actuator.position
 return result
