"""Two-body airborne research model. SI units, world NED, body FRD, xyzw quaternions."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
import dynamics
from geometry import configuration
ROOT=Path(__file__).parent
class Simulator:
 def __init__(self,flight='trout_lake',yaw=None,gyro_inertia=None,left_slack_mm=None,substeps=32,model='modal',spin_acceleration=True,roll_stiffness=None,right_preload_mm=None):
  if flight not in ['hood_river','trout_lake']:raise ValueError('flight must be hood_river or trout_lake')
  if yaw is None:yaw='mass_linear' if model=='modal' else 'constant'
  if yaw not in ['constant','mass_linear','geometry']:raise ValueError('unknown yaw model')
  if model not in ['modal','aero']:raise ValueError('model must be modal or aero')
  calibration=json.loads((ROOT/'modal-calibration.json').read_text())
  if gyro_inertia is None:gyro_inertia=calibration['rotor_spin_inertia'] if model=='modal' else 0.
  if not np.isfinite(gyro_inertia) or gyro_inertia<0 or not isinstance(substeps,(int,np.integer)) or substeps<1:raise ValueError('invalid inertia or integration resolution')
  self.flight=flight;self.substeps=int(substeps);day='2026-01-03' if flight=='hood_river' else '2026-02-15'
  self.geometry=configuration(day,dict(ballast_below_original_payload_CG_m=.10,imu_above_original_payload_CG_m=.05))
  if model=='modal':
   from cad_geometry import configuration as cad_configuration
   self.geometry=cad_configuration(day)
  self.model=model
  self.parameters=dynamics.DEFAULT.copy()|json.loads((ROOT/'parameters.json').read_text())['parameters']|self.geometry['parameters']
  if roll_stiffness is not None:
   if not np.isfinite(roll_stiffness) or roll_stiffness<0:raise ValueError('nonnegative finite roll stiffness required')
   self.parameters['joint_k_roll']=float(roll_stiffness)
  self.parameters['rotor_spin_inertia']=float(gyro_inertia)
  self.parameters['spin_acceleration_scale']=float(spin_acceleration)
  span=calibration['yaw_canopy_halfspacing'] if model=='modal' else .5
  self.parameters['yaw_canopy_halfspacing']=span
  if left_slack_mm is None and model=='modal':left_slack_mm=12.151707100207533 if flight=='hood_river' else 0.
  if left_slack_mm is not None:
   if not 0<=left_slack_mm<62.4:raise ValueError('invalid slack')
   self.parameters['left_offset']=left_slack_mm/1000
  if right_preload_mm is None:right_preload_mm=6. if model=='modal' and flight=='hood_river' else 0.
  if not np.isfinite(right_preload_mm) or not 0<=right_preload_mm<=12:raise ValueError('right preload must be 0..12 mm')
  self.parameters['right_offset']=-right_preload_mm/1000
  if yaw=='mass_linear':self.parameters['joint_k_yaw']=self.geometry['payload_mass']*9.80665*.0743*span/.85
  if yaw=='geometry':self.parameters['geometry_yaw']=1.
 def initial_state(self,speed=4.,canopy_pitch=0.):
  rp=Rotation.identity();rc=Rotation.from_euler('y',canopy_pitch);v=np.array([speed,0,0.]);return np.r_[np.zeros(3),rp.as_quat(),v,np.zeros(3),self.geometry['hinge']-rc.apply([0,0,.85]),rc.as_quat(),v,np.zeros(3)]
 def controls(self,throttle,left_mm,right_mm):
  a=np.array([throttle,left_mm,right_mm],float)
  if not np.isfinite(a).all() or not 0<=throttle<=1 or not 0<=left_mm<=62.4 or not 0<=right_mm<=72.01:raise ValueError('command outside logged actuator ranges')
  pull=np.array([left_mm/1000-self.parameters['left_offset'],right_mm/1000-self.parameters['right_offset']])
  if not self.parameters['routing_enabled']:pull=np.maximum(pull,0.)
  return np.r_[throttle,pull]
 def from_pwm(self,throttle_pwm,left_pwm,right_pwm):
  mm_per_us=np.pi*55/360*.1
  command=np.array([(throttle_pwm-1000)/1000,(left_pwm-800)*mm_per_us,(2400-right_pwm)*mm_per_us])
  self.controls(*command) # validate without changing public command units
  return command
 def replay(self,state,commands,voltage=22.2,rpm=0.,wind=(0,0,0),dt=.04):
  if not np.isfinite(voltage) or voltage<=0 or not np.isfinite(dt) or dt<=0:raise ValueError('positive finite voltage and dt required')
  if np.asarray(state).shape!=(26,) or not np.isfinite(state).all():raise ValueError('state must contain 26 finite values')
  if np.asarray(wind).shape!=(3,) or not np.isfinite(wind).all():raise ValueError('wind must contain three finite values')
  if not np.isfinite(rpm) or rpm<0:raise ValueError('nonnegative finite RPM required')
  commands=np.asarray(commands,float)
  if commands.ndim!=2 or commands.shape[1]!=3 or len(commands)<1:raise ValueError('commands must be Nx3 throttle,left_mm,right_mm')
  u=np.array([self.controls(*row) for row in commands])
  return dynamics.causal_replay(state,u,self.parameters,self.geometry['mass'],np.asarray(wind,float),float(voltage),float(rpm),dt=dt,substeps=self.substeps)
 def replay_actuated(self,state,commands,voltage=22.2,rpm=0.,wind=(0,0,0),dt=.04,rate_mm_s=221.522558906):
  from actuator import limited_commands
  applied=limited_commands(commands,dt,rate_mm_s)
  y,r=self.replay(state,applied,voltage,rpm,wind,dt)
  return y,r,applied
 def step_actuated(self,state,rpm,actuator,command,voltage=22.2,wind=(0,0,0),dt=.04):
  self.controls(*command)
  applied=np.r_[command[0],actuator.advance(command[1:],dt)]
  y,r=self.step(state,rpm,applied,voltage,wind,dt)
  return y,r,applied
 def step(self,state,rpm,command,voltage=22.2,wind=(0,0,0),dt=.04):
  y,r=self.replay(state,np.tile(command,(2,1)),voltage,rpm,wind,dt);return y[-1],float(r[-1])

def demo():
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--flight',choices=['hood_river','trout_lake'],default='trout_lake');p.add_argument('--seconds',type=float,default=120);p.add_argument('--output',default='demo.npz');args=p.parse_args()
 sim=Simulator(args.flight);t=np.arange(round(args.seconds/.04)+1)*.04;u=np.tile([.4,0.,0.],(len(t),1));u[(t>=40)&(t<50),2]=10;u[(t>=70)&(t<80),1]=20;y,r=sim.replay(sim.initial_state(),u);np.savez_compressed(args.output,t=t,state=y,rpm=r,commands=u);print(json.dumps(dict(output=args.output,finite=bool(np.isfinite(y).all()),duration_s=float(t[-1]),final_speed=float(np.linalg.norm(y[-1,7:10])),final_climb=float(-y[-1,9])),indent=2))
if __name__=='__main__':demo()
