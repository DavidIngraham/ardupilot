from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation as Rot
from simulator import Simulator
from actuator import BrakeActuator
checks={}
for flight in ['hood_river','trout_lake']:
 sim=Simulator(flight);command=sim.from_pwm(1400,1000,2200);np.testing.assert_allclose(command,[.4,9.599310885968812,9.599310885968812]);checks[flight+'_pwm_command']=command.tolist()
 y0=sim.initial_state();commands=np.tile(command,(26,1));y,r=sim.replay(y0,commands);z=y0.copy();rpm=0.
 for _ in range(25):z,rpm=sim.step(z,rpm,command)
 np.testing.assert_allclose(z,y[-1],atol=1e-12);np.testing.assert_allclose(rpm,r[-1],atol=1e-12);checks[flight+'_step_batch_max']=float(np.max(abs(z-y[-1])))
 ap=Rot.from_quat(y[:,3:7]).apply(np.broadcast_to(sim.geometry['hinge'],(len(y),3)));ac=Rot.from_quat(y[:,16:20]).apply(np.broadcast_to([0,0,.85],(len(y),3)));err=y[:,:3]+ap-y[:,13:16]-ac;assert abs(err).max()<1e-5;checks[flight+'_joint_position_max_m']=float(abs(err).max())
 for sl in [slice(3,7),slice(16,20)]:np.testing.assert_allclose(np.linalg.norm(y[:,sl],axis=1),1,atol=1e-10)
 servo=BrakeActuator(position_mm=command[1:]);ys,rs,applied=sim.step_actuated(y0,0,servo,command);yb,rb,ub=sim.replay_actuated(y0,np.tile(command,(2,1)));np.testing.assert_allclose(ys,yb[-1],atol=1e-12)
act=BrakeActuator(rate_mm_s=10,position_mm=[0,10]);np.testing.assert_allclose(act.advance([2,0],1),[1.8,5]);np.testing.assert_allclose(act.position,[2,0]);np.testing.assert_allclose(act.advance([22,0],1),[7,0]);np.testing.assert_allclose(act.position,[12,0]);checks['servo_exact_interval_average']='passed'
sim=Simulator('hood_river');np.testing.assert_allclose(sim.controls(*sim.from_pwm(1000,800,2400)),[0,0,.006]);checks['preload_applied_once']='passed'
for command in [[-1,0,0],[0,63,0],[0,0,float('nan')]]:
 try:sim.controls(*command)
 except ValueError:pass
 else:raise AssertionError('bad command accepted')
checks['invalid_commands']='rejected'



from profiles import make_simulator
for kwargs in [dict(roll_stiffness=2),dict(right_preload_mm=2),dict(model='aero')]:
 try:make_simulator('trout_lake','dynamic_roll',**kwargs)
 except ValueError:pass
 else:raise AssertionError('Conflicting profile override was accepted')
checks['profile_override_validation']='passed'
Path(__file__).with_name('verification.json').write_text(json.dumps(checks,indent=2));print(json.dumps(checks,indent=2))
