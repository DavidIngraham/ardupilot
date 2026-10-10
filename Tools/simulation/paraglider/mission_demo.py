from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent
from simulator import Simulator
import argparse
parser=argparse.ArgumentParser(description='Ideal-sensor mission-controller diagnostic, not flight firmware')
parser.add_argument('--flight',choices=['hood_river','trout_lake'],default='trout_lake')
parser.add_argument('--case',choices=['nominal','rigging_low','rigging_high','soft_roll','low_damping','rotor_low','rotor_high','slow_servo','dynamic_roll'],default='nominal')
parser.add_argument('--feedforward',choices=['power_trim'],default='power_trim')
parser.add_argument('--output',default='mission-demo.npz')
args=parser.parse_args()
from controllers import Turn,Climb
from actuator import BrakeActuator
from equilibrium import solve
trims=json.loads((R/'equilibrium-trim-results.json').read_text());results=[]
for flight in [args.flight]:
 offset=12.151707100207533 if flight=='hood_river' else 0.;table=sorted([q for q in trims if q['flight']==flight and q['requested_climb']==0],key=lambda q:q['requested_turn']);tx=[q['requested_turn'] for q in table];fx=[(q['command'][2]-max(q['command'][1]-offset,0))/72 for q in table];straight=[next(q for q in trims if q['flight']==flight and q['requested_turn']==0 and q['requested_climb']==c) for c in [0,.5]];bias_slope=(straight[1]['command'][2]-straight[0]['command'][2])/(straight[1]['command'][0]-straight[0]['command'][0])/72;th0=straight[0]['command'][0]
 for case in [args.case]:
  for ffmode in [args.feedforward]:
   sim=Simulator(flight)
   if case=='dynamic_roll':
    from profiles import make_simulator
    sim=make_simulator(flight,'dynamic_roll')
   if case=='rigging_low':
    if flight=='hood_river':sim.parameters['right_offset']=-.004
    else:sim.parameters['left_offset']=.006
   if case=='rigging_high':sim.parameters['right_offset']=-.008 if flight=='hood_river' else -.004
   if case=='soft_roll':sim.parameters['joint_k_roll']=1.
   if case=='low_damping':sim.parameters['joint_c_yaw']*=.25;sim.parameters['joint_c_pitch']*=.25
   if case=='rotor_low':sim.parameters['rotor_spin_inertia']*=.75
   if case=='rotor_high':sim.parameters['rotor_spin_inertia']*=1.25
   d=np.load(R/f'equilibrium-trim-{flight}-0.0-0.0.npz');root=solve(sim,d['state'],d['command']);assert abs(root['residual']).max()<1e-6,'Equilibrium not resolved';state=root['state'];rpm=root['rpm'];act=BrakeActuator(rate_mm_s=110.761279453 if case=='slow_servo' else 221.522558906,position_mm=root['command'][1:]);throttle=th0
   def feedforward(rate):return float(np.interp(rate,tx,fx)+(bias_slope*(throttle-th0) if ffmode=='power_trim' else 0))
   turn=Turn(feedforward=None if ffmode=='fixed' else feedforward,p=.005,i=.0005,rdamp=.003,dff=0,filter_hz=1);climb=Climb();climb.last=th0;climb.integral=th0-.43;dt=.04;t=np.arange(6001)*dt;out=[];states=[];previous=np.arctan2(state[8],state[7]);airrate=0.
   for now in t:
    request=[0,5,10,0,-5,-10,0,0,0][min(int(now//30),8)];creq=.5 if 90<=now<150 else 0.;rot=Rot.from_quat(state[3:7]);phi,theta,_=rot.as_euler('xyz');body=rot.inv().apply(state[10:13]);hr=np.rad2deg((body[1]*np.sin(phi)+body[2]*np.cos(phi))/np.cos(theta));throttle=climb.update(creq,-state[9]);u=turn.update(request,hr,np.rad2deg(body[0]),np.linalg.norm(state[7:10]));cmd=[throttle,min(max(-u,0)*72+(offset if u<0 else 0),62.4),min(max(u,0)*72,72)];out.append([request,creq,hr,airrate,u,throttle,turn.integral,-state[9],np.rad2deg(phi),*act.position]);states.append(state.copy());state,rpm,applied=sim.step_actuated(state,rpm,act,cmd);course=np.arctan2(state[8],state[7]);airrate=np.rad2deg(np.arctan2(np.sin(course-previous),np.cos(course-previous))/dt);previous=course
   a=np.array(out);states=np.array(states);mask=(t%30>=10)&(t<240);key=f'{flight}-{case}-{ffmode}';np.savez_compressed(Path(args.output),t=t,data=a,state=states,data_columns=np.array(['requested_course_rate_deg_s','requested_climb_m_s','payload_heading_rate_deg_s','air_course_rate_deg_s','normalized_brake','throttle_fraction','turn_integral','climb_m_s','payload_bank_deg','left_travel_mm','right_travel_mm']));row=dict(flight=flight,case=case,feedforward=ffmode,course_rate_rmse=float(np.sqrt(np.mean((a[mask,3]-a[mask,0])**2))),heading_rate_rmse=float(np.sqrt(np.mean((a[mask,2]-a[mask,0])**2))),climb_rmse=float(np.sqrt(np.mean((a[mask,7]-a[mask,1])**2))),saturation_fraction=float(np.mean(abs(a[:,4])>.999)),actuator_clipping_fraction=float(np.mean((np.maximum(-a[:,4],0)*72+(a[:,4]<0)*offset>62.4+1e-9)|(np.maximum(a[:,4],0)*72>72+1e-9))),max_left_mm=float(a[:,9].max()),max_right_mm=float(a[:,10].max()),bias_slope_fraction_per_throttle=float(bias_slope));results.append(row);Path(args.output).with_suffix('.json').write_text(json.dumps(results,indent=2));print(row,flush=True)
