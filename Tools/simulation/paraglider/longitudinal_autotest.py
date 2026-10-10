"""Native FBWB longitudinal mission and bounded lateral ablation study."""
import os,json,runpy
from pathlib import Path
import numpy as np
os.environ['PG_GAINS']='original';os.environ['PG_SAMPLE_DIVISOR']='8'
import compare_autotest as h
config=json.loads(Path(os.environ['PG_STUDY_CONFIG']).read_text())
class StudyBackend(h.Backend):
 def __init__(self,*args,**kwargs):
  kwargs['voltage']=config.get('voltage',22.2)
  super().__init__(*args,**kwargs)
  if config.get('fixed_brakes'):
   step=self.sim.step_actuated;brakes=self.trim['command'][1:].copy()
   def fixed(state,rpm,actuator,command,**options):
    command=np.array(command,copy=True);command[1:]=brakes
    return step(state,rpm,actuator,command,**options)
   self.sim.step_actuated=fixed
h.Backend=StudyBackend

def experiment(self):
 h.airborne(self,altitude=150)
 self.set_parameters({'TECS_PTCH_DAMP':0,'FBWB_CLIMB_RATE':1.0,'FBWB_ELEV_REV':0,'RC2_REVERSED':0}|config.get('parameters',{}))
 backend=self._pg_backend[0];events=[]
 def phase(name,seconds,roll=1500,pitch=1500,params=None):
  if params:self.set_parameters(params)
  self.set_rc_from_map({1:roll,2:pitch})
  events.append(dict(name=name,start=backend.time,duration=seconds,roll_pwm=roll,pitch_pwm=pitch,parameters=params or {}))
  self.delay_sim_time(seconds,reason=name)
  events[-1]['end']=backend.time
 if config.get('ablation'):
  reference={'PG_TURN_P':.04,'PG_TURN_I':.002,'PG_TURN_RDAMP':.03,'PG_TURN_D_FF':.02,'PG_TURN_FILT':4}
  for name,change in [('reference',{}),('no_rdamp',{'PG_TURN_RDAMP':0}),('no_dff',{'PG_TURN_D_FF':0}),('no_p',{'PG_TURN_P':0}),('rdamp_small',{'PG_TURN_RDAMP':.003}),('reference_repeat',{})]:
   phase(name,70,params=reference|change)
 else:
  phase('settle',40)
  phase('level',35)
  phase('climb',35,pitch=1700)
  phase('climb_capture',45)
  phase('descent',35,pitch=1300)
  phase('descent_capture',45)
  phase('right_turn',65,roll=1550)
  phase('left_turn',65,roll=1450)
  phase('final_capture',35)
 Path(self.buildlogs_path('study-events.json')).write_text(json.dumps(dict(config=config,trim_command=backend.trim['command'].tolist(),events=events),indent=2))
 self.assert_armed();self.assert_mode('FBWB')
h.original=experiment;h.mission.__name__='ParagliderLongitudinalStudy';h.mission.__doc__=__doc__
runpy.run_path(str(h.ROOT/'Tools/autotest/autotest.py'),run_name='__main__')
