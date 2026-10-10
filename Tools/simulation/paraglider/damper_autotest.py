"""Native TECS damper sign experiment: matched TL geometry and pitch-rate kick."""
import os
import json
import runpy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
os.environ['PG_GAINS']='original'
os.environ['PG_SAMPLE_DIVISOR']='8'
import compare_autotest as harness

# Diagnostic intervention: retain native TECS, but remove lateral actuator forcing.
if os.environ.get('PG_FIXED_BRAKES') == '1':
    class FixedBrakeBackend(harness.Backend):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            step = self.sim.step_actuated
            brakes = self.trim['command'][1:].copy()
            def fixed_step(state, rpm, actuator, command, **options):
                command = np.array(command, copy=True)
                command[1:] = brakes
                return step(state, rpm, actuator, command, **options)
            self.sim.step_actuated = fixed_step
    harness.Backend = FixedBrakeBackend


def experiment(self):
    harness.airborne(self,altitude=100)
    self.set_parameter('TECS_PTCH_DAMP',0)
    self.delay_sim_time(30,reason='settle native level-flight controller without pitch damper')
    gain=float(os.environ.get('PG_DAMP_GAIN','0'))
    self.set_parameter('TECS_PTCH_DAMP',gain)
    assert abs(self.get_parameter('TECS_PTCH_DAMP')-gain)<1e-6
    self.assert_parameter_values({'TECS_PG_PR_FILT':10})
    self.delay_sim_time(1,reason='propagate selected damper gain')
    backend=self._pg_backend[0]
    # Run the kick inside the physics thread to avoid racing state integration.
    original_step=backend.sim.step_actuated
    event={}
    def kick_once(state,rpm,actuator,command,**kwargs):
        state=state.copy()
        rot=Rotation.from_quat(state[3:7])
        delta=rot.apply([0,np.deg2rad(5.),0])
        state[10:13]+=delta
        # Preserve velocity compatibility at the point joint.
        state[7:10]-=np.cross(delta,rot.apply(backend.sim.geometry['hinge']))
        event.update(time_s=backend.time,gain=gain,kick_body_pitch_rate_deg_s=5,
                     state_before=backend.state.tolist(),state_after=state.tolist(),
                     mass_kg=backend.sim.geometry['mass'],profile='mission',
                     fixed_brakes=os.environ.get('PG_FIXED_BRAKES') == '1',brake_trim_mm=backend.trim['command'][1:].tolist(),
                     filter_hz=10,sign_equation='delta_throttle = -gain * filtered_body_pitch_rate_rad_s')
        backend.sim.step_actuated=original_step
        return original_step(state,rpm,actuator,command,**kwargs)
    backend.sim.step_actuated=kick_once
    self.delay_sim_time(75,reason='observe pitch-rate perturbation and damper response')
    Path(self.buildlogs_path('damper-event.json')).write_text(json.dumps(event,indent=2))
    self.assert_armed()
    self.assert_mode('FBWB')

harness.original=experiment
harness.mission.__name__='ParagliderDamperSign'
harness.mission.__doc__='Compare pitch-rate damper signs after a matched airborne perturbation'
runpy.run_path(str(harness.ROOT/'Tools/autotest/autotest.py'),run_name='__main__')
