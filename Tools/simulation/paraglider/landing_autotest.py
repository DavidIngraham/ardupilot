"""Exercise native AP_Landing flare with the airborne two-body model.

Stop at first payload ground-plane crossing: no impact or canopy-collapse model.
PG_LANDING_CASE selects nominal, weak, strong, crosswind, abort, takeover, disabled, timed, or reserve.
Outputs use the standard autotest buildlogs directory.
"""
# AP_FLAKE8_CLEAN
import json
import math
import os
import runpy
from pathlib import Path

import compare_autotest as harness

import numpy as np

from pymavlink import mavutil

CASE = os.environ.get('PG_LANDING_CASE', 'nominal')
if CASE not in ('nominal', 'weak', 'strong', 'crosswind', 'abort', 'takeover', 'disabled', 'timed', 'reserve'):
    raise ValueError('Unknown landing case: ' + CASE)
os.environ['PG_GAINS'] = 'original'
os.environ['PG_SAMPLE_DIVISOR'] = '8'


class LandingBackend(harness.Backend):
    """Apply explicit aerodynamic uncertainty and detect first ground crossing."""

    def __init__(self, *args, **kwargs):
        """Keep the ordinary profile unchanged outside this study."""
        super().__init__(*args, **kwargs)
        # These are uncertainty cases, not fitted flare coefficients.
        factor = {'weak': 0.5, 'strong': 1.5}.get(CASE, 1.0)
        self.sim.parameters['CLbrake'] *= factor
        self.sim.parameters['CDbrake'] *= factor
        self.contact = None
        original_step = self.sim.step_actuated

        def step(state, rpm, actuator, command, **options):
            options['wind'] = (0, 1.5, 0) if CASE == 'crosswind' else (0, 0, 0)
            result = original_step(state, rpm, actuator, command, **options)
            if result[0][2] >= 0 and self.contact is None:
                self.contact = {'time': self.time, 'position': result[0][:3].tolist(),
                                'velocity': result[0][7:10].tolist(),
                                'brake_mm': actuator.position.tolist()}
            return result

        self.sim.step_actuated = step

    def respond(self, data):
        """Report wind-relative airspeed consistently with the dynamics."""
        response = super().respond(data)
        if CASE == 'crosswind':
            message = json.loads(response)
            message['velocity_wind'] = [0., 1.5, 0.]
            message['airspeed'] = float(np.linalg.norm(self.state[7:10] - [0, 1.5, 0]))
            self.last = ('\n' + json.dumps(message, allow_nan=False) + '\n').encode()
            return self.last
        return response


harness.Backend = LandingBackend


def experiment(self):
    """Fly the native landing state machine and record first-contact metrics."""
    harness.airborne(self, altitude=40)
    backend = self._pg_backend[0]
    self.set_parameters({
        'PG_FLARE_PCT': {'disabled': 0, 'reserve': 100}.get(CASE, 60), 'PG_FLARE_RATE': 30, 'PG_FLARE_RES': 20,
        'LAND_FLARE_ALT': {'timed': 0, 'reserve': 8}.get(CASE, 3), 'LAND_FLARE_SEC': 3 if CASE == 'timed' else 0,
        'LAND_DISARMDELAY': 0, 'TECS_LAND_THR': -1,
    })
    north, east = backend.state[:2]
    waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    land = mavutil.mavlink.MAV_CMD_NAV_LAND
    mission = self.create_simple_relhome_mission([
        (waypoint, north + 50, east, 40),
        (land, north + 260, east, 0),
    ])
    self.check_mission_upload_download(mission)
    self.change_mode('AUTO')
    start = backend.time
    samples = []
    action = None
    while backend.time - start < 180:
        msg = self.assert_receive_message('GLOBAL_POSITION_INT')
        height = -backend.state[2]
        samples.append([backend.time, height, *backend.state[7:10], *backend.command])
        if CASE == 'abort' and height < 20 and action is None:
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_GO_AROUND, p1=40)
            action = backend.time
        if CASE == 'takeover' and height < 2 and action is None:
            self.change_mode('MANUAL')
            self.set_rc_from_map({1: 1500, 2: 1500, 3: 1000})
            action = backend.time
        if action is not None and backend.time - action > (20 if CASE == 'abort' else 1):
            break
        if backend.contact is not None:
            break
        if not math.isfinite(msg.relative_alt):
            raise RuntimeError('Nonfinite altitude')
    np.savetxt(self.buildlogs_path('landing.csv'), samples, delimiter=',',
               header='time,height,vn,ve,vd,throttle,left_mm,right_mm', comments='')
    summary = {'case': CASE, 'contact': backend.contact, 'action_time': action,
               'final_height': -float(backend.state[2]), 'duration': backend.time-start}
    Path(self.buildlogs_path('landing.json')).write_text(json.dumps(summary, indent=2))
    if CASE == 'abort':
        if action is None or summary['final_height'] < 20:
            raise RuntimeError('Go-around did not regain height')
    elif CASE == 'takeover':
        if action is None:
            raise RuntimeError('Manual takeover not exercised')
    elif backend.contact is None:
        raise RuntimeError('No ground-plane crossing')
    self.disarm_vehicle(force=True)


harness.original = experiment
harness.mission.__name__ = 'ParagliderLandingStudy'
harness.mission.__doc__ = __doc__
if __name__ == '__main__':
    runpy.run_path(str(harness.ROOT / 'Tools/autotest/autotest.py'), run_name='__main__')
