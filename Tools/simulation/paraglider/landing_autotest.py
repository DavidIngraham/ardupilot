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
from collections import deque
from pathlib import Path

import compare_autotest as harness

import numpy as np

from pymavlink import mavutil

from scipy.spatial.transform import Rotation

CASE = os.environ.get('PG_LANDING_CASE', 'nominal')
if CASE not in ('nominal', 'weak', 'strong', 'crosswind', 'abort', 'takeover', 'disabled', 'timed', 'reserve'):
    raise ValueError('Unknown landing case: ' + CASE)
CONFIG = json.loads(Path(os.environ['PG_LANDING_CONFIG']).read_text()) if os.environ.get('PG_LANDING_CONFIG') else {}
# Ordinary landing tests share the complete mission controller. Explicit study
# overrides retain their existing behavior for reproducible archived runs.
if 'PG_CONTROLLER_CONFIG' not in os.environ and 'PG_GAINS' not in os.environ:
    os.environ['PG_CONTROLLER_CONFIG'] = str(Path(__file__).with_name('mission-controller-candidate.json'))
os.environ.setdefault('PG_GAINS', 'mission')
os.environ['PG_SAMPLE_DIVISOR'] = '8'


class LandingBackend(harness.Backend):
    """Apply explicit aerodynamic uncertainty and detect first ground crossing."""

    def __init__(self, *args, **kwargs):
        """Keep the ordinary profile unchanged outside this study."""
        kwargs['voltage'] = CONFIG.get('voltage_v', kwargs.get('voltage', 22.2))
        kwargs['altitude'] = kwargs.get('altitude', 40) + CONFIG.get('height_error_m', 0)
        super().__init__(*args, **kwargs)
        heading = Rotation.from_euler('z', np.deg2rad(CONFIG.get('heading_error_deg', 0)))
        self.state[:3] = heading.apply(self.state[:3])
        self.state[13:16] = heading.apply(self.state[13:16])
        for offset in (3, 16):
            self.state[offset:offset+4] = (heading * Rotation.from_quat(self.state[offset:offset+4])).as_quat()
        for offset in (7, 20):
            self.state[offset:offset+3] = heading.apply(self.state[offset:offset+3]) * CONFIG.get('speed_scale', 1)
        self.actuator.rate = CONFIG.get('actuator_rate_mm_s', self.actuator.rate)
        self.delayed_commands = deque()
        self.wind_epoch = None
        # These are uncertainty cases, not fitted flare coefficients.
        factor = {'weak': 0.5, 'strong': 1.5}.get(CASE, 1.0)
        self.sim.parameters['CLbrake'] *= CONFIG.get('brake_lift_scale', factor)
        self.sim.parameters['CDbrake'] *= CONFIG.get('brake_drag_scale', factor)
        self.contact = None
        self.viewer_frames = []
        original_step = self.sim.step_actuated

        def step(state, rpm, actuator, command, **options):
            if self.wind_epoch is None:
                self.wind_epoch = self.time
            options['wind'] = self.wind()
            self.delayed_commands.append((self.time, np.array(command, copy=True)))
            delayed_time = self.time - CONFIG.get('actuator_delay_s', 0)
            while len(self.delayed_commands) > 1 and self.delayed_commands[1][0] <= delayed_time:
                self.delayed_commands.popleft()
            applied = np.array(command, copy=True)
            applied[1:] = self.delayed_commands[0][1][1:]
            result = original_step(state, rpm, actuator, applied, **options)
            if state[2] < 0 <= result[0][2] and self.contact is None:
                fraction = -state[2] / (result[0][2] - state[2])
                contact_state = state + fraction * (result[0] - state)
                self.contact = {'time': self.time + fraction * options['dt'],
                                'position': contact_state[:3].tolist(),
                                'velocity': contact_state[7:10].tolist(),
                                'brake_mm': actuator.position.tolist()}
            return result

        self.sim.step_actuated = step

    def wind(self):
        """Deterministic smooth gusts, explicitly not a validated turbulence spectrum."""
        mean = np.array(CONFIG.get('wind_ned_m_s', [0, 1.5, 0] if CASE == 'crosswind' else [0, 0, 0]))
        time = 0 if self.wind_epoch is None else self.time - self.wind_epoch
        phase = CONFIG.get('gust_phase_rad', 0)
        omega = 2 * np.pi / CONFIG.get('gust_period_s', 12)
        amplitude = CONFIG.get('gust_amplitude_m_s', 0)
        return mean + amplitude * np.array([np.sin(omega*time+phase),
                                           np.sin(0.73*omega*time+phase+1),
                                           0.25*np.sin(1.31*omega*time+phase+2)])

    def respond(self, data):
        """Report wind-relative airspeed consistently with the dynamics."""
        response = super().respond(data)
        if CONFIG.get('record_viewer') and self.released:
            if not self.viewer_frames or self.time >= self.viewer_frames[-1]['time_s'] + 0.05:
                self.viewer_frames.append(self.viewer_state() | {'wind_ned_m_s': self.wind().tolist()})
        if CASE == 'crosswind' or CONFIG:
            message = json.loads(response)
            wind = self.wind()
            message['velocity_wind'] = wind.tolist()
            message['airspeed'] = float(np.linalg.norm(self.state[7:10] - wind))
            self.last = ('\n' + json.dumps(message, allow_nan=False) + '\n').encode()
            return self.last
        return response


harness.Backend = LandingBackend


def experiment(self):
    """Fly the native landing state machine and record first-contact metrics."""
    harness.airborne(self, altitude=40)
    backend = self._pg_backend[0]
    self.set_parameters({
        'PG_FLARE_PCT': CONFIG.get('flare_pct', {'disabled': 0, 'reserve': 100}.get(CASE, 60)),
        'PG_FLARE_RATE': 30, 'PG_FLARE_RES': 20,
        'LAND_FLARE_ALT': {'timed': 0, 'reserve': 8}.get(CASE, 3), 'LAND_FLARE_SEC': 3 if CASE == 'timed' else 0,
        'LAND_DISARMDELAY': 0, 'TECS_LAND_THR': -1,
        'LAND_FLARE_AIM': CONFIG.get('flare_aim_pct', 50),
        'TECS_LAND_SINK': CONFIG.get('flare_sink_m_s', 0.25),
    })
    north, east = backend.state[:2]
    east -= CONFIG.get('cross_track_error_m', 0)
    target = np.array([north + 260, east, 0])
    waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    land = mavutil.mavlink.MAV_CMD_NAV_LAND
    mission = self.create_simple_relhome_mission([
        (waypoint, north + 50, east, 40),
        (land, target[0], target[1], CONFIG.get('landing_datum_error_m', 0)),
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
               'final_height': -float(backend.state[2]), 'duration': backend.time-start,
               'config': CONFIG, 'target_ned_m': target.tolist(),
               'mission_land_lat_lon': [mission[-1].x * 1e-7, mission[-1].y * 1e-7]}
    if backend.contact is not None:
        error = np.array(backend.contact['position']) - target
        summary['along_track_error_m'] = float(error[0])
        summary['cross_track_error_m'] = float(error[1])
        summary['radial_error_m'] = float(np.linalg.norm(error[:2]))
    if backend.viewer_frames:
        Path(self.buildlogs_path('viewer.json')).write_text(json.dumps(backend.viewer_frames))
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
