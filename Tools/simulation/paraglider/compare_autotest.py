"""Run the unchanged extended AUTO checks using airborne two-body initialization.

PG_PROFILE selects mission/dynamic_roll. PG_GAINS selects original/conservative.
The original native launch is replaced; route, metrics and pass criteria are not.
"""
from pathlib import Path
import os
import runpy
import socket
import sys
import threading
import json
import shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Tools/autotest'))
import arduplane
from json_backend import Backend
from pymavlink import mavutil


def airborne(self, altitude=60, **kwargs):
    backend=Backend(profile=os.environ.get('PG_PROFILE','mission'), altitude=altitude,
                    voltage=float(os.environ.get('PG_VOLTAGE', '22.2')))
    stop=threading.Event()
    errors=[]
    samples=[]
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1',9002))
    sock.settimeout(.5)
    def serve():
        try:
            while not stop.is_set():
                try:data,address=sock.recvfrom(2048)
                except socket.timeout:continue
                sock.sendto(backend.respond(data),address)
                if backend.frame % int(os.environ.get("PG_SAMPLE_DIVISOR", "40")) == 0:
                    samples.append([backend.time, float(backend.released), *backend.state])
        except Exception as e:
            errors.append(repr(e))
        finally:sock.close()
    thread=threading.Thread(target=serve,daemon=True)
    thread.start()
    self._pg_backend=(backend,stop,thread,errors,samples)
    self.customise_SITL_commandline(['--rate','400'],model='JSON:127.0.0.1',
        defaults_filepath=[str(ROOT/'Tools/autotest/default_params/paraglider.parm'),
                           str(Path(__file__).with_name('sitl.parm'))],wipe=True)
    self.set_rc_default()
    self.set_parameters({'SIM_WIND_SPD':0,'SIM_WIND_TURB':0,'SIM_SPEEDUP':10})
    if os.environ.get('PG_GAINS','conservative')=='original':
        self.set_parameters({'PG_TURN_P':.04,'PG_TURN_I':.002,'PG_TURN_RDAMP':.03,
                             'PG_TURN_D_FF':.02,'PG_TURN_FILT':4,'PG_TURN_RMAX':25})
    if os.environ.get('PG_CONTROLLER_CONFIG'):
        controller_config = json.loads(Path(os.environ['PG_CONTROLLER_CONFIG']).read_text())
        self.set_parameters(controller_config['parameters'])
    self.change_mode('MANUAL')
    self.wait_ready_to_arm()
    self.arm_vehicle()
    self.set_rc_from_map({1:1500,2:1500,3:1500,4:1500})
    # Arming sets home at the airborne position: restore the ground datum.
    self.set_home(self.sitl_start_location())
    self.delay_sim_time(2, reason="allow ground home datum to propagate")
    self.change_mode('FBWB')
    self.delay_sim_time(1, reason="initialize FBWB height demand with valid position")
    self.progress('Releasing two-body airborne physics to native controllers')
    backend.released=True


original=arduplane.AutoTestPlane.ParagliderAutoMission

def mission(self):
    reset = self.reset_SITL_commandline
    captured = False

    def capture_then_reset(*args, **kwargs):
        nonlocal captured
        # The original AUTO test resets SITL before returning. Save the flying
        # log before that reset creates a newer, unrelated startup log.
        if hasattr(self, '_pg_backend') and self._pg_backend[0].released and not captured:
            logs = list((ROOT / 'logs').glob('*.BIN'))
            if logs:
                shutil.copy2(max(logs, key=lambda p: p.stat().st_mtime), self.buildlogs_path('flight.BIN'))
                captured = True
        return reset(*args, **kwargs)

    self.reset_SITL_commandline = capture_then_reset
    try:
        original(self)
    finally:
        try:
            if hasattr(self, '_pg_backend'):
                backend, stop, thread, errors, samples = self._pg_backend
                self.reset_SITL_commandline()
                stop.set()
                thread.join(timeout=2)
                np.savez_compressed(self.buildlogs_path('high-dof-trace.npz'), samples=np.array(samples))
                Path(self.buildlogs_path('high-dof-backend.json')).write_text(json.dumps(dict(
                    profile=os.environ.get('PG_PROFILE', 'mission'),
                    gains=os.environ.get('PG_GAINS', 'conservative'),
                    time_s=backend.time, voltage=backend.voltage, clipped_packets=backend.clipped_packets,
                    state=backend.state.tolist(), errors=errors), indent=2))
        finally:
            self.reset_SITL_commandline = reset

mission.__name__='ParagliderAutoMission'
mission.__doc__=original.__doc__
arduplane.AutoTestPlane.paraglider_takeoff=airborne
arduplane.AutoTestPlane.ParagliderAutoMission=mission
if __name__ == '__main__':
    runpy.run_path(str(ROOT/'Tools/autotest/autotest.py'),run_name='__main__')
