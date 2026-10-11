"""Airborne, lockstep ArduPilot JSON backend for the two-body parafoil model.

Run this first, then SITL with --model JSON:127.0.0.1. Press Enter to release
from the moving equilibrium initialization after the autopilot is ready.
"""
import argparse
import json
import socket
import struct
import threading
import numpy as np
from scipy.spatial.transform import Rotation
from profiles import make_simulator
from trim import find_trim
from sensors import payload_observation
from actuator import BrakeActuator


def decode_packet(data):
    if len(data) not in (40, 72):
        raise ValueError('Expected 16- or 32-channel servo packet')
    magic, rate, frame = struct.unpack_from('<HHI', data)
    if magic != (18458 if len(data) == 40 else 29569) or not 1 <= rate <= 2000:
        raise ValueError('Invalid magic or frame rate')
    return rate, frame, struct.unpack_from('<' + 'H' * ((len(data)-8)//2), data, 8)


class Backend:
    def __init__(self, flight='trout_lake', profile='mission', altitude=100., voltage=22.2):
        self.sim = make_simulator(flight, profile)
        self.trim = find_trim(self.sim, voltage=voltage)
        self.state = self.trim['state'].copy()
        self.state[[2, 15]] -= altitude
        self.rpm = self.trim['rpm']
        self.command = self.trim['command'].copy()
        self.actuator = BrakeActuator(position_mm=self.trim['command'][1:])
        self.voltage = voltage
        self.time = 0.
        self.frame = None
        self.last = None
        self.released = False
        self.clipped_packets = 0

    def viewer_state(self):
        """Versioned truth state: NED world, FRD bodies, body-to-world xyzw."""
        payload = Rotation.from_quat(self.state[3:7])
        canopy = Rotation.from_quat(self.state[16:20])
        return dict(
            schema='microagu.state.v1', time_s=self.time, released=self.released,
            state=self.state.tolist(), rpm=float(self.rpm), voltage_v=self.voltage,
            brake_mm=self.actuator.position.tolist(),
            throttle=float(self.command[0]),
            joint_rotvec_rad=(payload.inv()*canopy).as_rotvec().tolist(),
            joint_rate_rad_s=payload.inv().apply(self.state[23:26]-self.state[10:13]).tolist())

    def respond(self, data):
        rate, frame, pwm = decode_packet(data)
        if frame == self.frame:
            return self.last  # retransmission must not advance physics
        if self.frame is not None and frame != (self.frame+1) % 2**32:
            raise RuntimeError('SITL frame discontinuity; restart backend with SITL')
        dt = 1./rate
        before = payload_observation(self.state, self.sim.geometry)
        if self.released:
            raw = np.array([pwm[0], pwm[2], pwm[3]])
            bounded = np.clip(raw, [1000, 800, 900], [2000, 2100, 2400])
            self.clipped_packets += int(np.any(raw != bounded))
            command = self.sim.from_pwm(*bounded)
            self.command = command
            # Maintain <=1.25 ms integration even if the requested frame rate is low.
            self.sim.substeps = max(1, int(np.ceil(dt/.00125)))
            self.state, self.rpm, _ = self.sim.step_actuated(
                self.state, self.rpm, self.actuator, command,
                voltage=self.voltage, dt=dt)
        else:
            # Straight, level trim translated through space while EKF initializes.
            # No fake ground contact or paused timestamp. Release is explicit.
            delta = self.state[7:10]*dt
            self.state[:3] += delta
            self.state[13:16] += delta
        obs = payload_observation(self.state, self.sim.geometry)
        acceleration = (obs['velocity_ned_m_s']-before['velocity_ned_m_s'])/dt
        rotation = Rotation.from_quat(self.state[3:7])
        specific_force = rotation.inv().apply(acceleration-[0, 0, 9.80665])
        self.time += dt
        message = dict(timestamp=self.time,
                       battery=dict(voltage=self.voltage),
                       imu=dict(gyro=obs['body_rates_rad_s'].tolist(),
                                accel_body=specific_force.tolist()),
                       position=obs['position_ned_m'].tolist(),
                       attitude=obs['euler_rad'].tolist(),
                       velocity=obs['velocity_ned_m_s'].tolist(),
                       airspeed=obs['true_cg_airspeed_m_s'],
                       velocity_wind=[0., 0., 0.])
        self.last = ('\n'+json.dumps(message, allow_nan=False)+'\n').encode()
        self.frame = frame
        return self.last


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flight', choices=['hood_river', 'trout_lake'], default='trout_lake')
    parser.add_argument('--profile', choices=['mission', 'dynamic_roll'], default='mission')
    parser.add_argument('--port', type=int, default=9002)
    parser.add_argument('--altitude', type=float, default=100.)
    parser.add_argument('--voltage', type=float, default=22.2)
    parser.add_argument('--viewer-port', type=int, default=0,
                        help='Optional localhost truth-state UDP port (0 disables)')
    parser.add_argument('--viewer-hz', type=float, default=30.)
    args = parser.parse_args()
    if not 0 <= args.viewer_port <= 65535 or not 0 < args.viewer_hz <= 100:
        parser.error('viewer port must be 0..65535 and rate must be 0..100 Hz')
    backend = Backend(args.flight, args.profile, args.altitude, args.voltage)
    def release():
        input('Trim ready. Start SITL, initialize/arm/select mode, then press Enter to release.\n')
        backend.released = True
        print('Physics released to native controller PWM.', flush=True)
    threading.Thread(target=release, daemon=True).start()
    viewer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.viewer_port else None
    if viewer is not None:
        viewer.setblocking(False)
    next_viewer_time = 0.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', args.port))
        print(f'Listening on localhost:{args.port}', flush=True)
        while True:
            data, address = sock.recvfrom(2048)
            sock.sendto(backend.respond(data), address)
            if viewer is not None and backend.time >= next_viewer_time:
                next_viewer_time = backend.time + 1./args.viewer_hz
                try:
                    viewer.sendto(json.dumps(backend.viewer_state(), allow_nan=False).encode(),
                                  ('127.0.0.1', args.viewer_port))
                except OSError:
                    pass  # A missing/slow viewer must never stop lockstep physics.


if __name__ == '__main__':
    main()
