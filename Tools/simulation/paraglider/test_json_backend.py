"""Protocol checks including a real physics step and duplicate delivery."""
import json
import struct
import unittest
import numpy as np
from json_backend import Backend, decode_packet

class ProtocolTests(unittest.TestCase):
    def test_packet_validation(self):
        for count, magic in [(16,18458),(32,29569)]:
            p=struct.pack('<HHI'+'H'*count,magic,400,7,*([1500]*count))
            self.assertEqual(decode_packet(p)[:2],(400,7))
        for p in [b'', bytes(40), struct.pack('<HHI16H',18458,0,0,*([0]*16))]:
            with self.assertRaises(ValueError):decode_packet(p)

    def test_physics_and_retransmission(self):
        b=Backend(voltage=20.4)
        pwm=[1000]*16
        pwm[0]=round(1000+1000*b.trim['command'][0])
        pwm[2]=round(800+b.trim['command'][1]/(np.pi*55/360*.1))
        pwm[3]=round(2400-b.trim['command'][2]/(np.pi*55/360*.1))
        packet=lambda frame:struct.pack('<HHI16H',18458,400,frame,*pwm)
        initial=json.loads(b.respond(packet(0)))
        self.assertEqual(initial['battery']['voltage'], 20.4)
        np.testing.assert_allclose(np.linalg.norm(initial['imu']['accel_body']),9.80665,atol=1e-8)
        self.assertLess(initial['position'][2],-99)
        b.released=True
        for i in range(1,401):
            reply=b.respond(packet(i))
            self.assertEqual(reply,b.respond(packet(i)))
        truth = b.viewer_state()
        self.assertEqual(truth['schema'], 'microagu.state.v1')
        self.assertEqual(truth['time_s'], b.time)
        self.assertEqual(truth['state'], b.state.tolist())
        self.assertEqual(truth['brake_mm'], b.actuator.position.tolist())
        self.assertEqual(truth['throttle'], b.command[0])
        self.assertEqual(truth['voltage_v'], 20.4)
        # Export is read-only and serializable, including after duplicate delivery.
        snapshot = b.state.copy()
        json.dumps(b.viewer_state(), allow_nan=False)
        np.testing.assert_array_equal(snapshot, b.state)
        from scipy.spatial.transform import Rotation
        rp = Rotation.from_quat(b.state[3:7])
        rc = Rotation.from_quat(b.state[16:20])
        np.testing.assert_allclose(truth['joint_rotvec_rad'], (rp.inv()*rc).as_rotvec())
        np.testing.assert_allclose(truth['joint_rate_rad_s'], rp.inv().apply(b.state[23:26]-b.state[10:13]))
        final=json.loads(reply)
        self.assertEqual(final['battery']['voltage'], 20.4)
        self.assertAlmostEqual(final['timestamp'],1.0025)
        self.assertEqual(b.clipped_packets,0)
        self.assertLess(abs(final['velocity'][2]),.1)
        self.assertGreater(final['position'][0],initial['position'][0]+2)
        with self.assertRaises(RuntimeError):b.respond(packet(500))

if __name__=='__main__':unittest.main()
