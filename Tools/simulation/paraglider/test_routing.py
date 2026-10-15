# AP_FLAKE8_CLEAN
"""Physical invariants for the optional brake-route closure."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from simulator import Simulator
from routing import configure, diagnostic
from dynamics import brake_paths


class RoutingTest(unittest.TestCase):
    def test_reference_and_world_rotation(self):
        sim = Simulator()
        state = sim.initial_state()
        configure(sim, state)
        np.testing.assert_allclose(diagnostic(sim, state, [0.3, 0, 0])['delta_m'], 0, atol=1e-14)
        rotated = state.copy()
        rotation = Rotation.from_euler('xyz', [0.4, -0.2, 1.0])
        for index in [3, 16]:
            rotated[index:index+4] = (rotation*Rotation.from_quat(state[index:index+4])).as_quat()
        np.testing.assert_allclose(brake_paths(rotated, sim.parameters),
                                   brake_paths(state, sim.parameters), atol=1e-14)

    def test_slack_and_pitch_limit(self):
        sim = Simulator(left_slack_mm=12)
        state = sim.initial_state()
        configure(sim, state, ring_height=0)
        initial = brake_paths(state, sim.parameters)
        state[3:7] = Rotation.from_euler('y', 0.4).as_quat()
        np.testing.assert_allclose(brake_paths(state, sim.parameters), initial, atol=1e-14)
        sim.parameters['route_reference_left'] -= 0.020
        self.assertAlmostEqual(diagnostic(sim, state, [0.3, 0, 0])['effective_m'][0], 0.008)
        self.assertEqual(sim.controls(0.3, 0, 0)[1], -0.012)
        sim.parameters['routing_enabled'] = 0
        self.assertEqual(sim.controls(0.3, 0, 0)[1], 0)


if __name__ == '__main__':
    unittest.main()
