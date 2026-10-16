#!/usr/bin/env python3

'''
Regression tests for paraglider frame configuration lookup.

AP_FLAKE8_CLEAN
'''

import unittest

from types import SimpleNamespace
from unittest.mock import patch

from pysim.vehicleinfo import VehicleInfo


class TestParagliderVehicleInfo(unittest.TestCase):

    def assert_frame_options(self, frame, model=None, build_target=None):
        opts = SimpleNamespace(model=model, build_target=build_target)
        with patch('builtins.print') as mock_print:
            options = VehicleInfo().options_for_frame(frame, 'ArduPlane', opts)

        mock_print.assert_not_called()
        self.assertEqual(options['default_params_filename'], 'default_params/paraglider.parm')
        self.assertEqual(options['model'], model if model is not None else frame)
        self.assertEqual(options['waf_target'], build_target if build_target is not None else 'bin/arduplane')
        self.assertTrue(options['sitl-port'])

    def test_exact_frame(self):
        self.assert_frame_options('paraglider')

    def test_throw_variant(self):
        self.assert_frame_options('paraglider-throw')

    def test_tow_variant(self):
        self.assert_frame_options('paraglider-tow')

    def test_json_model_variant(self):
        self.assert_frame_options('paraglider:custom.json')

    def test_throw_json_model_variant(self):
        self.assert_frame_options('paraglider-throw:models/custom.json')

    def test_explicit_model_override_preserves_defaults(self):
        self.assert_frame_options('paraglider', model='paraglider-tow:custom.json')

    def test_explicit_build_target_override_preserves_defaults(self):
        self.assert_frame_options('paraglider-throw', build_target='bin/custom-arduplane')


if __name__ == '__main__':
    unittest.main()
