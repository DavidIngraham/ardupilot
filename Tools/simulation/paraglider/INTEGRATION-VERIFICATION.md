# Integration verification — 10 October 2026

- Fresh Eigen dynamics build: passed.
- `python3 verify.py`: passed for HR and TL; step/batch results identical,
  joint position error below 6.3e-8 m, servo interval averaging and preload checks passed.
- `python3 -m unittest test_json_backend -v`: passed. Covers both packet sizes,
  malformed input, duplicate frames, skipped frames, level-trim specific force,
  NED altitude, and one second of real physics at 400 Hz.
- Fresh `./waf configure --board sitl` and `./waf plane -j12`: passed.
- `python3 smoke_sitl.py`: 800 real native SITL packets exchanged; JSON parser
  accepted all provided sensor fields and firmware loaded both parameter files.
  At the native 1200 Hz rate this is 0.667 s, with physics released halfway.
  Startup/disarmed PWM required range bounding on all 400 released packets.
  This is a transport check, not an armed controller or AUTO flight test.
- Standalone 240 s TL mission demonstration reproduced course-rate RMSE
  0.4438076 deg/s and climb RMSE 0.0667105 m/s, with zero normalized saturation
  and physical actuator clipping. This exercises the archived scheduled controller,
  not the native constant-FF controller.

Next control-development test: initialize EKF, arm, release in a controlled mode,
then test native closed-loop steps and AUTO guidance on both model profiles.
Do not infer native tracking quality from the standalone demonstration.

## Full native AUTO comparison, 10 October 2026

The unchanged extended AUTO checks passed for:

| Model / gains | Logged AUTO duration | Max WP height error | Max corner overshoot | First-loiter bank SD |
| --- | ---: | ---: | ---: | ---: |
| Original model / original gains | 1063 s | 1.92 m | 0.42 m | 0.051 deg |
| High DOF mission / original gains | 1397 s | 1.81 m | 0.20 m | 2.60 deg |
| High DOF mission / lower gains | 1292 s | 2.51 m | 3.35 m | 19.36 deg |

The original archive also passed: 1062 s, 1.84 m height error, 0.31 m corner overshoot.
Bank SD uses the last 30 s of the first loiter. The lower gains are smoother in a
straight segment but produce large sustained-turn oscillations. They should not
be called an improved native tuning merely because both missions pass.
The original gains are the better comparison reference. No physics/gain tuning
was performed. Model, actuator calibration, throttle cap and pitch-damper settings
differ from the original simulator as detailed in README.md.

The harness fixes initialization order, reuses the existing route and assertions,
and records actual output channels by function. An earlier run entered FBWB before
EKF readiness and captured an invalid altitude target; it is excluded. Full results
and plots are archived in Mission Planner analysis, turn-model-spike/native-sitl-comparison.
Raw logs are in /home/david/repos/pg-sitl-comparison. These tests validate nominal
native integration, not real minimum turn radius or the untested dynamic-roll profile.
