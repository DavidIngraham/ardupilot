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
