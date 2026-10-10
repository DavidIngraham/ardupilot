# High-DOF parafoil control development

This branch combines the native ArduPlane heading-rate controller and paraglider
TECS from `research/paraglider-rebase` (`b2f6da3a18`) with the retained 10 October
2026 two-body model, candidate v3. The original `paraglider-dev` checkout is not
modified. Its local TECS spelling fix is unnecessary on this base: that code has
already been rewritten.

## What runs where

* Native flight code: `ArduPlane/PG_TurnController.cpp` and
  `libraries/AP_TECS/AP_TECS_Paraglider.cpp`, including existing navigation and
  mode handover. These are inherited from the controller-development branch.
* New physics: this directory, compiled as an Eigen shared library and run through
  ArduPilot's existing JSON backend. It is not a replacement for SIM_Paraglider.cpp.
* `mission` is the retained mission-performance profile; `dynamic_roll` is the
  alternative with stronger observed roll motion. Both support HR and TL mass and
  rigging configurations.
* `mission_demo.py` and `controllers.py` preserve the newer trim-table and power
  bias experiments. Their scheduled feedforward is **not** in the native flight
  controller. The native preset uses conservative feedback gains and constant FF.

## Build (Ubuntu / WSL)

From the repository root, with g++, Eigen3 and the Python requirements installed:

```sh
git submodule update --init --recursive
./waf configure --board sitl
./waf plane -j4
cd Tools/simulation/paraglider
sh build.sh
python3 verify.py
python3 -m unittest test_json_backend -v
```

## Run with native controllers

First terminal, in this directory:

```sh
python3 json_backend.py --flight trout_lake --profile mission
```

Second terminal, from the repository root:

```sh
Tools/autotest/sim_vehicle.py -v ArduPlane -f paraglider --model JSON:127.0.0.1 \
  --add-param-file Tools/autotest/default_params/paraglider.parm \
  --add-param-file Tools/simulation/paraglider/sitl.parm --console
```

Use a fresh SITL parameter store (or `-w` to reset that SITL store). The overlay must
follow paraglider.parm: throttle is output 1, left brake 3, right brake 4, with the
logged asymmetric strokes and reversed right output. Never load this research
preset onto an aircraft as a flight setup.

The model starts in straight level trim 100 m above the reference, moving while
SITL initializes. Prepare the desired controller mode and arm in SITL, then press
Enter in the backend terminal to release physics to native PWM. This initialization
is explicit: no ground, toss, landing or canopy inflation model is implemented.
Restart both processes for a new run. Native controller tracking after release
still requires tuning; the standalone controller demonstration is not evidence of
native AUTO mission performance.

The adapter sends payload IMU position, velocity, body gyro and specific force,
NED position, Euler attitude, and true airspeed. Acceleration is the finite
difference of IMU velocity, including the modeled lever arm. Wind is currently
zero and voltage constant. Duplicate UDP frames resend their cached sensor state;
frame gaps/restarts fail explicitly. Out-of-range startup PWM is bounded to the
logged ranges, with a counter exposed as `clipped_packets`. Both JSON servo packet
sizes are supported. Timing follows SITL, with physics substeps at most 1.25 ms.

## Research controller comparison

```sh
python3 mission_demo.py --help
```

See README.txt for mechanics, state definitions, calibration and limitations.
SOURCE-NOTES.txt and configuration-provenance.json retain source provenance.
Generated libraries, histories and verification output are ignored by Git.
The early TL turn bias and powered HR equilibrium remain model limitations;
these profiles are useful competing development models, not identified truth.
