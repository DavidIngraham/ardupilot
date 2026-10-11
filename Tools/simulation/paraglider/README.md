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
  controller. The native preset uses lower feedback gains and constant FF. The full AUTO comparison found large sustained-turn oscillations with that preset; use PG_GAINS=original in the comparison harness as the better native reference.

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
zero and voltage constant. The JSON battery voltage matches the motor model
voltage. The SITL overlay enables a voltage-only monitor and native forward
throttle compensation, referenced to the existing 22.2 V calibration (not a
25.2 V full-charge calibration). Use --voltage for the backend or PG_VOLTAGE
for the AUTO harness. Current and battery discharge are not modeled.
Duplicate UDP frames resend their cached sensor state;
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

## Compare with the original extended AUTO test

With the standard ArduPilot autotest Python environment active, run from the repo
root. Give each run its own BUILDLOGS directory; run them sequentially because
SITL and the JSON backend use the standard local ports.

```sh
BUILDLOGS=/tmp/pg-original Tools/autotest/autotest.py \
  --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission
BUILDLOGS=/tmp/pg-high-original PG_GAINS=original \
  python Tools/simulation/paraglider/compare_autotest.py \
  --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission
BUILDLOGS=/tmp/pg-high-conservative PG_GAINS=conservative \
  python Tools/simulation/paraglider/compare_autotest.py \
  --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission
```

The comparison wrapper uses the existing mission, checks and CSV measurements.
Only launch is replaced by a trimmed airborne start. It waits for EKF readiness,
arms, restores the ground home datum, then enters FBWB to capture a valid height
target before release. Entering FBWB before EKF initialization captures an invalid
altitude and commands a descent; that is a harness error, not a model result.
`PG_PROFILE=dynamic_roll` selects the alternative physics profile.

`PG_GAINS=original` restores the original six changed lateral parameters; the
flight-calibrated PWM ranges, throttle cap and disabled pitch damper remain.
This compares complete development configurations, not a physics-only ablation.
Partial failures retain the mission CSV where available and high-dof-trace.npz
with time, release flag and the 26-state plant history. A failed mission check
must not be interpreted as a general rejection of the model.

## Native pitch-rate damper sign experiment

From the repository root, with the autotest Python environment active:

```sh
BUILDLOGS=/tmp/pg-damper-off PG_DAMP_GAIN=0 python Tools/simulation/paraglider/damper_autotest.py --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderDamperSign
```

Repeat sequentially with separate BUILDLOGS directories and PG_DAMP_GAIN set to
0.3, -0.3, 0.02 and -0.02. The native equation is delta_throttle=-gain*q_filtered;
negative gains reverse the sign for this isolated PG-enabled SITL experiment.
Do not use this as a generic TECS aircraft parameter recommendation.
The harness reuses the airborne setup and leaves all other gains/model settings
fixed. It settles for 30 s with the damper off, enables the selected gain, applies
a 5 deg/s body-pitch-rate increment preserving joint velocity compatibility,
and records 75 s. The active lateral controller remains enabled. Plant truth is
sampled at 50 Hz; native output is retained in flight.BIN. Completion is not a
stability pass. The gain activates before the kick, so the full initial states
are not identical; the applied kick increment is.

Results on 10 October 2026 (45–75 s after kick): pitch-rate RMS off=16.01 deg/s,
original 0.30=61.19, reversed 0.30=55.94, original 0.02=21.37, reversed 0.02=12.66.
The reversed 0.30 case has a much larger slow pitch excursion (64.28 deg
peak-to-peak, dominant rate frequency 0.33 Hz), despite its slightly lower rate
RMS. Reversed 0.02 reduces the fast motion without saturation, but slightly raises
climb variability. No flight-code sign or default gain is changed by this study.
Full plots and methodology are in Mission Planner analysis,
turn-model-spike/native-damper-sign/report.html. Raw runs are in
/home/david/repos/pg-damper-study. Regenerate plots with plot_damper.py --runs-dir.

## Longitudinal isolation and frequency response

Set `PG_FIXED_BRAKES=1` in the damper harness to hold physical brake commands at
its solved equilibrium (0 and 3.7472 mm for nominal Trout Lake), starting before
release. Native TECS and the lateral controller still execute; the lateral
controller's brake outputs are bypassed at the plant input. The pitch kick and
all other settings remain the same. This is a diagnostic intervention, not a
flight configuration. The event JSON records the intervention and brake values.

Run off, +0.02 and -0.02 sequentially in separate output directories. Compare to
the previous active-brake runs using:

```sh
python3 Tools/simulation/paraglider/plot_longitudinal_isolation.py --active /home/david/repos/pg-damper-study --fixed /home/david/repos/pg-longitudinal-isolation
python3 Tools/simulation/paraglider/longitudinal_frequency.py --output /home/david/repos/pg-longitudinal-isolation/frequency
```

The frequency study linearizes the full causal-RPM model at straight level trim
in 15 joint-compatible tangent coordinates plus rotor RPM. It uses central
finite differences of the 5 ms discrete transition, with 1.25 ms RK4 substeps.
Outputs are body pitch rate and CG climb rate; physical brakes remain fixed.
Five nonlinear sinusoidal tests at each of two amplitudes (0.001 and 0.0005
throttle fraction) independently check the response from 0.1 to 3 Hz. The fit
uses the second half of each run, with a constant and linear trend included.
The 0.02–8 Hz plotted curve extends beyond those validation frequencies.
This characterizes the model locally; it does not establish aircraft frequency
response, include native EKF/controller delays, or provide closed-loop margins.

## Mission longitudinal candidate

The two-revision native controller study and reproduction commands are in
[LONGITUDINAL-STUDY.md](LONGITUDINAL-STUDY.md). Load
`mission-controller-candidate.json` through `PG_CONTROLLER_CONFIG` in the AUTO
comparison harness, or apply `mission-controller-candidate.parm` after the
existing SITL defaults. These are simulator development settings, not aircraft
flight tuning. The candidate uses calibrated feedforward and smooth altitude
captures with the pitch damper off. Generic defaults are unchanged.
