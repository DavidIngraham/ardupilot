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
captures with frequency-shaped pitch-rate damping. The [gust-control study](GUST-CONTROL.md) documents the two filter stages and native validation. Generic defaults are unchanged.

### Optional viewer truth-state output

Run json_backend.py with --viewer-port 9010 to send microagu.state.v1 JSON
frames to localhost over nonblocking UDP at 30 simulation Hz (--viewer-hz changes
the rate). Disabled by default. Includes both bodies' full state, relative joint
rotation/rate, actual brake position, throttle, RPM, voltage and release status.
No sensor/control behavior changes. The Three.js viewer, bridge, schema and replay
converter live in the ArduPilot-tinkering repository under viewer/.

## Experimental native landing brake response (13 October 2026)

Automatic brake flare is **disabled by default**. This implementation uses the
existing AP_Landing slope-landing state: LAND_FLARE_ALT, LAND_FLARE_SEC, existing
landing height/rangefinder handling, throttle suppression, go-around, and disarm.
It adds no independent AGL estimator or flare trigger.

The normal throttle-only controller flies the approach. On an armed AUTO landing
flare with both paraglider brake outputs assigned, the mixer ramps a common brake
floor onto both channels. Differential steering uses the remaining travel, and
the heading-rate controller includes that reduced travel in its anti-windup limit.
TECS stops accumulating throttle correction while landing suppresses thrust.
Abort, mode exit, safety engagement, or disarm clears the automatic common command.
Manual pitch-stick flare remains unchanged. Brake release on takeover is immediate
at the command level; the actuator still has its physical rate limit.

| Parameter | Default | Meaning |
| --- | --- | --- |
| PG_FLARE_PCT | 0 | Requested common brake percent; zero disables |
| PG_FLARE_RATE | 30 | Application rate in percentage points per second |
| PG_FLARE_RES | 20 | Travel reserved above common brake for differential steering |

Common brake is capped at 100 minus the reserve. Percent is normalized servo travel,
not calibrated equal millimetres on both lines: the logged left/right strokes differ.
Reserved travel also does not establish aerodynamic steering authority near stall.
These are experimental controls, not flight-qualified flare settings.

PGFL logs activation, common demand and allocated left/right brake percentages.
Existing LAND, TECS, PGYR and RCOU messages retain the landing/controller context.
Existing throttle slew limiting still applies when landing commands idle.

### Reproduce the studies

With the SITL Plane binary built and the simulator Python dependencies available:

    BUILDLOGS=/tmp/pg-landing-nominal PG_LANDING_CASE=nominal python Tools/simulation/paraglider/landing_autotest.py --no-check-parameter-leaks test.Plane.ParagliderLandingStudy
    BUILDLOGS=/tmp/pg-landing-native python Tools/autotest/autotest.py --no-check-parameter-leaks test.Plane.ParagliderLanding
    python Tools/simulation/paraglider/landing_report.py /tmp/pg-landing-nominal /tmp/pg-landing-weak /tmp/pg-landing-strong /tmp/pg-landing-crosswind /tmp/pg-landing-abort /tmp/pg-landing-takeover /tmp/pg-landing-disabled /tmp/pg-landing-timed /tmp/pg-landing-reserve --output /tmp/pg-landing-report

Run each case sequentially: the harness uses the standard SITL ports.
Weak/strong multiply CLbrake and CDbrake by 0.5/1.5; these are sensitivity assumptions,
not flight-derived flare fits. Crosswind applies 1.5 m/s lateral wind.
The ordinary test uses a 3 m altitude trigger and a 60 percent target at 30 percent/s.
Timed uses LAND_FLARE_SEC=3 and LAND_FLARE_ALT=0. Reserve requests 100 percent with
20 percent reserved and an 8 m trigger so the 80 percent cap is actually reached.
Abort sends the existing go-around command below 20 m; takeover enters MANUAL below
2 m with centered sticks and idle throttle. Disabled retains ordinary landing with
no common brake. The script writes telemetry, truth traces and first-contact metrics.

The two-body study ends at the first payload-CG crossing of a flat ground plane.
It has no contact, inflation, collapse, or validated high-brake stall model.
The separate native SITL test checks flare, throttle cutoff, ground handling and
automatic disarm; it does not establish landing quality of the two-body model.

Initial results: common brake reduced nominal contact sink speed from 1.63 to
1.41 m/s and ground speed from 3.96 to 3.06 m/s. Weak/strong cases contacted at
1.41/1.46 m/s down; crosswind at 1.47 m/s. The earlier 80 percent case contacted
at 1.54 m/s, so more/earlier brake was not automatically better. These are
first-contact model outputs, not predictions of a safe real landing.
The pre-flare go-around regained 31.3 m after the command below 20 m.
Log checks cover activation, bounds, ramp rate, reserve cap, takeover reset,
and idle throttle after the existing slew ramp.

Next work should compare flare timing and pull schedules, examine transient
sink increase after thrust removal, and measure common-brake lift/drag, stall
margin and retained steering aloft before selecting real-aircraft settings.

### Exploratory spot-landing Monte Carlo

The Monte Carlo runs the same native AUTO landing with the moderate flare held
fixed (60 percent common brake, 30 percent/s, 20 percent reserve, 3 m trigger).
It samples wind/gusts, approach position/heading/speed, voltage, independent brake
lift/drag multipliers, actuator response, and landing-datum error. The default
seed is 20261013: 24 moderate-flare trials and eight matched no-flare controls.

    python Tools/simulation/paraglider/landing_monte_carlo.py --output /path/to/research/logs --count 24 --pairs 8 --seed 20261013
    python Tools/simulation/paraglider/landing_monte_carlo_report.py /path/to/research/logs

Use the same Python environment and SITL build as the preceding landing study.
Run sequentially; each trial starts a fresh SITL process. The runner resumes
completed records in the selected directory and rejects a conflicting trial
manifest. Use a new directory when changing code or sampling assumptions.
The manifest and per-trial configurations preserve the exact draws. Failed or
incomplete trials remain visible; the report does not silently exclude them
from the attempted-trial denominator.

The harness records the uploaded landing coordinates and intended ground-plane
target, then interpolates first payload-CG ground crossing within the integration
frame. Along-track error is positive beyond the target and negative short;
cross-track error is east of the northbound approach. The report includes radial
error, 5/10/20 m counts, sink/ground speed, and paired changes relative to no flare.
The report's distributions are explicit engineering assumptions, not measured
environmental probabilities. No flight-controller code or tuning changes are
made by this experiment. Sparse samples cannot establish rare-event reliability.

For seed 20261013, all 24 moderate-flare trials reached the ground plane:
0/24 were within 5 m, 1/24 within 10 m, and 24/24 within 20 m.
Median radial miss was 15.27 m (90th percentile 17.05 m), with a mean
14.68 m short bias and 0.56 m 90th-percentile absolute cross-track error.
Median contact sink was 1.47 m/s. All eight no-flare controls also completed.
Matched mean changes with moderate flare were +0.26 m radial error,
-0.04 m/s sink speed and -0.79 m/s ground speed. The strongest observed
benefit was slower forward arrival, not a tighter landing footprint.

The tested LAND_FLARE_AIM=50 and TECS_LAND_SINK=0.25 remain unmodified.
The existing approach calculation uses them to estimate flare distance.
An overly long estimated flare duration is a plausible source of the short
bias; test that mechanism with matched settings comparisons and new held-out
draws rather than treating a post-hoc shift of these landing points as validation.

### One bounded landing-aim iteration

The follow-up screened LAND_FLARE_AIM=0 against TECS_LAND_SINK=1.4 on the first
four seed-20261013 conditions. The 1.4 m/s planning assumption was selected, with
LAND_FLARE_AIM=50 and the existing 60% / 30%/s / 3 m flare held fixed.
Twelve fresh seed-20261014 conditions then compared baseline and candidate.

    python Tools/simulation/paraglider/landing_monte_carlo.py --output /path/to/heldout-baseline --count 12 --pairs 0 --seed 20261014
    python Tools/simulation/paraglider/landing_monte_carlo.py --output /path/to/heldout-candidate --count 12 --pairs 0 --seed 20261014 --flare-sink 1.4 --record-viewer

All 24 held-out runs completed. Median radial miss improved from 13.30 to 1.73 m;
90th percentile from 15.34 to 3.77 m; within-5 m count from 0/12 to 12/12.
All 12 paired misses improved. Median sink was 1.42 versus 1.41 m/s; p90 sink
1.48 versus 1.55 m/s. This is a placement improvement, not evidence of softer impact.
The test retains the landing harness's original turn-gain preset.

The existing AP_Landing slope calculation mixes approach sink with
TECS_LAND_SINK to estimate flare time and upstream aim distance. The original
0.25 m/s assumption overestimated float time for this model. The candidate
changes that planning assumption; it does not command closed-loop sink control
during the fixed common-brake flare.

Use landing-spot-candidate.parm as an explicit simulation overlay. Firmware
PG_FLARE_PCT still defaults to zero. Full truth recording is opt-in and sampled
at approximately 20 Hz to viewer.json; it does not feed state into the controller.
The research article, held-out figure, log audits and presentation replay are in
the tinkering repository's docs/spot-landing.md and docs/assets/landing.
