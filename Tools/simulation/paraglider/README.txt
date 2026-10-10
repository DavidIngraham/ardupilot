HIGHER-DOF PARAFOIL RESEARCH CANDIDATE v3
10 October 2026

Purpose
-------
A standalone two-body airborne research simulator for comparing ordinary mission
controller architectures. This is a development model, not a flight-qualified
controller or a validated prediction of aggressive maneuver limits.

Build and run (Ubuntu / WSL)
---------------------------
Dependencies: Python 3, NumPy, SciPy; g++ with Eigen3 headers. Matplotlib is used by
optional plotting scripts. The included .so was built on Ubuntu 24.04; rebuild
on another compatible Linux installation.

  sh build.sh
  python3 verify.py
  python3 simulator.py --flight trout_lake --seconds 120 --output demo.npz

Public API
----------
  from simulator import Simulator
  from trim import find_trim
  sim = Simulator('trout_lake')
  trim = find_trim(sim, climb_m_s=0, turn_deg_s=0)
  state, rpm = trim['state'], trim['rpm']
  command = sim.from_pwm(1400, 1000, 2200)
  state, rpm = sim.step(state, rpm, command)

Commands are [throttle fraction, LEFT PHYSICAL DRUM TRAVEL mm, RIGHT PHYSICAL
DRUM TRAVEL mm]. from_pwm returns those same units and composes with step/replay.
controls() is the explicit INTERNAL conversion to effective aerodynamic metres;
do not feed its output into step/replay. Slack/preload is applied exactly once.

replay(state, Nx3_commands, ...) returns (states, rpm). The first output is the
initial state; command i acts over interval i to i+1. The last command has no
following integration interval. step() advances one 0.04 s interval by default.

replay_actuated and step_actuated add a bounded servo travel rate. The default
221.5 mm/s follows the listed DS339HV 7.4 V speed and the nominal 55 mm drum
mapping. It is a diagnostic rate bound, not an identified loaded actuator model.
The applied command is its interval-average position. For manual closed-loop
integration keep one BrakeActuator object and pass it to step_actuated.

Frames and state
----------------
World NED; body forward/right/down; quaternions xyzw. SI units except physical
brake-command millimetres. State indices:
  0:3 payload position, 3:7 quaternion, 7:10 world velocity,
  10:13 WORLD angular velocity;
  13:16 canopy position, 16:20 quaternion, 20:23 world velocity,
  23:26 WORLD angular velocity.
Body gyros require rotating world angular velocity into the payload frame.
Payload IMU velocity differs from CG velocity by omega cross the IMU lever arm.
No sensor noise, EKF, navigation or actuator-position measurement is included.

Nominal and alternatives
------------------------
Default model='modal': approximate CAD-informed component inertia/arms, retained
panel-aerodynamic correction, calibrated rotor momentum, mass-dependent yaw
spring, physical APC 9x4.5 torque proxy, spin-acceleration reaction, and separate
HR/TL effective rigging configurations.

  Simulator('hood_river')  # 1.385 kg AUW, left deadband 12.15 mm,
                          # right effective preload 6 mm
  Simulator('trout_lake')  # 1.758 kg AUW, zero nominal deadband/preload
  Simulator('hood_river', right_preload_mm=8)  # alternative trim hypothesis
  Simulator('trout_lake', right_preload_mm=2)  # unresolved early/late bias
  Simulator('trout_lake', roll_stiffness=1)    # softer-joint sensitivity
  Simulator('trout_lake', model='aero')        # earlier aero comparator
  Simulator('trout_lake', yaw='geometry')     # conservative nonlinear yaw option

Effective preload adds aerodynamic brake deformation even at minimum PWM. It
does not increase servo travel. The inferred value can absorb canopy asymmetry,
rigging zero and other model error; it is not a measured line length. Alternative
values are competing hypotheses, not confidence intervals. Never interpolate
rigging solely from aircraft mass.

Per-flight logged PWM mapping (both analyzed main flights)
---------------------------------------------------------
Throttle S1 1000..2000, trim1000, reversed0, function70.
Left S3 800..2100, trim800, reversed0, function190.
Right S4 900..2400, trim2400, reversed1, function191.
No in-flight changes to those logged values were found.
Nominal drum conversion = pi*55/360*0.1 mm/us:
  left = (PWM3-800)*conversion
  right = (2400-PWM4)*conversion
Thus configured physical strokes are about62.4 mm left and72.0 mm right. The
asymmetry is real. The supplied90 mm figure refers to full physical travel,
not either configured PWM range.

Mechanics and limits
--------------------
Two rigid bodies joined by an ideal point constraint; three relative rotations
remain. A curved five-panel canopy generates aerodynamic forces/moments. The
constraint is enforced at acceleration level, with small RK4 drift. It can
transmit compression; line slack/collapse and distributed flexible suspension
are not modeled. The canopy is rigid and the stall law is only a surrogate.

Joint stiffness acts through the SO(3) logarithm with the corresponding Jacobian
transpose. The optional geometric yaw potential reduces to a mass-dependent
spring at small twist; it uses supported weight, not an ad-hoc instantaneous
load multiplier. It is not a full distributed line-tension model.

Propeller reaction torque, angular momentum/precession and rotor-acceleration
reaction are separate terms. Rotation is clockwise viewed from behind, as
reported. The APC9x4.5 map is a proxy for the actual9x4.5 propeller on the FFTY
3115-900KV motor. HR current readings are excluded as known erroneous; the
missing avionics input capacitor was fixed before TL.

The rotor inertia and effective canopy attachment spacing are calibrated
parameters, not direct measurements. Native rate spectra support coupled
pitch/yaw motion, but oscillation amplitudes—especially roll—remain too small.
Damping and external excitation are not uniquely identified.

Trim and controller use
-----------------------
find_trim first generates a seed, then solves a relative equilibrium and reports
its residual and a continuation check. A small residual proves numerical force/
moment balance, not dynamic stability. Small disturbance tests are separate.
The nominal HR powered-climb equilibrium has a weak slow instability in this
model; its static brake/turn slope can reverse. Do not blindly interpolate that
branch into a monotonic control inverse. TL and level-HR tested equilibria recover
from the small disturbances studied.

controllers.py is a diagnostic GPL-3.0-or-later core port from ArduPilot commit
b2f6da3a183a68d3de6f2c721ec90fca0eab53d5, ArduPlane/PG_TurnController.cpp.
It excludes navigation/EKF and is not a production controller deployment.
Use the public Simulator methods; raw dynamics.derivative/replay are legacy
internal helpers and do not select the causal RPM/propeller path used here.

Evidence and reproducibility
----------------------------
The parent report.html and research log contain comparison plots, scripts,
flight-window choices and parameter provenance. Flight results reuse the two
available flights; retrospective development splits are not independent flight
validation. Baseline, candidate-v1 and candidate-v2 remain alongside this package.
No production firmware or blog content was changed by this research study.

Controller demonstration
------------------------
  python3 mission_demo.py --flight trout_lake --case nominal --output tl-demo.npz
  python3 mission_demo.py --flight hood_river --case soft_roll --output hr-soft.npz

Each demonstration runs240 seconds with0,+5,+10,0,-5,-10,0,0 deg/s requests,
and a0.5m/s climb request from90 to150 seconds. The controller uses ideal payload
heading/rate observations and true airspeed. There is no wind, turbulence, EKF,
navigation delay, or sensor noise in this demonstration. Its feedforward table
comes from this model's level equilibria; a throttle-dependent straight-trim
correction uses the level and powered straight equilibria, including the weakly
unstable HR powered equilibrium. Feedback stabilizes the tested cases. This is
an architecture experiment, not independent evidence of real flight capability.

Available plant cases: nominal, rigging_low, rigging_high, soft_roll, low_damping,
rotor_low, rotor_high, slow_servo. These are discrete engineering hypotheses,
not statistical confidence bounds. Feedback and nominal feedforward stay fixed
across cases. A separate soft_retrim experiment lives in the parent study; the
soft_roll demo intentionally leaves the nominal rigging unchanged.

The .npz includes time, full state and named observation/command columns. The
.json reports course/heading/climb errors, normalized-controller saturation and
physical actuator clipping separately. Physical left clipping can begin before
normalized output reaches-1 because the allocator compensates left deadband.
These diagnostics do not implement allocator-aware integral anti-windup.

Reproduction in the original study folder
----------------------------------------
  python3 ../verify_retained_replays.py
This replays all98 stored ordinary windows with the public v3 API and checks
agreement with the archived evidence. It requires the parent flight-derived
.npz files; they are not included in the portable simulator archive. Source and
artifact SHA256 hashes plus dependency versions are recorded in the parent
reproducibility-manifest.json. See report.html for the evidence index.

Co-retained dynamic-response candidate
-------------------------------------
  from profiles import make_simulator
  sim = make_simulator('trout_lake', 'dynamic_roll')
  python3 mission_demo.py --flight trout_lake --case dynamic_roll --output dynamic.npz

The mission profile is the Simulator default. The dynamic_roll profile uses
roll stiffness1 N m/rad and a model-derived correction that matches nominal
straight-level differential trim: HR effective right preload5.4823mm; TL right
deadband0.9302mm. It has the same damping and rotor calibration. This correction
uses model equilibria, not a second fit to flight residuals. It changes unilateral
brake release and is not a purely algebraic removal of trim.

Both profiles are retained. mission is stronger on ordinary trajectory fit.
dynamic_roll is substantially closer in high-frequency roll amplitude and peak
location, while still improving ordinary course error over the starting model.
Cross-axis phase errors and missing low-frequency yaw energy remain. Its powered
HR instability is stronger. Use BOTH for attitude/rate-controller conclusions;
matching RMS is not proof that the underlying mode has been uniquely identified.

The dynamic_roll factory rejects conflicting roll_stiffness/right_preload_mm
keywords and a non-modal mechanics base. For a separate sensitivity experiment,
create the profile first and then change its parameter dictionary explicitly.
