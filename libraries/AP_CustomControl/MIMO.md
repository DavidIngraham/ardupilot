# Coordinated microAGU control

AI-assisted implementation. `CP_TYPE=3` selects the experimental microAGU LQI
backend. Auxiliary function 109 engages/disengages it without rebooting. Selecting
the backend type still requires a reboot. `CP_MASK` does not split the MIMO
actuator vector: throttle and both brakes always change owner together.

## Vehicle contract

`AP_CustomControl_MIMO.h` defines timestamped guidance, estimated state, actuator
authority, command and allocation records. Coordinates use SI units, NED vectors,
body FRD rates, positive-up height relative to home, and an explicit EAS target
and EAS-to-TAS conversion. Height and climb-rate reference types are distinct.
Plane currently supplies its upstream height demand, including FBWB/CRUISE pilot
changes, before TECS performs its own reference shaping. The heading-rate demand
comes from the existing lateral guidance conversion.

Backends implement `engage`, `update_mimo`, `allocation_feedback` and `disengage`.
The frontend rejects an invalid command vector atomically. The existing PID
backend retains its independent-axis API and behavior.

The MIMO request has three normalized entries: throttle, common brake and signed
differential brake. Allocation constrains physical left/right brake travel and
slew, then writes the dedicated brake servo functions. It bypasses the legacy
aileron/elevator mixer, retaining servo endpoint/reversal calibration. Throttle
still passes through Plane suppression and output limits. Feedback reports the
final software actuator demand, not measured servo position. The Rust model's
actuator dynamics remain separate.

PG heading control and TECS continue updating in the background. Their output
matching tracks the allocated steering/throttle; common brake decays on return
to an eligible baseline mode. MANUAL and unsupported flight stages take normal
vehicle ownership immediately. TECS tracking bounds its integral and releases a
temporary matching residual rather than resetting TECS on every active cycle.

## Initial backend

The fixed 50 Hz model has 16 physical states and three integral errors (height,
heading rate, airspeed). A linear predictor/corrector estimates the unmeasured
canopy states from EKF payload attitude, body gyro, air-relative velocity and
height. It never reads simulator truth. Generated matrices come from microAGU's
current Rust `as_built` model at 132 mm riser spacing, 22.2 V and a 10% common-brake
trim. This is a new three-input design, not the archived two-input scheduled LQI.

Model-derived equilibrium feed-forward covers turn, climb and airspeed demands.
The local reference governor limits climb to +/-0.5 m/s, heading rate to +/-10
deg/s and TAS to 3.8–4.4 m/s. Allocation residuals feed back through the full
integral gain matrix, so a saturated common brake does not freeze heading control.

The initial implementation uses a local fixed model, not voltage/gain scheduling
or a nonlinear onboard predictor. Its noise weights are design assumptions,
not a flight-identified sensor covariance. It is enabled by default only in SITL
(including WASM), and is not a flight-qualified tune. No superiority claim follows
from local eigenvalue or smoke checks.

Eligibility requires armed normal flight, automatic throttle, valid heading
guidance, valid attitude/navigation/wind/airspeed estimates, and no RC failsafe or
throttle suppression. Launch, landing, MANUAL and FBWA retain baseline control.
Stale inputs, estimator resets, scheduling stalls or invalid output cause return
to baseline; valid conditions allow re-engagement with output matching. The
`MIMO` named telemetry value reports actual ownership separately from selection.

## Regeneration and validation

In microAGU:

```sh
simulation/.venv/bin/python simulation/tools/design_mimo.py --output /tmp/microagu-mimo
```

Copy the generated `AP_CustomControl_LQI_gains.h` into this library only after
reviewing the design report and rerunning validation. Reports include generator,
installed Rust extension and calibration hashes.

Native validation uses `tests/test_mimo` plus Plane's `ParagliderMIMO`,
`ParagliderCustomControl` and `ParagliderLanding` autotests. microAGU's
`viewer/tests/integration/mimo.ts` exercises packaged WASM, the real EKF, full
Rust physics, ownership and mode changes. `control-comparison.ts` runs a paired
AUTO weather scenario and records metrics and samples; it is a diagnostic
comparison, not statistical flight evidence.
