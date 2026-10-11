# Gust-response controller study

The articulated flight viewer exposed a forced longitudinal oscillation that
route-completion checks did not measure. The gust study keeps the calibrated
Trout Lake model, mass, brake trim, motor, voltage compensation and lateral
controller fixed, and shapes the body pitch-rate feedback into throttle.

## Implementation

`TECS_PG_PR_FILT2` is an optional second first-order low-pass after
`TECS_PG_PR_FILT` and before `TECS_PG_PR_WASH`. Its generic default is zero
(bypass). Reset and unhealthy-AHRS paths clear both low-pass states. The existing
pitch gain, correction clamp and final throttle limits are unchanged.

Selected SITL candidate:

```
TECS_PTCH_DAMP 0.1
TECS_PG_PR_FILT 1.5
TECS_PG_PR_FILT2 1.5
TECS_PG_PR_WASH 0
```

Each cutoff is per stage. In continuous-time notation the additional throttle
fraction is `-0.1 * [2*pi*1.5/(s+2*pi*1.5)]^2 * q`, with body pitch rate q in
rad/s. Actual code uses the measured update interval in each discrete filter.
The existing correction clamp is +/-0.25 throttle fraction.

This is frequency shaping, not a conclusion that the original flight damper's
broadband sign was correct. Both phase and gain vary across the coupled modes.

## Evidence and reproduction

Research and plot scripts live in the sibling ArduPilot-tinkering repository:

- `tools/gust_control_study.py`: matched nonlinear truth-state screen, native
  throttle algebra, unchanged lateral loop and a repeatable gust history.
- `tools/gust_linear_check.py`: saved seven coupled plants, lateral airspeed
  scaling 0.9/1.15/1.4, single-stage and two-stage sensitivity checks.
- `tools/run_gust_native.sh`: unchanged full native AUTO mission in gusts or calm.
- `tools/summarize_gust_mission.py`: BIN parameter verification, truth-derived
  settled-loiter metrics, mission errors and source hashes.
- `tools/plot_gust_control.py` and `tools/plot_gust_filter.py`: research report
  and final figures. The blog explanation is `docs/longitudinal-observer.md`.

Raw output is under Mission Planner's
`analysis/longitudinal-2026-10-09/turn-model-spike/gust-control`. No raw flight BIN,
telemetry, or simulation state histories are added to either repository.

Build once with `./waf plane`, then run the native missions sequentially (both
use port 9002):

```sh
# Activate the existing ArduPilot Python environment first.
TK=/path/to/ArduPilot-tinkering
AP=/path/to/ardupilot-high-dof
OUT=/path/to/new/research/output
bash "$TK/tools/run_gust_native.sh" "$AP" "$OUT/gust" "$AP/Tools/simulation/paraglider/mission-controller-candidate.json" gust
bash "$TK/tools/run_gust_native.sh" "$AP" "$OUT/calm" "$AP/Tools/simulation/paraglider/mission-controller-candidate.json" calm
python "$TK/tools/summarize_gust_mission.py" "$OUT/gust"
python "$TK/tools/summarize_gust_mission.py" "$OUT/calm"
```

The gust field is uniform smooth random-phase multisine wind, with nominal
N/E/D RMS amplitudes 0.5/0.5/0.25 m/s. This is a repeatable synthetic disturbance,
not a measured atmospheric spectrum. A separate screen uses seed 417 and 1.5x
amplitudes. Native validation includes the EKF and original route, corner,
altitude, opposing-loiter and mode-handover checks without changing their gates.

Matched screening uses seconds 15-80; gusts stop at 80 s and the run continues
to 120 s. Native loiter statistics begin 20 s after entering 60 m of each loiter
center and end at that mission item's last sample. Independently aligned native
runs sample different gust phases; the matched screen is the causal comparison.

These are simulator development parameters. The model and synthetic gusts do
not establish real-aircraft robustness or certify launch/landing behavior.

## Completed native validation (11 October 2026)

The selected controller passed the unchanged `Plane.ParagliderAutoMission` test
in both gusts and calm air. Both BINs confirm `TECS_PTCH_DAMP=0.1`,
`TECS_PG_PR_FILT=1.5`, `TECS_PG_PR_FILT2=1.5`, and `TECS_PG_PR_WASH=0`.
Neither backend recorded errors or clipped PWM packets.

| Settled gust loiter | CW before / after | CCW before / after |
|---|---:|---:|
| Payload pitch std (deg) | 9.897 / 6.624 | 11.131 / 6.594 |
| Climb RMS (m/s) | 0.701 / 0.425 | 0.779 / 0.432 |
| Height RMS error (m) | 0.499 / 0.408 | 0.524 / 0.435 |
| Throttle std (percentage points) | 1.688 / 2.900 | 1.802 / 2.810 |

Gust mission maximum waypoint altitude error was 2.263 m (baseline 2.656 m),
with maximum corner overshoot 1.390 m (baseline 3.503 m). Calm mission maxima
were 1.708 m and 0.356 m, versus 1.669 m and 0.645 m before the change. Calm
settled pitch std decreased from 0.213/0.325 deg to 0.098/0.065 deg. CW calm
height RMS increased slightly, 0.055 to 0.064 m; CCW decreased 0.098 to 0.080 m.

A first-order 0.5 Hz candidate also passed both native missions, but was not
selected because of a faster-mode instability in the wider local linear check.
The selected two-stage design was locally stable across all seven saved plant
cases and three lateral scaling values. Low-voltage and softer-roll validation
in this follow-up is linear only; the new full native runs use 22.2 V and the
mission profile. Larger gains with 0.75 Hz stages increased pitch-rate motion.

Native raw truth SHA-256:

- Gust: `53774d5121e5a58501be7b390c6ee1d771b78059ecfc907ca6ddf44ffbfdc7df`
- Calm: `18f37a862269716a88a40e9deade53cac5ce6c4a442eb817a06b412684dee448`

Final raw run folders are `gust-control/two-pole-native-gust` and
`gust-control/two-pole-native-calm`. Earlier one-stage results are in
`gust-control/native-gust` and `gust-control/native-calm`. The selected nonlinear
screen is `two-pole-cutoff-check/two_pole_hz_150.npz`, compared with
`screen/baseline.npz`; the stronger, alternate-phase validation is
`selected-validation`. The simulator's physical parameters were not changed.
