# Longitudinal controller development study

This study runs native Plane/TECS against the high-DOF Trout Lake model. It is
scoped to airborne mission control, not launch, landing, or aerobatics. The
maximum is two design revisions; model coefficients are held fixed.

## Reproduction

Build Plane with `./waf plane`. Activate the autotest Python environment. Run
these sequentially from the repository root (same UDP ports and log directory):

```sh
python Tools/simulation/paraglider/run_longitudinal_study.py Tools/simulation/paraglider/study-initial.json
python Tools/simulation/paraglider/run_longitudinal_study.py Tools/simulation/paraglider/study-revision1.json
python Tools/simulation/paraglider/run_longitudinal_study.py Tools/simulation/paraglider/study-revision2.json
python Tools/simulation/paraglider/analyze_longitudinal_study.py /home/david/repos/pg-longitudinal-design
```

Each named run saves its configuration, phase times, native BIN log, 50 Hz plant
truth, extracted signals and a response plot. Runs start in airborne trim at
150 m, settle, command climb and descent ramps followed by captures, then fly
opposing sustained turns. RC pitch 1700 commands climb and 1300 descent with
RC2_REVERSED=0 and FBWB_ELEV_REV=0. Use logged TECS demands, not an assumed exact
RC-to-climb conversion. FBWB captures the current altitude when pitch input
returns to neutral; the extended AUTO route is the separate altitude-waypoint
check. Test completion asserts armed FBWB execution, not good performance.

The first exploratory `baseline` run predates the RC pitch direction correction
and has mislabeled climb/descent phases. The comparison uses `baseline_matched`
from revision one instead. It uses the rebuilt firmware with washout disabled.

Ablation phases last 70 s and restore the original lateral gains before removing
one term. The original reference is repeated at the end. These are sequential
interventions, not identically initialized flights. Initial altitude transients
must not be compared as if matched. Report pitch and turn metrics over the last
20 s, with phase-wide altitude/climb tracking and throttle use.

## Design choices

Revision one reduces PG_TURN_P from 0.04 to 0.005 and RDAMP from 0.03 to 0.003,
while preserving the original 4 Hz feedback filter and other lateral settings.
It calibrates cruise throttle to 45% and TECS_CLMB_MAX to 1 m/s. Model trims at
22.2 V require 44.59% for level, 51.00% for 0.4 m/s, 57.02% for 0.8 m/s and
59.91% for 1 m/s. These settings fit the model's 60% throttle ceiling; they are
not new flight-derived climb-limit estimates.

Each revision compares damper off, K=-0.02 with the original 10 Hz filter, and
K=-0.02 with 0.6 Hz washout plus 3 Hz low-pass. The latter limits slow feedback
near the 0.34 Hz mode and higher-frequency feedback while retaining moderate
response around 1.5 Hz. Those frequencies are an informed candidate, not a
closed-loop margin claim. The native equation is delta_throttle=-K*q_filtered.
Negative generic TECS_PTCH_DAMP is a SITL-only diagnostic here; its normal
metadata does not advertise negative values.

Revision two uses PG_TURN_FF=0.009 and D_FF=0. The model's +/-10 deg/s trim
solutions correspond to FF approximately 0.00564 left and 0.01229 right when
including neutral bias. A common gain near their midpoint lets the existing
integrator correct the asymmetry. The previous 0.051 FF causes excessive turn
rate after the high-frequency P-driven limit cycle is removed. Disabling the
large acceleration feedforward avoids an uncalibrated brake kick at turn entry.
TECS_SINK_MAX and SINK_MIN become 2.5 m/s: the -0.4 m/s trim requires 37.52%
throttle, implying about 0.177 throttle fraction per m/s near cruise. This is
local feedforward scaling, not a validated full-glide performance claim.

## Interpretation

Heading-rate and pitch-rate measurements include payload motion. In a steady
banked turn, body pitch rate can have a substantial mean without oscillation.
Compare standard deviation and slow (0.1–0.8 Hz) / fast (0.8–4 Hz) spectral RMS
alongside total RMS. Spectral estimates use the final 20 s of each phase.
Height/climb errors use TECS estimates versus logged filtered demands; separate
truth climb and attitude plots expose model motion. Requested servo outputs
are not applied brake outputs in fixed-brake diagnostic runs.

The final revision also tests TECS_PG_HR_TC=0.5 s. Native FBWB captures the current
altitude when the pitch stick returns to neutral; the resulting position-demand
change produces a short derivative-feedforward throttle kick. The optional
first-order smoothing changes only derivative feedforward, not height feedback
or altitude-trajectory slew limits. It resets with TECS and defaults to zero for
backward compatibility. `r2_raw_feedforward` is the matched damper-off ablation
with HR_TC=0. This is part of revision two, not another tuning sweep.

TECS.dhdem records the filtered rate with HR_TC enabled. For consistent climb
error metrics, the analysis reconstructs the unfiltered trajectory derivative
from logged TECS.hdem, rather than comparing one controller to a smoother
reference. Height error remains the primary tracking comparison. Peak pitch
rate and throttle range over the whole phase supplement final-20-second metrics
to expose capture transients.

## Selected candidate and validation

Select `mission-controller-candidate.json` (or its equivalent `.parm` overlay):
damper off, HR_TC=0.5 s, revision-two feedforward and lateral settings. Both
small reversed damper variants slightly improve some peaks but generally
increase settled pitch-rate variation. They remain recorded experiments, not
the selected configuration. No third design revision was made.

```sh
python Tools/simulation/paraglider/run_longitudinal_study.py Tools/simulation/paraglider/study-validation.json
PG_GAINS=original PG_CONTROLLER_CONFIG=Tools/simulation/paraglider/mission-controller-candidate.json BUILDLOGS=/home/david/repos/pg-longitudinal-design/candidate_auto python Tools/simulation/paraglider/compare_autotest.py --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission
```

The sensitivity runs use the same candidate with `dynamic_roll` at 22.2 V and
separately the nominal profile at 20.4 V. No gains change for either case.
The AUTO check uses the existing route and acceptance checks unchanged.

Recreate final nominal plots with `compare_longitudinal_study.py` and the case
lists `baseline_matched r2_off`, `r2_raw_feedforward r2_off`, and
`r2_off r2_reversed r2_shaped`. The report builder accepts `--runs` and `--output`;
output is a local HTML report with figures and metric/configuration downloads.
The existing `plot_comparison.py` accepts `--candidate` for the new AUTO CSV.

Research logs and raw data stay in their existing Mission Planner/native run
locations. The tinkering repository's `tools/archive_flight_figures.py` archives
only report-linked final figures and plotting source, per the user's requested
scope. It does not copy telemetry exports or simulation datasets.

## Completed results

The selected candidate reduced matched climb height RMSE from 0.92 to 0.14 m.
The smoothing-only ablation reduced climb-capture peak pitch rate from 64.06 to
14.74 deg/s, with height RMSE increasing from 0.06 to 0.08 m. Sustained right/left
turn-rate RMSE was 0.31/0.14 deg/s. The softer-roll model remained well behaved.
At 20.4 V, climb height RMSE increased to 0.48 m without throttle saturation;
native voltage compensation was disabled in this original study. The corrected
voltage experiment below supersedes the proposed new-feedforward work.

The existing extended AUTO test PASSED, including opposing loiters, altitude,
corner, home-loiter and controller-handover checks. Its 1,200-second mission
record had maximum waypoint altitude error 1.67 m and corner overshoot 0.66 m.
First-loiter bank standard deviation was 0.039 deg versus 2.60 deg for the
high-DOF original gains. Median radii were 26.74/26.68 m for 25 m commands.
Steadier motion costs slightly larger radius/corner errors than that baseline.
The model and controller still require flight validation; these are SITL results.

## Native voltage-compensation correction (11 October 2026)

The existing `ArduPlane/servos.cpp` compensation was previously disabled by
`FWD_BAT_VOLT_MAX=0` and a disabled SITL battery monitor. The JSON adapter also
omitted its motor-model voltage. It now publishes that voltage, and `sitl.parm`
enables `BATT_MONITOR=3` (voltage only), voltage pin 13 and multiplier 10.1.
There is no current/discharge model. `mission-controller-candidate` and the SITL
overlay set `FWD_BAT_VOLT_MAX=22.2`, `FWD_BAT_VOLT_MIN=18`, `FWD_BAT_IDX=0`.
The reference matches the existing throttle calibration, not full-charge 6S.
The bounds clamp compensation; only 22.2 V and 20.4 V were evaluated here.

Reproduce the four matched FBWB cases:

```sh
python Tools/simulation/paraglider/run_longitudinal_study.py Tools/simulation/paraglider/study-voltage-compensation.json
python Tools/simulation/paraglider/analyze_longitudinal_study.py /home/david/repos/pg-longitudinal-design --cases voltage_22_2_off voltage_22_2_on voltage_20_4_off voltage_20_4_on --output /tmp/pg-voltage-report
```

The original study-initial/revision1/revision2/validation configurations now
explicitly disable compensation to preserve their historical comparisons.
The new voltage study uses the current candidate. The analyzer records battery
voltage and uses the compensated output limit when counting saturation.

Native output scaling is `c=22.2/clamp(V,18,22.2)`. At 20.4 V, `c=1.08824` and
the integer-percent physical throttle ceiling rises from 60% to 65%. Native
Plane returns that limit to TECS, so the climb feedforward slope becomes
`(0.65-0.45)/1=0.20` before output scaling, versus 0.15 at 22.2 V. The vertical
speed PI gain also grows with the limit. The corrected standalone longitudinal
analysis includes this behavior; it does not assume only a feedforward offset
or silently change the flight-controller implementation.

All four native FBWB runs completed. At nominal voltage, climb height RMSE was
0.1332 m off and 0.1334 m on. At 20.4 V it improved from 0.4855 to 0.2504 m;
level-phase height RMSE improved from 0.6333 to 0.0156 m. Climb-capture height
RMSE increased from 0.0958 to 0.1692 m. The compensation is useful, but the
changed native feedforward slope leaves a capture tradeoff. Damper and lateral
gains are unchanged. Logged voltage agrees with physics to less than 0.01 V,
and settled PWM agrees with compensated TECS output within 0.002 throttle
fraction (median absolute error, including PWM quantization and scheduler delay).

Run the unchanged extended AUTO test at each voltage, sequentially:

```sh
PG_GAINS=original PG_VOLTAGE=22.2 PG_CONTROLLER_CONFIG=Tools/simulation/paraglider/mission-controller-candidate.json BUILDLOGS=/home/david/repos/pg-longitudinal-design/voltage_auto_22_2 python Tools/simulation/paraglider/compare_autotest.py --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission
PG_GAINS=original PG_VOLTAGE=20.4 PG_CONTROLLER_CONFIG=Tools/simulation/paraglider/mission-controller-candidate.json BUILDLOGS=/home/david/repos/pg-longitudinal-design/voltage_auto_20_4 python Tools/simulation/paraglider/compare_autotest.py --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission
```

Final plots and plotting sources are archived in ArduPilot-tinkering. Local
reports are in Mission Planner's `turn-model-spike/voltage-compensation` and
`classical-longitudinal-compensated`; raw native data stay in the run directories.

Both extended AUTO runs passed the unchanged altitude, corner, opposing-loiter
and handover checks. Maximum waypoint altitude error was
1.669/1.679 m at 22.2/20.4 V; maximum corner overshoot was
0.947/2.893 m. Thus completion does not imply identical corner performance.
The saved BIN logs confirm voltage and compensation settings for each mission.

The AUTO wrapper also now captures its flight BIN before the original test
resets SITL, preventing a newer startup BIN from being mistaken for the mission
log. The 20.4 V flight BIN was recovered from the completed run; the 22.2 V
mission was repeated with the corrected capture wrapper. A focused success /
failure-path check also verified preservation across multiple resets.


## Gust-response follow-up

The later [gust-control study](GUST-CONTROL.md) adds an optional second pitch-rate
filter and updates the mission candidate to gain 0.1 with two 1.5 Hz stages.
The voltage-compensation results above describe the earlier damper-off version;
the gust study records the validation of the new candidate separately.
