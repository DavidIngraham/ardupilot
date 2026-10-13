# Lateral configuration after the landing review

The selected mission profile uses PG_TURN_FF=0.009, P=0.005, I=0.002,
RDAMP=0.003, D_FF=0 and FILT=1 Hz. The other command limits and gains are
unchanged. Load mission-controller-candidate.parm after the model defaults
and sitl.parm for the complete development controller.

The landing harness now defaults PG_GAINS to mission. compare_autotest.py
resolves that preset's PG_TURN parameters from the same JSON file, leaving the
landing study's longitudinal setup unchanged. An explicit PG_CONTROLLER_CONFIG
still takes precedence; PG_GAINS=original reproduces the old lateral preset.
This prevents the landing harness from silently reinstating the old aggressive
feedback. Generic firmware defaults and existing vehicle parameters are not
rewritten by this opt-in simulation profile.

## Evidence

The earlier lateral filter work did include five trims: level, shallow climb/
descent and both turn directions. It had not established behavior throughout
landing, and the landing harness was not using the analyzed mission preset.
New straight trims at 0, 0.5 and 1 m/s sink reproduce the local nonlinear
frequency responses. Neither that analysis nor its pure-delay sensitivity
represents saturation, unilateral brake switching or the complete flare.

Twelve matched landing conditions were replayed with only lateral parameters
changed. Cases 0–3 informed the diagnosis; the 1-Hz candidate was frozen before
examining cases 4–11. All twelve completed. Unfiltered truth heading-rate error
and brake-command variation improved in every case versus the original landing
preset. Final miss: median 1.279 m, p90 3.498 m, worst 7.010 m; 11/12 within 5 m.
The worst case is 7.009 m long and 0.038 m cross-track. This is a steering
improvement, not a promise of improved touchdown placement in every condition.
Median sink at the modeled CG ground-plane crossing is 1.407 m/s.

The paired landing study intentionally preserves its previous longitudinal
parameters to isolate lateral changes. Combined-controller AUTO missions use
the complete selected JSON profile, including the current longitudinal tuning.
The interactive comparison retains previously published trial 1 instead of
selecting a more favorable example after tuning.

Analysis scripts, parameter/log audits, figures and bounded replays are in the
ArduPilot-tinkering repository. Raw logs stay in Mission Planner under
analysis/lateral-tuning-2026-10-13. The blog reports final results and before/
after controller activity, with the unchanged native landing architecture.

AI-assisted implementation, analysis and documentation.

## Native regression validation

The complete selected controller passed fresh AUTO missions in calm air,
moderate repeatable gusts and steady 2 m/s eastward wind. All include altitude
changes and opposing loiters. Maximum corner overruns were 0.43, 1.37 and
0.64 m respectively. The default landing-preset path also passed a native
landing and logged the selected PG_TURN values without a configuration override.

Compared with the matched 4-Hz references, calm loiter rate tracking improved.
In gusts, left-turn rate error increased from 9.79 to 10.80 deg/s while radial
error improved and brake content above 1 Hz fell about 42–44%. This is a known
tradeoff, not a universal tracking improvement. No settled loiter saturated.
