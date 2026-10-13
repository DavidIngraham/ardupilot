# Lateral configuration after the landing review

The selected mission profile uses PG_TURN_FF=0.009, P=0.005, I=0.002,
RDAMP=0.003, D_FF=0 and FILT=1 Hz. The other command limits and gains are
unchanged. Load mission-controller-candidate.parm after the model defaults
and sitl.parm for the complete development controller.

The landing harness defaults to the complete mission-controller-candidate.json
when neither PG_CONTROLLER_CONFIG nor PG_GAINS is explicitly set. This includes
the current longitudinal tuning as well as the selected lateral filter. An
explicit configuration still takes precedence. Explicit PG_GAINS=mission retains
the lateral-only preset for archived studies, and PG_GAINS=original retains the
old lateral preset. Generic firmware defaults and existing vehicle parameters
are not rewritten by this opt-in simulation profile.

The current landing study is documented under `Complete-profile refinement`
below. The earlier evidence is retained as the record of the initial diagnosis.

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

## Complete-profile refinement

The landing-refinement-2026-10-13 study reruns all twelve established conditions
with both 4 Hz and 1 Hz filtering and the complete current longitudinal profile.
All 24 native flights completed. Audited parameters differ only in PG_TURN_FILT.
Truth heading-rate RMSE improves in nine conditions and brake-command variation
improves in all twelve; worst RMSE across the set drops from 29.60 to 10.61 deg/s.
Spot placement improves in six conditions, so reduced steering activity must not
be presented as uniform touchdown-accuracy improvement.

Nine local equilibria include straight descents, both descending turn directions,
and partial common brake. The linear model uses the actual 50 Hz controller
update and zero-order command hold, verified by SCHED_LOOP_RATE and PGYR timing.
The earlier exploratory 400 Hz feedback calculation used the physics frame rate;
its pole values are superseded by this analysis. The current 4 Hz baseline has a
growing roughly 2 Hz mode in a 0.5 m/s descending right turn with 80–120 ms added
command delay. The 1 Hz configuration remains locally stable across the sampled
trims and delays. Small nonlinear response checks agree within 1.5e-5 relative
error. The outer guidance, estimator and longitudinal loop are outside this local
model; native landings evaluate their combined trajectory and flare behavior.

The separate validation campaign uses twelve new conditions, seed 2026101307,
with the complete 1 Hz profile fixed before inspecting them. Final figures,
parameter audits, exact configs and plotting scripts are published in the
ArduPilot-tinkering landing-refinement research record; raw data stays in Mission
Planner. Reproduction scripts explicitly load the complete controller JSON.

All twelve fresh validation landings completed within 5 m. Median miss was
1.838 m, p90 2.549 m and worst 2.603 m; median downward speed was 1.374 m/s.
These are results for the new seed, not a matched accuracy comparison with the
previous Monte Carlo campaign.


## Whole-guidance-loop regression

The complete development profile now explicitly selects NAVL1_PERIOD=10 seconds
and NAVL1_DAMPING=1.0. Inner lateral and longitudinal gains remain unchanged.
The earlier fixed-target linear analysis excluded waypoint guidance. A calm
landing exposed a sustained outer-loop oscillation at the inherited 8-second
period; increasing the period removes that large cycle.

The 12-second intermediate candidate passed native mission and touchdown
assertions but produced a severe oscillating approach in one randomized landing.
Its repeat settled; the original failure is retained as evidence of sensitivity.
It was not promoted. The 10-second candidate settled the calm approach and three
runs of that difficult headwind/delayed-actuator condition.

Forty-two fresh native SITL runs cover the parameter investigation and regression.
The selected 10-second candidate passed the standard extended AUTO route in calm
air, 2 m/s crosswind and synthetic gusts, plus 20 m calm and 45 m crosswind loiters.
Each route includes climbs, descents, corners, opposing loiters and controller
handover. All twelve randomized landing regression cases completed within 5 m:
median miss 1.70 m, worst 2.51 m. Separate focused repeats reached 3.28 m miss.
These reused landing cases are regression evidence, not fresh held-out validation.

The selected setting preserves ordinary tracking, with tradeoffs: gusty left
loiter radial RMS is 2.48 m versus 2.27 m at 8 seconds. Landing condition 07 has
a larger transient roll excursion that settles before flare. Do not claim every
metric improved or that arbitrary initializations have been proven stable.

Reports, parameter audits, plots and reproduction scripts are archived in
ArduPilot-tinkering/research/flight-log-analysis/l1-regression-2026-10-13.
Raw recordings remain in Mission Planner. Generic firmware defaults and existing
aircraft parameters are unchanged; load the complete opt-in development profile.
