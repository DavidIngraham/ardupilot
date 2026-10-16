# External MicroAGU simulation moved

The external 9-DOF physics, Python JSON adapter, control experiments and study notes
are maintained in [microAGU/simulation](https://github.com/DavidIngraham/micro-agu/tree/main/simulation).
That repository also builds the browser simulator from the same Rust source.
Its migration manifest records this directory's source revision and file hashes.

ArduPilot retains `libraries/SITL/SIM_Paraglider.*`, its limited pitch articulation,
flight controllers/mixer/TECS/flare, native tests and upstream's JSON socket transport.
Use the external model when the full two-body model is needed; it is not a firmware
dependency. Older commits preserve the original study snapshots.

## Custom controller integration

The primary heading-rate controller lives in `libraries/APM_Control/PG_TurnController.*`.
It remains the default; no LQI controller is installed. Plane supplies the custom
frontend with `heading_rate_valid` and the same unshaped `heading_rate_target_dps`
used by the primary controller. Validity is cleared every stabilization loop and
set only when the paraglider control path actually runs. Backends must still gate
their supported modes, RC failsafe and estimator health in `can_run()`.

Custom differential demand uses `k_aileron` (-4500 to 4500). Symmetric brake uses
`set_brake_common_pct()` on every active update, not direct left/right PWM writes.
The mixer combines it with flare using the larger common demand, then allocates
the remaining differential travel. `brake_common_pct` reports the previous
allocated common demand for anti-windup/output matching. Manual takeover discards
custom common brake immediately; eligible automatic control releases an omitted
common request with a 0.25 s time constant. Backend `reset()` is responsible for
matching its initial outputs to current demands.

While custom aileron output overrides primary control, Plane resets the primary
heading controller with that output as its next starting demand. The first return
sample matches within the current brake limits; any residual beyond the normal
integrator bound decays with a 0.25 s time constant. Mode changes retain their
ordinary full reset. TECS retains the framework's stock reset behavior; this is
not a claim of bumpless throttle handover for a future integrated LQI backend.

These changes are AI-assisted. Controller gains and parameter indices are unchanged.
