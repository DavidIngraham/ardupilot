# External MicroAGU simulation moved

The external 9-DOF physics, Python JSON adapter, control experiments and study notes
are maintained in [microAGU/simulation](https://github.com/DavidIngraham/micro-agu/tree/main/simulation).
That repository also builds the browser simulator from the same C++ source.
Its migration manifest records this directory's source revision and file hashes.

ArduPilot retains `libraries/SITL/SIM_Paraglider.*`, its limited pitch articulation,
flight controllers/mixer/TECS/flare, native tests and the generic JSON/WASM transport.
Use the external model when the full two-body model is needed; it is not a firmware
dependency. Older commits preserve the original study snapshots.
