# Routed brake research profiles

`make_simulator(..., profile="routed_research")` uses 132 mm riser spacing and a fixed neutral reference from the archived level trim. `wide_research` uses 185 mm and re-ties neutral at that same reference attitude. Neither changes the default mission/dynamic_roll profiles or flight firmware.

The path follows servo → fixed guide → ring on a straight riser → canopy cascade. The constant servo/guide segment is absorbed in neutral trim. The C++ RHS recomputes path length every RK4 stage; positive length change adds pull, before clipping slack at zero. Riser directions follow the instantaneous canopy support point. This is a kinematic closure, not fabric/ring/contact/friction dynamics.

Measured/CAD anchors: 132 mm slot-center spacing, 91 mm guide spacing, guides 22.5 mm aft of the bolt axis. Assumptions: ring 60 mm along riser; effective canopy support halfspacing 0.3235535694 m; support 0.85 m above joint; cascade 0.10 m aft. Actual loaded ring height and brake cascade geometry remain unmeasured. Supply geometry to `routing.configure`; the neutral attitude must be a fixed setup reference, never reset each frame/window.

Exploratory pitch/yaw damping is 2.5 times legacy. The fit hit the search boundary and cannot establish a physical damping value. Matching the corrected spacing and fitted support product leaves the current rig's yaw stiffness unchanged. Wider-rig yaw stiffness scales by 185/132. Canopy and induced-drag hypotheses are applied separately in the study; wide_research holds aerodynamics fixed.

Study: micro-agu/research/rigging-refinement-2026-10-15. Tests check disabled-mode exact parity, geometric symmetry and rotation invariance, slack engagement and integration convergence. The data are reused development flights, not independent validation. Firmware/estimator integration of this research profile has not been flight-tested.
