# AP_FLAKE8_CLEAN
"""Optional measured-anchor / uncertain-riser research geometry. No firmware changes."""
import numpy as np
import dynamics


def configure(sim, reference_state, spacing=.132, ring_height=.06, support_halfspacing=None,
              support_height=.85, cascade_x=-.1):
    """Re-tie neutral at one fixed reference attitude, preserving legacy brake zeros.

    Positive route length change consumes free line and increases effective pull.
    Reference is fixed per rigging, never re-zeroed during integration or per log window.
    """
    values = np.array([spacing, ring_height, support_height, cascade_x], float)
    if not np.isfinite(values).all() or spacing <= 0 or not 0 <= ring_height < support_height:
        raise ValueError('invalid routing geometry')
    b = support_halfspacing if support_halfspacing is not None else sim.parameters['yaw_canopy_halfspacing']
    if not np.isfinite(b) or b <= 0:
        raise ValueError('positive support halfspacing required')
    sim.parameters.update(routing_enabled=1., riser_halfspacing=spacing/2, ring_height=ring_height,
                          support_halfspacing=b, support_height=support_height, cascade_x=cascade_x)
    reference = dynamics.brake_paths(reference_state, sim.parameters)
    sim.parameters.update(route_reference_left=float(reference[0]), route_reference_right=float(reference[1]))


def diagnostic(sim, state, command):
    paths = dynamics.brake_paths(state, sim.parameters)
    reference = [sim.parameters['route_reference_left'], sim.parameters['route_reference_right']]
    delta = paths - reference
    raw = sim.controls(*command)[1:]
    return dict(path_m=paths, delta_m=delta, effective_m=np.maximum(0., raw+delta))
