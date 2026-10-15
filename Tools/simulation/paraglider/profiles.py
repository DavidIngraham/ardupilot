"""Co-retained research configurations, selected for different aspects of flight evidence."""
from simulator import Simulator

def make_simulator(flight='trout_lake',profile='mission',**kwargs):
    if profile not in ('mission','dynamic_roll','routed_research','wide_research'):
        raise ValueError('unknown research profile')
    if profile=='dynamic_roll' and ({'roll_stiffness','right_preload_mm'} & kwargs.keys()):
        raise ValueError('dynamic_roll defines roll stiffness and right zero; set parameters explicitly after creation for a separate sensitivity case')
    if profile=='dynamic_roll' and kwargs.get('model','modal')!='modal':
        raise ValueError('dynamic_roll requires the modal mechanics base')
    sim=Simulator(flight,**kwargs)
    if profile=='dynamic_roll':
        sim.parameters['joint_k_roll']=1.0
        # Derived only by matching each nominal model's straight-level differential trim.
        sim.parameters['right_offset']={'hood_river':-.005482288179747503,
                                        'trout_lake':.0009301618779636765}[flight]
    if profile in ('routed_research','wide_research'):
        import numpy as np
        from pathlib import Path
        from routing import configure
        reference=np.load(Path(__file__).with_name(f'equilibrium-trim-{flight}-0.0-0.0.npz'))['state']
        # Exploratory fit: damping bound reached; not independently validated.
        sim.parameters['joint_c_pitch']*=2.5
        sim.parameters['joint_c_yaw']*=2.5
        # Correct the measured support location, then preserve the fitted yaw product.
        sim.parameters['yaw_payload_halfspacing']=.066
        sim.parameters['joint_k_yaw']*=.066/.0743*(.0743/.066)
        spacing=.185 if profile=='wide_research' else .132
        sim.parameters['joint_k_yaw']*=spacing/.132
        sim.parameters['yaw_payload_halfspacing']=spacing/2
        configure(sim,reference,spacing=spacing)
    return sim
