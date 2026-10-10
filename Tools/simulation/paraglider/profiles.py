"""Co-retained research configurations, selected for different aspects of flight evidence."""
from simulator import Simulator

def make_simulator(flight='trout_lake',profile='mission',**kwargs):
    if profile not in ('mission','dynamic_roll'):
        raise ValueError('profile must be mission or dynamic_roll')
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
    return sim
