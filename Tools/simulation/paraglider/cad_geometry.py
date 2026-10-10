"""Load approximate CAD-informed geometry for optional research replay; no production changes."""
import json
from pathlib import Path
import numpy as np

def configuration(day,case='nominal'):
 data=json.loads(Path(__file__).with_name('napkin-payload-model.json').read_text())
 r=next(x for x in data['cases'] if x['case']==case and x['with_lead']==(day=='2026-02-15'))
 I=np.array(r['inertia_body_kg_m2']);arms=r['arms_body_m'];p=dict(zip(['hinge_x','hinge_y','hinge_z'],arms['hinge']))
 p.update(dict(zip(['Ixx','Iyy','Izz','Ixy','Ixz','Iyz'],[I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]])))
 p.update(dict(zip(['thrust_x','thrust_y','thrust_arm_z'],arms['thrust'])))
 p['canopy_mass']=r['canopy_mass_kg']
 return dict(mass=r['AUW_kg'],payload_mass=r['payload_mass_kg'],canopy_mass=r['canopy_mass_kg'],inertia=I,hinge=np.array(arms['hinge']),thrust=np.array(arms['thrust']),imu=np.array(arms['imu']),parameters=p)
