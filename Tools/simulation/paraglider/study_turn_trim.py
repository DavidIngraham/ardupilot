from profiles import make_simulator
from trim import find_trim
import json,numpy as np
s=make_simulator('trout_lake','mission')
for turn in [-10.,10.]:
 t=find_trim(s,turn_deg_s=turn);u=t['command'];speed=np.linalg.norm(t['state'][7:10]);signed=u[2]/72-u[1]/62.4
 print(json.dumps({'turn':turn,'command':u.tolist(),'speed':float(speed),'signed_fraction':float(signed),'ff_equivalent':float(signed/(turn*5/speed))}),flush=True)
