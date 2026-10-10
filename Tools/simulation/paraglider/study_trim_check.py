from profiles import make_simulator
from trim import find_trim
import json
s=make_simulator('trout_lake','mission')
for c in [-.4,0,.4,.8,1.]:
 t=find_trim(s,climb_m_s=c);print(json.dumps({'climb':c,'command':t['command'].tolist()}),flush=True)
