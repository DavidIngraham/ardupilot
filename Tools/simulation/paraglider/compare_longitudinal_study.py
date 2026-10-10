"""Overlay phase-aligned mission responses for selected native study cases."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('cases',nargs='+');p.add_argument('--name',default='comparison');a=p.parse_args()
fig,ax=plt.subplots(4,2,figsize=(14,12),sharex='col')
for name in a.cases:
 folder=a.root/name;d=np.load(folder/'signals.npz');events=json.loads((folder/'study-events.json').read_text())['events']
 for col,phase in enumerate(['climb','right_turn']):
  e=next(e for e in events if e['name']==phase);start=e['start'];end=e['end']+35
  tecs=d['tecs'];servo=d['servo'];turn=d['turn'];m=(tecs[:,0]>=start-5)&(tecs[:,0]<end);n=(d['time']>=start-5)&(d['time']<end);v=(servo[:,0]>=start-5)&(servo[:,0]<end);z=(turn[:,0]>=start-5)&(turn[:,0]<end)
  ax[0,col].plot(tecs[m,0]-start,tecs[m,1]-tecs[m,2],label=name)
  ax[1,col].plot(d['time'][n]-start,d['q'][n],lw=.65)
  ax[2,col].plot(servo[v,0]-start,servo[v,1]*100,lw=.8)
  ax[3,col].plot(turn[z,0]-start,turn[z,2],lw=.8)
  if name==a.cases[-1]:ax[3,col].plot(turn[z,0]-start,turn[z,1],'k--',label='Turn demand')
  ax[0,col].set_title(phase+' and following capture/transition')
for col in range(2):
 for row,lab in enumerate(['Height error (m)','Pitch rate (deg/s)','Throttle (%)','Heading rate (deg/s)']):
  ax[row,col].set_ylabel(lab);ax[row,col].grid(alpha=.25);ax[row,col].axvline(0,color='k',lw=.6)
 ax[0,col].legend(fontsize=8);ax[-1,col].set_xlabel('Seconds from phase start')
fig.tight_layout();fig.savefig(a.root/(a.name+'.png'),dpi=140)
