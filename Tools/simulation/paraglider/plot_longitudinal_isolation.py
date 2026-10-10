"""Compare active-brake and held-brake native longitudinal experiments."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import periodogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import argparse
p=argparse.ArgumentParser();p.add_argument('--active',type=Path,required=True);p.add_argument('--fixed',type=Path,required=True);a=p.parse_args()
fig,axes=plt.subplots(3,2,figsize=(13,10),sharex=True);summary={}
for col,(root,setting) in enumerate([(a.active,'Active lateral controller'),(a.fixed,'Brakes held at equilibrium')]):
 summary[setting]={}
 for case,label in [('off','Off'),('original_low','Original K=+0.02'),('reversed_low','Reversed K=-0.02')]:
  event=json.loads((root/case/'damper-event.json').read_text())
  if col==1:assert event['fixed_brakes']
  s=np.load(root/case/'high-dof-trace.npz')['samples'];t=s[:,0]-event['time_s'];y=s[:,2:]
  r=Rotation.from_quat(y[:,3:7]);q=np.rad2deg(r.inv().apply(y[:,10:13])[:,1]);pitch=r.as_euler('xyz',degrees=True)[:,1];climb=-y[:,9]
  shown=(t>=-2)&(t<=75);late=(t>=45)&(t<75);early=(t>=0)&(t<10)
  for row,v in enumerate([q,pitch,climb]):axes[row,col].plot(t[shown],v[shown],label=label,lw=.8)
  summary[setting][case]=dict(late_q_rms_deg_s=float(np.sqrt(np.mean(q[late]**2))),late_pitch_p2p_deg=float(np.ptp(pitch[late])),late_climb_std_m_s=float(np.std(climb[late])),first10s_q_rms_deg_s=float(np.sqrt(np.mean(q[early]**2))))
 axes[0,col].set_title(setting);axes[0,col].legend(fontsize=9)
 for row,lab in enumerate(['Pitch rate (deg/s)','Pitch (deg)','Climb (m/s)']):
  axes[row,col].set_ylabel(lab);axes[row,col].grid(alpha=.25);axes[row,col].axvline(0,color='k',lw=.7,ls='--')
 axes[-1,col].set_xlabel('Seconds after 5 deg/s pitch-rate kick')
fig.suptitle('Native TECS, Trout Lake 1.758 kg: removing brake forcing');fig.tight_layout();fig.savefig(a.fixed/'isolation.png',dpi=150)
(a.fixed/'isolation-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
