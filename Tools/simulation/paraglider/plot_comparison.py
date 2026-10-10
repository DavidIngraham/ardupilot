"""Summarize the archived and repeated extended AUTO tests, including partial runs."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import argparse
root=Path(__file__).resolve().parents[3]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--runs-dir',type=Path,default=root.parent/'pg-sitl-comparison')
parser.add_argument('--output',type=Path)
parser.add_argument('--candidate',type=Path,help='Additional ParagliderAutoMission.csv to compare')
parser.add_argument('--archive',type=Path,default=root.parent/'ArduPilot-tinkering/scratch/paraglider-tests/yaw-controller/ParagliderAutoMission-final-native.csv')
args=parser.parse_args()
base=args.runs_dir
out=args.output or base
out.mkdir(parents=True,exist_ok=True)
arch=args.archive
cases=[('Archived original',arch),('Original model rerun',base/'baseline/ParagliderAutoMission.csv'),('High DOF / original gains',base/'high-original-v3/ParagliderAutoMission.csv'),('High DOF / conservative gains',base/'high-conservative/ParagliderAutoMission.csv')]
if args.candidate:cases.append(('Longitudinal mission candidate',args.candidate))
fig,axes=plt.subplots(2,2,figsize=(13,10));summary={}
for label,path in cases:
 if not path.exists():continue
 d=np.genfromtxt(path,delimiter=',',names=True)
 if d.size<2:continue
 j=json.loads(Path(str(path)+'.json').read_text());t=d['time_s']-d['time_s'][0]
 n=np.deg2rad(d['latitude_deg']-j['home_lat'])*6371000;e=np.deg2rad(d['longitude_deg']-j['home_lng'])*6371000*np.cos(np.deg2rad(j['home_lat']))
 axes[0,0].plot(e,n,label=label,alpha=.8);axes[0,1].plot(t,d['relative_alt_m'],label=label)
 axes[1,0].plot(t,np.rad2deg(d['roll_rad']),label=label);axes[1,1].plot(t,d['nav_cross_track_error_m'],label=label)
 metrics=j['metrics'];wp=[str(i) for i in (1,2,4,5,6,7,9,10,11) if metrics[str(i)]['altitude_at_closest_m'] is not None]
 err=[abs(metrics[i]['altitude_at_closest_m']-j['route'][int(i)-1][3]) for i in wp]
 overs=[x['next_track_overshoot_m'] for x in metrics.values() if x['next_track_overshoot_m'] is not None]
 summary[label]=dict(duration_s=float(t[-1]),last_seq=int(d['seq'][-1]),samples=len(d),max_waypoint_altitude_error_m=max(err,default=None),max_corner_overshoot_m=max(overs,default=None),max_abs_bank_deg=float(abs(np.rad2deg(d['roll_rad'])).max()),xtrack_rms_m=float(np.sqrt(np.mean(d['nav_cross_track_error_m']**2))),loiter_turns={i:metrics[i]['circle_angle_rad']/(2*np.pi) for i in ['3','8']}, loiter_radius_m={i:dict(median=float(np.median(d['target_distance_m'][(d['seq']==int(i)) & (d['target_distance_m']<50)])),p95=float(np.percentile(d['target_distance_m'][(d['seq']==int(i)) & (d['target_distance_m']<50)],95))) for i in ['3','8'] if np.any((d['seq']==int(i)) & (d['target_distance_m']<50))})
 summary[label]['median_groundspeed_m_s']=float(np.median(d['groundspeed_mps']))
 steady={}
 for seq in [2,3,8]:
  phase=d[d['seq']==seq]
  if len(phase)<10:continue
  phase=phase[phase['time_s']>phase['time_s'][-1]-30]
  steady[str(seq)]=dict(bank_mean_deg=float(np.rad2deg(phase['roll_rad']).mean()),bank_std_deg=float(np.rad2deg(phase['roll_rad']).std()),groundspeed_mean_m_s=float(phase['groundspeed_mps'].mean()),throttle_pwm_mean=float(phase['throttle_pwm'].mean()))
 summary[label]['last_30s_by_item']=steady
route=np.array(json.loads(Path(str(arch)+'.json').read_text())['route']);axes[0,0].plot(route[:,2],route[:,1],'k--',lw=.7,label='Mission route')
for ax,title,x,y in zip(axes.flat,['Extended AUTO ground track','Altitude','Payload bank','Navigation cross-track error'],['East (m)','Time since AUTO start (s)','Time since AUTO start (s)','Time since AUTO start (s)'],['North (m)','Relative altitude (m)','Bank (deg)','Cross-track (m)']):
 ax.set(title=title,xlabel=x,ylabel=y);ax.grid(alpha=.25)
axes[0,0].axis('equal');axes[0,0].legend(fontsize=8);fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150);plt.close(fig)
(out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
# Preserve even pre-mission failures as useful controller-response evidence.
fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
for folder in ['high-original-v3','high-conservative']:
 p=base/folder/'high-dof-trace.npz'
 if not p.exists():continue
 a=np.load(p)['samples'];a=a[a[:,1]>0]
 if not len(a):continue
 from scipy.spatial.transform import Rotation
 t=a[:,0]-a[0,0];y=a[:,2:];angles=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)
 axes[0].plot(t,-y[:,2],label=folder);axes[1].plot(t,angles[:,0],label=folder)
axes[0].set(ylabel='Height above ground datum (m)',title='High-DOF response after physics release');axes[1].set(ylabel='Payload bank (deg)',xlabel='Time after release (s)')
for ax in axes:ax.grid(alpha=.25);ax.legend()
fig.tight_layout();fig.savefig(out/'release-response.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
for label,path in cases:
 if not path.exists():continue
 d=np.genfromtxt(path,delimiter=',',names=True)
 if d.size<2:continue
 phase=d[d['seq']==3]
 if len(phase)<20:continue
 phase=phase[phase['time_s']>phase['time_s'][-1]-30]
 t=phase['time_s']-phase['time_s'][-1]
 axes[0].plot(t,np.rad2deg(phase['roll_rad']),label=label,alpha=.85)
 axes[1].plot(t,phase['target_distance_m'],label=label)
axes[0].set(ylabel='Payload bank (deg)',title='Last 30 seconds of the first loiter: motion versus tracking')
axes[1].set(ylabel='Distance from loiter center (m)',xlabel='Seconds before leaving loiter');axes[1].axhline(25,color='k',ls='--',lw=.8)
for ax in axes:ax.grid(alpha=.25)
axes[0].legend(fontsize=8);fig.tight_layout();fig.savefig(out/'loiter-detail.png',dpi=150);plt.close(fig)
