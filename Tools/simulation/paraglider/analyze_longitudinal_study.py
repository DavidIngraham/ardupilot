"""Extract phase-wise native tracking, pitch and actuator metrics."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import periodogram
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--cases',nargs='+');p.add_argument('--output',type=Path);a=p.parse_args();results={}
out=a.output or a.root;out.mkdir(parents=True,exist_ok=True)
for folder in sorted(a.root.iterdir()):
 if a.cases and folder.name not in a.cases:continue
 if not (folder/'study-events.json').exists():continue
 ev=json.loads((folder/'study-events.json').read_text());s=np.load(folder/'high-dof-trace.npz')['samples'];assert np.isfinite(s).all(), folder;assert not json.loads((folder/'high-dof-backend.json').read_text())['errors'], folder;t=s[:,0];y=s[:,2:];rot=Rotation.from_quat(y[:,3:7]);euler=rot.as_euler('xyz',degrees=True);rates=np.rad2deg(rot.inv().apply(y[:,10:13]));climb=-y[:,9]
 data={'TECS':[],'RCOU':[],'PGYR':[],'BAT':[]};params={}
 log=mavutil.mavlink_connection(str(folder/'flight.BIN'))
 while True:
  m=log.recv_match(type=['TECS','RCOU','PGYR','BAT','PARM'])
  if m is None:break
  k=m.get_type();ts=m.TimeUS*1e-6
  if k=='TECS':data[k].append([ts,m.h,m.hdem,m.dh,m.dhdem,m.th,m.hin])
  elif k=='RCOU':data[k].append([ts,(m.C1-1000)/1000,m.C3,m.C4])
  elif k=='PGYR':data[k].append([ts,m.Tgt,m.Act,m.RD,m.P,m.DFF,m.Out])
  elif k=='BAT':
   if m.Inst==0:data[k].append([ts,m.Volt])
  else:params[m.Name]=m.Value
 for k in data:data[k]=np.array(data[k])
 tecs=data['TECS'];servo=data['RCOU'];turn=data['PGYR'];battery=data['BAT'];r={};trajectory_rate=np.gradient(tecs[:,2],tecs[:,0])
 # These study runs use constant voltage, voltage-only monitoring and battery index 0.
 voltage=float(np.median(battery[:,1])) if len(battery) else None
 vmax=params.get('FWD_BAT_VOLT_MAX',0);vmin=params.get('FWD_BAT_VOLT_MIN',0)
 ratio=vmax/np.clip(voltage,vmin,vmax) if voltage is not None and 0<vmin<vmax and voltage>=.25*vmin else 1.0
 throttle_max=int(min(params.get('THR_MAX',60)*ratio,100))/100
 for event in ev['events']:
  start=event['start'];end=event['end'];mask=(t>=start)&(t<end);late=(t>=max(start,end-20))&(t<end)
  tm=(tecs[:,0]>=start)&(tecs[:,0]<end);sm=(servo[:,0]>=start)&(servo[:,0]<end);lm=(servo[:,0]>=end-20)&(servo[:,0]<end);ym=(turn[:,0]>=end-20)&(turn[:,0]<end)
  err=tecs[tm,1]-tecs[tm,2];cr=tecs[tm,3]-trajectory_rate[tm]
  freq,power=periodogram(rates[late,1],fs=1/np.median(np.diff(t)))
  band=lambda lo,hi:float(np.sqrt(np.sum(power[(freq>=lo)&(freq<hi)])*(freq[1]-freq[0])))
  r[event['name']]=dict(q_peak_abs=float(np.max(np.abs(rates[mask,1]))),throttle_peak_to_peak=float(np.ptp(servo[sm,1])),q_std=float(np.std(rates[late,1])),q_slow_band_rms=band(.1,.8),q_fast_band_rms=band(.8,4),q_rms=float(np.sqrt(np.mean(rates[late,1]**2))),roll_std=float(np.std(euler[late,0])),height_rmse=float(np.sqrt(np.mean(err**2))),height_max_abs=float(max(abs(err))),climb_rmse=float(np.sqrt(np.mean(cr**2))),climb_mean_late=float(np.mean(climb[late])),throttle_std=float(np.std(servo[sm,1])),throttle_mean=float(np.mean(servo[sm,1])),throttle_clip_fraction=float(np.mean((servo[sm,1]>=throttle_max-.001)|(servo[sm,1]<=.001))),turn_target_mean=float(np.mean(turn[ym,1])),turn_actual_mean=float(np.mean(turn[ym,2])),turn_error_rms=float(np.sqrt(np.mean((turn[ym,1]-turn[ym,2])**2))),brake_left_request_std_pwm=float(np.std(servo[lm,2])),brake_right_request_std_pwm=float(np.std(servo[lm,3])))
 results[folder.name]=r
 (folder/'voltage-check.json').write_text(json.dumps(dict(logged_voltage=voltage,compensation_ratio=float(ratio),physical_throttle_max=throttle_max),indent=2))
 np.savez_compressed(folder/'signals.npz',time=t,pitch=euler[:,1],roll=euler[:,0],q=rates[:,1],climb=climb,tecs=tecs,servo=servo,turn=turn,battery=battery)
 (folder/'logged-parameters.json').write_text(json.dumps(params,indent=2))
 fig,axes=plt.subplots(5,1,figsize=(13,12),sharex=True);origin=ev['events'][0]['start']
 axes[0].plot(tecs[:,0]-origin,tecs[:,1]-tecs[0,1],label='Estimated height');axes[0].plot(tecs[:,0]-origin,tecs[:,2]-tecs[0,1],label='Demand')
 axes[1].plot(t-origin,climb,label='Truth climb');axes[1].plot(tecs[:,0]-origin,trajectory_rate,label='Unfiltered trajectory rate')
 axes[2].plot(t-origin,rates[:,1]);axes[3].plot(servo[:,0]-origin,100*servo[:,1]);axes[4].plot(turn[:,0]-origin,turn[:,1],label='Target');axes[4].plot(turn[:,0]-origin,turn[:,2],label='Actual')
 for ax,label in zip(axes,['Height offset (m)','Climb (m/s)','Pitch rate (deg/s)','Throttle (%)','Heading rate (deg/s)']):
  ax.set_ylabel(label);ax.grid(alpha=.2);ax.set_xlim(0,ev['events'][-1]['end']-origin)
  for event in ev['events']:ax.axvline(event['start']-origin,color='gray',lw=.5)
 for event in ev['events']:axes[0].text(event['start']-origin,1,event['name'],rotation=90,va='top',fontsize=8,transform=axes[0].get_xaxis_transform())
 for ax in [axes[0],axes[1],axes[4]]:ax.legend(loc='upper right',fontsize=8)
 axes[-1].set_xlabel('Seconds');fig.suptitle(folder.name);fig.tight_layout();fig.savefig(folder/'response.png',dpi=130);plt.close(fig)
(out/'metrics.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
