"""Pitch-damper native SITL comparison using plant truth and logged actuator PWM."""
from pathlib import Path
import argparse,json
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import periodogram
from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--runs-dir',type=Path,default=Path(__file__).resolve().parents[4]/'pg-damper-study')
p.add_argument('--output',type=Path)
a=p.parse_args();root=a.runs_dir;out=a.output or root;out.mkdir(parents=True,exist_ok=True)
fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True);results={}
for case,label in [('off','Damper off'),('original','Original sign: -0.30 q'),('reversed','Reversed sign: +0.30 q'),('original_low','Original sign: -0.02 q'),('reversed_low','Reversed sign: +0.02 q')]:
 folder=root/case
 if not (folder/'damper-event.json').exists():continue
 event=json.loads((folder/'damper-event.json').read_text());sample=np.load(folder/'high-dof-trace.npz')['samples'];t=sample[:,0]-event['time_s'];y=sample[:,2:]
 rot=Rotation.from_quat(y[:,3:7]);pitch=rot.as_euler('xyz',degrees=True)[:,1];q=np.rad2deg(rot.inv().apply(y[:,10:13])[:,1]);climb=-y[:,9]
 m=mavutil.mavlink_connection(str(folder/'flight.BIN'));servo=[];tecs=[]
 while True:
  v=m.recv_match(type=['RCOU','TECS'])
  if v is None:break
  if v.get_type()=='RCOU':servo.append([v.TimeUS*1e-6-event['time_s'],(v.C1-1000)/10])
  else:tecs.append([v.TimeUS*1e-6-event['time_s'],v.th,v.h,v.hdem,v.dh])
 servo=np.array(servo);tecs=np.array(tecs)
 shown=(t>=-5)&(t<=75); shown_servo=(servo[:,0]>=-5)&(servo[:,0]<=75)
 if 'low' not in case:
  for ax,value in zip([axes[0],axes[1],axes[3]],[pitch,q,climb]):ax.plot(t[shown],value[shown],label=label,lw=1)
  axes[2].plot(servo[shown_servo,0],servo[shown_servo,1],label=label,lw=1)
 early=(t>=2)&(t<12);late=(t>=45)&(t<75);post=(t>=0)&(t<75)
 fs=1/np.median(np.diff(t));f,power=periodogram(q[late],fs=fs);band=(f>.1)&(f<10)
 sl=servo[(servo[:,0]>=45)&(servo[:,0]<75)]
 results[case]=dict(gain=event['gain'],mass_kg=event['mass_kg'],early_q_rms_deg_s=float(np.sqrt(np.mean(q[early]**2))),late_q_rms_deg_s=float(np.sqrt(np.mean(q[late]**2))),late_pitch_peak_to_peak_deg=float(np.ptp(pitch[late])),late_pitch_mean_deg=float(np.mean(pitch[late])),late_throttle_peak_to_peak_pct=float(np.ptp(sl[:,1])),late_climb_std_m_s=float(np.std(climb[late])),late_dominant_q_hz=float(f[band][np.argmax(power[band])]),max_abs_pitch_deg=float(np.max(abs(pitch[post]))),late_throttle_at_limits_fraction=float(np.mean((sl[:,1]<=.1)|(sl[:,1]>=59.9))))
 np.savez_compressed(out/(case+'-signals.npz'),time=t,pitch_deg=pitch,pitch_rate_deg_s=q,climb_m_s=climb,servo=servo,tecs=tecs)
for ax,title in zip(axes,['Payload pitch (deg)','Body pitch rate (deg/s)','Native throttle output (%)','Climb rate (m/s)']):
 ax.set_ylabel(title);ax.set_xlim(-5,75);ax.axvline(0,color='k',ls='--',lw=.8);ax.grid(alpha=.25)
axes[0].legend();axes[0].set_title('Native longitudinal damper sign — Trout Lake 1.758 kg, fixed brake rigging')
axes[-1].set_xlabel('Seconds after identical 5 deg/s pitch-rate kick');fig.tight_layout();fig.savefig(out/'damper-sign.png',dpi=150)
(out/'summary.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))

fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True)
for case,label in [('off','Damper off'),('original_low','Original sign: -0.02 q'),('reversed_low','Reversed sign: +0.02 q')]:
 path=out/(case+'-signals.npz')
 if not path.exists():continue
 d=np.load(path); shown=(d['time']>=-5)&(d['time']<=75); shown_servo=(d['servo'][:,0]>=-5)&(d['servo'][:,0]<=75)
 for ax,key in zip([axes[0],axes[1],axes[3]],['pitch_deg','pitch_rate_deg_s','climb_m_s']):ax.plot(d['time'][shown],d[key][shown],label=label,lw=1)
 axes[2].plot(d['servo'][shown_servo,0],d['servo'][shown_servo,1],label=label,lw=1)
for ax,title in zip(axes,['Payload pitch (deg)','Body pitch rate (deg/s)','Native throttle output (%)','Climb rate (m/s)']):
 ax.set_ylabel(title);ax.set_xlim(-5,75);ax.axvline(0,color='k',ls='--',lw=.8);ax.grid(alpha=.25)
axes[0].legend();axes[0].set_title('Same sign comparison at the logged Trout Lake gain magnitude, 0.02')
axes[-1].set_xlabel('Seconds after 5 deg/s pitch-rate kick');fig.tight_layout();fig.savefig(out/'damper-sign-low.png',dpi=150)
