"""Diagnostic port of PG_TurnController core, GPL-3.0-or-later.
Source: ArduPilot commit b2f6da3a183a68d3de6f2c721ec90fca0eab53d5, ArduPlane/PG_TurnController.cpp.
No navigation/EKF/servo dynamics. Scheduled feedforward is a research alternative.
"""
import numpy as np
class Turn:
 def __init__(self,dt=.04,feedforward=None,p=.04,i=.002,rdamp=.03,dff=.02,filter_hz=4.):self.dt=dt;self.feedforward=feedforward;self.initial=False;self.integral=0.;self.p=p;self.i=i;self.rdamp=rdamp;self.dff=dff;self.filter_hz=filter_hz
 def update(self,request,actual,roll_rate,speed):
  dt=self.dt
  if not self.initial:self.actual=actual;self.roll=roll_rate;self.target=actual;self.initial=True
  alpha=dt/(dt+1/(2*np.pi*self.filter_hz));self.actual+=alpha*(actual-self.actual);self.roll+=alpha*(roll_rate-self.roll)
  request=np.clip(request,-20,20);self.target=np.clip(self.target,-20,20);prev=self.target;self.target+=np.clip(dt/(dt+.1)*(request-self.target),-40*dt,40*dt);self.target=np.clip(self.target,-20,20)
  scale=np.clip(5/max(speed,2),.25,2.5);error=self.target-self.actual
  ff=.051*self.target*scale if self.feedforward is None else self.feedforward(self.target)
  p=self.p*error*scale;dff=self.dff*(self.target-prev)/dt*scale**2;rd=-self.rdamp*self.roll*scale;inc=self.i*error*dt;candidate=np.clip(self.integral+inc,-.15,.15);proposed=ff+p+dff+rd+candidate
  if (proposed<=1 or inc<=0) and (proposed>=-1 or inc>=0):self.integral=candidate
  raw=ff+p+dff+rd+self.integral;self.terms=dict(ff=ff,p=p,dff=dff,rdamp=rd,integral=self.integral,raw=raw,target=self.target)
  return float(np.clip(raw,-1,1))
class Climb:
 def __init__(self,dt=.04):self.dt=dt;self.integral=0.;self.last=.43
 def update(self,request,actual):
  request=np.clip(request,-3,1.8);ff=.43+(request/1.8*(.6-.43) if request>0 else request/3*.43);error=request-actual;k=.6/(10*7);pd=np.clip(ff+k*error,0,.6);pd=np.clip(pd,self.last-self.dt*.6,self.last+self.dt*.6);self.last=pd
  hi=np.clip(.6-pd+.1,-.3,.3);lo=np.clip(-pd-.1,-.3,.3);self.integral=np.clip(self.integral+error*.3*self.dt*k,lo,hi);return float(np.clip(pd+self.integral,0,.6))
