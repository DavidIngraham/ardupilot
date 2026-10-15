from pathlib import Path
import ctypes,numpy as np
from defaults import DEFAULT
KEYS=['rho','S','b','c','CL0','CLa','CD0','k_drag','CYbeta','CLbrake','CDbrake','Clbeta','Clp','Cldelta','Cm0','Cma','Cmq','payload_CDA','thrust_max','thrust_z','prop_torque','joint_k_roll','joint_k_pitch','joint_k_yaw','joint_c_roll','joint_c_pitch','joint_c_yaw','Cnbeta','Cnr','Cndelta']
KEYS+=['added_mx','added_my','added_mz','added_Jx','added_Jy','added_Jz','added_z1','added_z2']
DEFAULT=DEFAULT.copy();DEFAULT.update(dict(zip(KEYS[-8:],[.01,.1,.5,.01,.005,.005,0.,.5])))
KEYS+=['canopy_half_arc']
DEFAULT['canopy_half_arc']=.6
KEYS += ['hinge_x','hinge_y','hinge_z','Ixx','Iyy','Izz','Ixy','Ixz','Iyz','thrust_x','thrust_y','thrust_arm_z']
DEFAULT.update(dict(zip(KEYS[39:],[0,0,-.13,.006,.008,.003,0,0,0,0,0,-.058])))
KEYS+=['canopy_mass'];DEFAULT['canopy_mass']=.19
KEYS+=['brake_tilt_rad_per_m'];DEFAULT['brake_tilt_rad_per_m']=0.
KEYS+=['rotor_spin_inertia','rotor_spin_sign'];DEFAULT.update(rotor_spin_inertia=0.,rotor_spin_sign=0.)
KEYS+=['geometry_yaw','yaw_payload_halfspacing','yaw_canopy_halfspacing','yaw_vertical_separation'];DEFAULT.update(geometry_yaw=0.,yaw_payload_halfspacing=.0743,yaw_canopy_halfspacing=.5,yaw_vertical_separation=.85)
KEYS+=['spin_acceleration_scale'];DEFAULT['spin_acceleration_scale']=0.
# Routing dimensions in metres; disabled preserves the legacy command path exactly.
KEYS += ['routing_enabled','riser_halfspacing','guide_halfspacing','guide_x','ring_height',
         'support_halfspacing','support_height','cascade_x','route_reference_left','route_reference_right']
DEFAULT.update(routing_enabled=0.,riser_halfspacing=.066,guide_halfspacing=.0455,guide_x=-.0225,
               ring_height=.060,support_halfspacing=.3235535694,support_height=.85,cascade_x=-.1,
               route_reference_left=0.,route_reference_right=0.)
lib=ctypes.CDLL(str(Path(__file__).with_name('libdynamics.so')));ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');lib.derivative.argtypes=[ptr,ptr,ptr,ctypes.c_double,ptr,ptr];lib.replay.argtypes=[ptr,ptr,ctypes.c_int,ctypes.c_double,ctypes.c_int,ptr,ctypes.c_double,ptr,ptr];lib.replay.restype=ctypes.c_int

def params(p=None):
 d=DEFAULT.copy();d.update(p or {});scale=d.get('added_scale',1.)
 if d['geometry_yaw']>0:
  a,b,h=d['yaw_payload_halfspacing'],d['yaw_canopy_halfspacing'],d['yaw_vertical_separation']
  if min(a,b,h)<=0 or h*h<=4*a*b:raise ValueError('Yaw geometry requires positive spacings and h^2 > 4ab')
 for k in KEYS[30:36]:d[k]*=scale
 return np.array([d[k] for k in KEYS])
def derivative(y,u,p,m,w):
 out=np.zeros(26);lib.derivative(np.ascontiguousarray(y),np.ascontiguousarray(u,dtype=float),params(p),m,np.ascontiguousarray(w,dtype=float),out);return out
def replay(y,U,p,m,w,dt=.04,substeps=4):
 U=np.ascontiguousarray(U,dtype=float);out=np.zeros((len(U),26));n=lib.replay(np.ascontiguousarray(y),U,len(U),dt,substeps,params(p),m,np.ascontiguousarray(w,dtype=float),out)
 if n!=len(U):raise FloatingPointError('Replay diverged')
 return out


lib.causal_replay.argtypes=[ptr,ptr,ctypes.c_int,ctypes.c_double,ctypes.c_int,ptr,ctypes.c_double,ptr,ctypes.c_double,ctypes.c_double,ptr,ptr]
lib.causal_replay.restype=ctypes.c_int
lib.mapped_thrust.argtypes=[ctypes.c_double,ctypes.c_double];lib.mapped_thrust.restype=ctypes.c_double
def causal_replay(y,U,p,m,w,voltage,rpm,dt=.04,substeps=4):
 U=np.ascontiguousarray(U,dtype=float);out=np.zeros((len(U),26));r=np.zeros(len(U));n=lib.causal_replay(np.ascontiguousarray(y),U,len(U),dt,substeps,params(p),m,np.ascontiguousarray(w,dtype=float),voltage,rpm,out,r)
 if n!=len(U):raise FloatingPointError('Causal replay diverged')
 return out,r

lib.rotor_loads.argtypes=[ptr,ptr,ctypes.c_double,ctypes.c_double,ctypes.c_double,ptr]
def rotor_loads(omega_world,rotation,inertia,signed_rpm,signed_rpm_rate=0.):
 out=np.zeros(3)
 lib.rotor_loads(np.ascontiguousarray(omega_world,dtype=float),np.ascontiguousarray(np.asarray(rotation).flatten(order='F'),dtype=float),inertia,signed_rpm,signed_rpm_rate,out)
 return out

lib.mapped_torque.argtypes=[ctypes.c_double,ctypes.c_double];lib.mapped_torque.restype=ctypes.c_double

lib.brake_paths.argtypes=[ptr,ptr,ptr]
def brake_paths(y,p):
 out=np.zeros(2)
 lib.brake_paths(np.ascontiguousarray(y,dtype=float),params(p),out)
 return out
