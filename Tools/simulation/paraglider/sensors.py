"""Ideal payload measurements; no EKF/noise and no canopy-state feedback."""
import numpy as np
from scipy.spatial.transform import Rotation

def payload_observation(state,geometry,wind=(0.,0.,0.)):
 y=np.asarray(state);rot=Rotation.from_quat(y[3:7]);body=rot.inv().apply(y[10:13]);phi,theta,yaw=rot.as_euler('xyz');arm=rot.apply(geometry['imu']);velocity=y[7:10]+np.cross(y[10:13],arm)
 return dict(position_ned_m=y[:3]+arm,velocity_ned_m_s=velocity,body_rates_rad_s=body,euler_rad=np.array([phi,theta,yaw]),heading_rate_rad_s=float((body[1]*np.sin(phi)+body[2]*np.cos(phi))/np.cos(theta)),true_cg_airspeed_m_s=float(np.linalg.norm(y[7:10]-np.asarray(wind))))
