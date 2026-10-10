"""Explicit geometry sensitivity; coordinates are engineering assumptions, not measurements."""
import numpy as np

def configuration(day,case):
 m0=1.385-.19; mb=0. if day=='2026-01-03' else .373
 r=np.array([0.,0.,case['ballast_below_original_payload_CG_m']]);cg=mb*r/(m0+mb)
 def parallel(m,r):return m*((r@r)*np.eye(3)-np.outer(r,r))
 I=np.diag([.006,.008,.003])+parallel(m0,-cg)+parallel(mb,r-cg)
 hinge=np.array([0.,0.,-.13])-cg;thrust=np.array([0.,0.,-.058])-cg
 imu=np.array([0.,0.,-case['imu_above_original_payload_CG_m']])-cg
 p=dict(zip(['hinge_x','hinge_y','hinge_z'],hinge))
 p.update(dict(zip(['Ixx','Iyy','Izz','Ixy','Ixz','Iyz'],[I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]])))
 p.update(dict(zip(['thrust_x','thrust_y','thrust_arm_z'],thrust)))
 return dict(mass=m0+mb+.19,payload_mass=m0+mb,cg_shift=cg,inertia=I,hinge=hinge,thrust=thrust,imu=imu,parameters=p)

def imu_specific_force(state,derivative,geometry):
 from scipy.spatial.transform import Rotation
 R=Rotation.from_quat(state[3:7]).as_matrix();w=state[10:13];a=derivative[10:13];r=R@geometry['imu']
 return R.T@(derivative[7:10]-[0,0,9.80665]+np.cross(a,r)+np.cross(w,np.cross(w,r)))
