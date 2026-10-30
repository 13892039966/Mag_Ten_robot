"""Optional terminal heading feedback; never changes the true route or exit."""
import numpy as np

def heading_waypoint(goal,tip_axis,exit_axis,weight,lead_m,limit_m):
    goal=np.asarray(goal,float);tip=np.asarray(tip_axis,float);exit_=np.asarray(exit_axis,float)
    if any(x.shape!=(3,) or not np.isfinite(x).all() for x in [goal,tip,exit_]):raise ValueError('Finite 3-vectors required')
    if min(np.linalg.norm(tip),np.linalg.norm(exit_))<=1e-12:raise ValueError('Nonzero heading axes required')
    if not np.isfinite([weight,lead_m,limit_m]).all() or lead_m<0 or limit_m<=0:raise ValueError('Invalid heading compensation parameters')
    correction=float(np.clip(weight,0,1))*lead_m*(exit_/np.linalg.norm(exit_)-tip/np.linalg.norm(tip))
    norm=float(np.linalg.norm(correction))
    if norm>limit_m:correction*=limit_m/norm
    return goal+correction
