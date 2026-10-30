"""Allocate axial travel to feed, differential cable lengths to bending.

Uniform tendon shortening is not a calibrated axial actuator: unilateral
slack and compression otherwise accumulate pretension at far targets.
"""
import numpy as np
def differential_lengths(q):
    q=np.asarray(q,float)
    return q-np.mean(q)
def differential_jacobian(j):
    out=np.array(j,float,copy=True)
    out[:,:4]-=np.mean(out[:,:4],axis=1)[:,None]
    return out
