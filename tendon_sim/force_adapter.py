"""Read equivalent collision impulses across old text and new sparse bindings."""
import numpy as np
from scipy.sparse import issparse
from .contact_forces import recover_contact_force

def contact_force(jacobian,values,dt):
    if not issparse(jacobian):return recover_contact_force(jacobian,values,dt)
    if not np.isfinite(dt) or dt<=0:raise ValueError('Positive dt required')
    j=jacobian.tocsr();values=np.asarray(values,float).reshape(-1)
    if j.shape[1]%3 or j.shape[0]>len(values):raise ValueError('Invalid collision Jacobian dimensions')
    force=np.asarray(j.T@values[:j.shape[0]]).reshape(-1,3)/dt
    if not np.isfinite(force).all():raise ValueError('Nonfinite contact force')
    return force.sum(axis=0),float(np.linalg.norm(force,axis=1).sum()),int(np.count_nonzero(np.diff(j.indptr)))
