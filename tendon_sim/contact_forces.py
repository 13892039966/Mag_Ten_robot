"""Recover robot wall forces from collision-point Jacobian and solver impulses."""
import numpy as np

def recover_contact_force(jacobian_text, multipliers, dt):
    if not np.isfinite(dt) or dt<=0:raise ValueError('dt must be positive')
    values=np.asarray(multipliers,float).reshape(-1)
    point_forces={};rows=0
    for line in str(jacobian_text).splitlines():
        terms=line.split()
        if not terms:continue
        row=int(terms[0]);count=int(terms[1])
        if len(terms)!=2+4*count or row<0 or row>=len(values):
            raise ValueError('Malformed collision Jacobian row')
        rows+=1
        for j in range(count):
            offset=2+j*4;index=int(terms[offset])
            v=np.array([float(x) for x in terms[offset+1:offset+4]])*values[row]/dt
            point_forces[index]=point_forces.get(index,np.zeros(3))+v
    if not point_forces:return np.zeros(3),0.,0
    forces=np.array(list(point_forces.values()))
    if not np.isfinite(forces).all():raise ValueError('Nonfinite contact forces')
    return forces.sum(axis=0),float(np.linalg.norm(forces,axis=1).sum()),rows
