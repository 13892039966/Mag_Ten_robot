"""Segment-aware geometry; proximal guide and world frame are explicit."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .model import ModelConfig


@dataclass(frozen=True)
class BodyConfig(ModelConfig):
    dt: float = .005
    passive_length_m: float = 1.800
    active_length_m: float = .200
    cable_limit_m: float = .030
    passive_edge_m: float = .025
    active_edge_m: float = .005


def body_layout(c, entry=None, direction=None, insertion=None):
    values = np.asarray(list(vars(c).values()), float)
    if not np.isfinite(values).all() or np.any(values <= 0.):
        raise ValueError('Finite positive geometry/control values required')
    if not 0. < c.inner_radius_m < c.cable_radius_m < c.radius_m:
        raise ValueError('Invalid cross section')
    entry = np.zeros(3) if entry is None else np.asarray(entry, float)
    axis = np.array([1.,0.,0.]) if direction is None else np.asarray(direction,float)
    if entry.shape!=(3,) or axis.shape!=(3,) or not np.isfinite(np.r_[entry,axis]).all() or np.linalg.norm(axis)<1e-12:
        raise ValueError('Finite entry and nonzero direction required')
    axis = axis/np.linalg.norm(axis)
    helper=np.array([0.,0.,1.]) if abs(axis[2])<.9 else np.array([0.,1.,0.])
    y=np.cross(helper,axis); y/=np.linalg.norm(y)
    rotation=np.column_stack([axis,y,np.cross(axis,y)])
    npass=int(math.ceil(c.passive_length_m/c.passive_edge_m))
    nactive=int(math.ceil(c.active_length_m/c.active_edge_m))
    s=np.r_[np.linspace(0.,c.passive_length_m,npass+1),np.linspace(c.passive_length_m,c.length_m,nactive+1)[1:]]
    insertion=c.length_m if insertion is None else float(insertion)
    if not np.isfinite(insertion) or not 0.<=insertion<=c.length_m:
        raise ValueError('Insertion must lie in the robot length')
    positions=entry+(s-c.length_m+insertion)[:,None]*axis
    poses=np.column_stack([positions,np.repeat(Rotation.from_matrix(rotation).as_quat()[None,:],len(s),axis=0)])
    indices=np.arange(npass,len(s),dtype=int)
    cables=[]
    for j in range(4):
        a=j*np.pi/2.
        offset=rotation@np.array([0.,c.cable_radius_m*np.cos(a),c.cable_radius_m*np.sin(a)])
        cables.append(positions[indices]+offset)
    return dict(abscissa=s,poses=poses,active_index=npass,cable_indices=indices,cable_points=cables,
                entry=entry,direction=axis,rotation=rotation,initial_insertion=insertion)


def surface_vertices(poses, radius, inner_radius, sectors=16):
    p=np.asarray(poses,float)
    angles=np.arange(sectors)*2.*np.pi/sectors
    offsets=np.array([[0.,r*np.cos(a),r*np.sin(a)] for r in [radius,inner_radius] for a in angles])
    r=Rotation.from_quat(p[:,3:]).as_matrix()
    return (np.einsum('nij,kj->nki',r,offsets)+p[:,:3,None].transpose(0,2,1)).reshape(-1,3)


def surface_faces(count,sectors=16):
    width=2*sectors;faces=[]
    for j in range(count-1):
        for ring in [0,sectors]:
            for k in range(sectors):
                a=j*width+ring+k;b=j*width+ring+(k+1)%sectors
                tri=[[a,b,b+width],[a,b+width,a+width]]
                faces.extend([x[::-1] for x in tri] if ring else tri)
    for j in [0,count-1]:
        for k in range(sectors):
            a=j*width+k;b=j*width+(k+1)%sectors
            tri=[[a,a+sectors,b+sectors],[a,b+sectors,b]]
            faces.extend([x[::-1] for x in tri] if j else tri)
    return faces
