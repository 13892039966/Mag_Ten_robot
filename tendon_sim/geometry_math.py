import numpy as np
def point_triangle_distance(points,triangles):
    p=np.asarray(points)[:,None,:]; t=np.asarray(triangles)
    a,b,c=t[:,0],t[:,1],t[:,2]; ab=b-a; ac=c-a
    normal=np.cross(ab,ac); n2=(normal*normal).sum(axis=-1)
    signed=((p-a)*normal).sum(axis=-1); proj=p-signed[...,None]*normal/np.maximum(n2,1e-30)[...,None]
    v=proj-a; d00=(ab*ab).sum(-1); d01=(ab*ac).sum(-1); d11=(ac*ac).sum(-1)
    d20=(v*ab).sum(-1); d21=(v*ac).sum(-1); den=d00*d11-d01*d01
    u=(d11*d20-d01*d21)/np.maximum(den,1e-30); w=(d00*d21-d01*d20)/np.maximum(den,1e-30)
    dist=np.where((u>=0)&(w>=0)&(u+w<=1),np.abs(signed)/np.sqrt(np.maximum(n2,1e-30)),np.inf)
    for x,y in [(a,b),(b,c),(c,a)]:
        edge=y-x; v=p-x; f=np.clip((v*edge).sum(-1)/np.maximum((edge*edge).sum(-1),1e-30),0,1)
        dist=np.minimum(dist,np.linalg.norm(v-f[...,None]*edge,axis=-1))
    return dist

def entrance_exemption(points,entry,direction,radial_limit_m,seam_tolerance_m=.002):
    """Only the upstream, radially bounded feeder corridor is outside-exempt."""
    p=np.asarray(points,float)-np.asarray(entry,float)
    axis=np.asarray(direction,float);axis=axis/np.linalg.norm(axis)
    axial=p@axis
    radial=np.linalg.norm(p-axial[:,None]*axis,axis=1)
    return (axial<=seam_tolerance_m)&(radial<=radial_limit_m)
