import numpy as np
def batched_polyline(points,starts,delta,lengths2,chunk=32):
    distances=[]; projections=[]; indices=[]
    for a in range(0,len(points),chunk):
        p=points[a:a+chunk]
        displacement=p[:,None,:]-starts[None,:,:]
        t=np.clip(np.einsum('nsi,si->ns',displacement,delta)/lengths2,0,1)
        proj=starts[None,:,:]+t[:,:,None]*delta[None,:,:]
        d2=np.einsum('nsi,nsi->ns',p[:,None,:]-proj,p[:,None,:]-proj)
        idx=np.argmin(d2,axis=1); row=np.arange(len(p))
        distances.append(np.sqrt(d2[row,idx]));projections.append(proj[row,idx]);indices.append(idx)
    return np.concatenate(distances),np.concatenate(projections),np.concatenate(indices)

def indexed_polyline(points,q,tree,maxhalf,k=16):
    # Omitted segment distances are >= kth midpoint distance - longest half
    # segment length. Only accept candidates when this lower bound certifies it.
    cd,ix=tree.query(points,k=min(k,len(q.delta)))
    d=q.delta[ix];starts=q.starts[ix]
    t=np.clip(np.einsum('nki,nki->nk',points[:,None,:]-starts,d)/q.lengths2[ix],0,1)
    proj=starts+t[:,:,None]*d
    distance=np.linalg.norm(points[:,None,:]-proj,axis=2)
    j=distance.argmin(axis=1);row=np.arange(len(points));out=distance[row,j]
    unsafe=cd[:,-1]-maxhalf<out
    if unsafe.any():out[unsafe]=batched_polyline(points[unsafe],q.starts,q.delta,q.lengths2)[0]
    return out,int(unsafe.sum())

