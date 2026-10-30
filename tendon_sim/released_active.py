"""Available CC bending domain after the external feeder's last pinned node."""
import numpy as np
def released_active_segment(abscissa,active_index,pinned_indices):
    s=np.asarray(abscissa,float);p=np.asarray(pinned_indices,int).reshape(-1)
    index=max(int(active_index),int(p.max()) if len(p) else int(active_index))
    if index>=len(s)-1:raise ValueError('No free terminal bending segment')
    return index,float(s[-1]-s[index])
