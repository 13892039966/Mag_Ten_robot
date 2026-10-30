"""Unchanged binary-STL reader extracted from the existing rendering workflow."""
import numpy as np
DT=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
def read_stl(path):
    with open(str(path),'rb') as f:
        f.read(80);count=np.fromfile(f,'<u4',1)[0]
        return np.fromfile(f,DT,int(count))['vertices'].astype(float)
