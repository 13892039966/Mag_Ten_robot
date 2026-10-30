"""Explicit material coordinates for SOFA25 BeamAdapter mapping.

Observed 25.06 useCurvAbs=True collapses all samples onto tip at the frozen
high-contact state, despite correct points and lengthList. Explicit beam-index
coordinates avoid that runtime path and retain the original beam interpolation.
"""
import numpy as np

def beam_index_points(samples,knots):
    samples=np.asarray(samples,dtype=float);knots=np.asarray(knots,dtype=float)
    if samples.ndim!=1 or knots.ndim!=1 or len(knots)<2:raise ValueError('One-dimensional samples and beam knots required')
    if not np.isfinite(samples).all() or not np.isfinite(knots).all() or np.any(np.diff(knots)<=0):raise ValueError('Finite samples and increasing beam knots required')
    if np.any(samples<knots[0]) or np.any(samples>knots[-1]):raise ValueError('Samples must lie within beam knots')
    ids=np.clip(np.searchsorted(knots,samples,side='right')-1,0,len(knots)-2)
    bary=(samples-knots[ids])/(knots[ids+1]-knots[ids])
    encoded=np.minimum(ids+bary,np.nextafter(float(len(knots)-1),0.))
    return np.column_stack([encoded,np.zeros_like(samples),np.zeros_like(samples)]).tolist()

def migrate_collision_mapping(source):
    """Transform original builder source; caller injects beam_index_points global."""
    old='useCurvAbs=True,mapMasses=False'
    if source.count(old)!=1:raise ValueError('Expected exactly one original collision mapping anchor')
    return source.replace(old,"useCurvAbs=False,points=beam_index_points(s,d['abscissa']),mapMasses=False")
