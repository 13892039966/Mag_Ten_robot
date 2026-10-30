"""Same original-mesh safety decisions with certified batched centreline search.

No new collision response or relaxed safety threshold. Only the nearest-polyline
query is accelerated; exact triangle checks and port exemptions are retained.
"""
import numpy as np
from scipy.spatial import cKDTree
from .geometry_adapter import PhantomSafety,entrance_exemption
from .polyline_query import indexed_polyline
from .force_adapter import contact_force

class FastPhantomSafety(PhantomSafety):
    def __init__(self,geometry,radius,dt):
        super().__init__(geometry,radius,dt)
        self.tree=cKDTree(self.queries.starts+.5*self.queries.delta)
        self.maxhalf=float(np.sqrt(self.queries.lengths2).max()/2)
        self.fallback_points=0
    def check(self,samples,jacobian_text,multipliers):
        points=np.asarray(samples,float)
        if points.ndim!=2 or points.shape[1]!=3:raise ValueError('Expected collision centres Nx3')
        _,total,rows=contact_force(jacobian_text,multipliers,self.dt)
        result=dict(invalid_reason='',penetration_m=0.,contact_force_n=float(total),contact_rows=int(rows))
        if not np.isfinite(points).all():result['invalid_reason']='nonfinite_collision_samples';return result
        entry=entrance_exemption(points,self.entry,self.direction,self.radial_limit_m)
        exit_=entrance_exemption(points,self.exit,-self.exit_direction,self.radial_limit_m)
        active=points[~(entry|exit_)]
        if not len(active):return result
        distances,count=indexed_polyline(active,self.queries,self.tree,self.maxhalf)
        self.fallback_points+=count
        # Preserve scalar-reference decisions at floating point boundaries.
        boundary=(np.abs(distances-self.inner_radius)<1e-11)|(np.abs(distances-(self.inner_radius-self.radius+.002))<1e-11)
        for i in np.flatnonzero(boundary):distances[i]=self.queries.nearest_centreline(active[i])['lateral_error_m']
        inside=distances<=self.inner_radius
        suspect=(self.inner_radius-distances<self.radius-.002)|~inside
        if suspect.any():
            exact=self.queries.wall_distance_m(active[suspect])
            result['penetration_m']=float(max(0.,self.radius-exact.min()))
        if np.any(~inside):result['invalid_reason']='outside_lumen'
        elif result['penetration_m']>.003:result['invalid_reason']='wall_penetration'
        return result
