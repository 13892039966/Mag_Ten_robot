"""Read-only metre-scale phantom loader and sampled collision-centre safety.

The circular lumen sign uses the nearest *polyline segment*, never the old
colon voxel volume. Mesh distance is exact (KD-tree bound only prunes faces).
This monitor is not continuous collision detection or a mesh self-intersection
certificate. Port extensions assume an unobstructed straight external feeder.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from .geometry_math import point_triangle_distance
from .geometry_math import entrance_exemption
from .contact_forces import recover_contact_force

DEFAULT_ROOT = Path(__file__).resolve().parents[1]/'assets/original'


def load_phantom(root=DEFAULT_ROOT):
    """Validate original assets without rewriting/copying them; return (dict, route)."""
    root=Path(root).resolve()
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf8'))
    if (manifest.get('physical_unit') != 'm' or manifest.get('physical_loader_scale') != 1
            or manifest.get('flipNormals') is not False
            or manifest.get('inner_diameter_mm') != 40
            or manifest.get('outer_diameter_mm') != 46
            or manifest.get('wall_thickness_mm') != 3
            or manifest.get('actual_open_port_count') != 2):
        raise ValueError('Unexpected phantom units, dimensions, normals or open ports')
    verified={}
    for name in ('wall_world_m.stl','geometry_world_m.npz','centerline_world_m.csv'):
        digest=hashlib.sha256((root/name).read_bytes()).hexdigest()
        if digest != manifest['files'][name]: raise ValueError('SHA256 mismatch: '+name)
        verified[name]=digest
    with np.load(root/'geometry_world_m.npz', allow_pickle=False) as data:
        vertices=np.array(data['vertices'],float); faces=np.array(data['faces'],int)
        route=np.array(data['centerline'],float); tangents=np.array(data['tangents'],float)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or faces.ndim != 2 or faces.shape[1] != 3
            or len(vertices) != manifest['vertices'] or len(faces) != manifest['triangles']
            or faces.min()<0 or faces.max()>=len(vertices)):
        raise ValueError('Mesh dimensions/indices disagree with manifest')
    if (route.ndim!=2 or route.shape[1]!=3 or len(route)<2 or tangents.shape!=route.shape
            or not all(np.isfinite(a).all() for a in (vertices,route,tangents))):
        raise ValueError('Invalid coordinates or tangents')
    delta=np.diff(route,axis=0); lengths=np.linalg.norm(delta,axis=1)
    norms=np.linalg.norm(tangents,axis=1)
    if np.any(lengths<=1e-12) or np.any(norms<=1e-12): raise ValueError('Degenerate route or tangent')
    tangents=tangents/norms[:,None]
    if np.any(np.einsum('ij,ij->i',tangents[:-1],delta)<=0):
        raise ValueError('Tangent direction opposes route')
    arc=np.r_[0.,np.cumsum(lengths)]
    # CSV s is the generating curve's arc; polyline chords are slightly shorter.
    if not np.isclose(arc[-1],manifest['length_mm']/1000.,rtol=0,atol=.0001):
        raise ValueError('Route arc length disagrees with metre-scale manifest')
    csv=np.genfromtxt(root/'centerline_world_m.csv', delimiter=',',names=True)
    csv_route=np.column_stack([csv[k] for k in ('x_m','y_m','z_m')])
    csv_tangent=np.column_stack([csv[k] for k in ('tx','ty','tz')])
    if (csv_route.shape!=route.shape or not np.allclose(csv_route,route,atol=1e-9,rtol=0)
            or not np.allclose(csv['s_m'],arc,atol=.0001,rtol=0)
            or not np.isclose(csv['s_m'][-1],manifest['length_mm']/1000.,atol=1e-9,rtol=0)
            or not np.allclose(csv_tangent,tangents,atol=1e-7,rtol=0)):
        raise ValueError('CSV route/tangents disagree with NPZ/actual arc')
    geometry=dict(mesh_path=str(root/'wall_world_m.stl'), flipNormals=False,
                  source_manifest=str(root/'manifest.json'), manifest=manifest,
                  verified_sha256=verified, physical_unit='m', physical_loader_scale=1.,
                  vertices_m=vertices, faces=faces, centreline_m=route, tangents=tangents,
                  entry_position_m=route[0].copy(), entry_direction=tangents[0].copy(),
                  exit_position_m=route[-1].copy(), exit_direction=tangents[-1].copy(),
                  inner_radius_m=.020, arc_m=arc, arc_length_m=float(arc[-1]),
                  nominal_arc_length_m=float(manifest['length_mm']/1000.))
    return geometry,route


class PhantomQueries:
    def __init__(self, geometry):
        self.route=np.asarray(geometry['centreline_m'],float)
        self.starts=self.route[:-1]; self.delta=np.diff(self.route,axis=0)
        self.lengths2=np.einsum('ij,ij->i',self.delta,self.delta)
        if np.any(self.lengths2<=1e-24): raise ValueError('Degenerate route segment')
        self.arc=np.r_[0.,np.cumsum(np.sqrt(self.lengths2))]
        self.tri=np.asarray(geometry['vertices_m'],float)[np.asarray(geometry['faces'],int)]
        self.centres=self.tri.mean(axis=1); self.tree=cKDTree(self.centres)
        self.max_radius=float(np.linalg.norm(self.tri-self.centres[:,None,:],axis=-1).max())

    def nearest_centreline(self, point):
        point=np.asarray(point,float)
        fraction=np.clip(np.einsum('ij,ij->i',point-self.starts,self.delta)/self.lengths2,0.,1.)
        projections=self.starts+fraction[:,None]*self.delta
        distances=np.linalg.norm(projections-point,axis=1); i=int(distances.argmin())
        return dict(position_m=projections[i],index=i,fraction=float(fraction[i]),
                    lateral_error_m=float(distances[i]),
                    progress_m=float(self.arc[i]+fraction[i]*np.sqrt(self.lengths2[i])))

    def wall_distance_m(self, points):
        result=[]
        for point in np.asarray(points,float).reshape(-1,3):
            k=min(64,len(self.tri))
            while True:
                cd,indices=self.tree.query(point,k=k)
                d=float(point_triangle_distance(point[None],self.tri[np.atleast_1d(indices)]).min())
                # Every omitted centroid is at least max(cd) away; subtracting
                # global triangle radius bounds every omitted face distance.
                if k==len(self.tri) or np.max(cd)-self.max_radius>=d: break
                k=min(k*2,len(self.tri))
            result.append(d)
        return np.asarray(result)


class PhantomSafety:
    def __init__(self, geometry, radius, dt):
        if not np.isfinite([radius,dt]).all() or radius<=0 or dt<=0:
            raise ValueError('Positive finite body radius and physical dt required')
        self.radius=float(radius); self.dt=float(dt)
        self.inner_radius=float(geometry['inner_radius_m'])
        self.radial_limit_m=self.inner_radius-self.radius-.0005
        if self.radial_limit_m<=0: raise ValueError('Robot does not fit lumen')
        self.entry=np.asarray(geometry['entry_position_m'],float)
        self.direction=np.asarray(geometry['entry_direction'],float)
        self.exit=np.asarray(geometry['exit_position_m'],float)
        self.exit_direction=np.asarray(geometry['exit_direction'],float)
        for axis in (self.direction,self.exit_direction):
            if not np.isfinite(axis).all() or np.linalg.norm(axis)<=1e-12: raise ValueError('Invalid port direction')
        self.direction=self.direction/np.linalg.norm(self.direction)
        self.exit_direction=self.exit_direction/np.linalg.norm(self.exit_direction)
        self.queries=PhantomQueries(geometry)

    def check(self, samples, jacobian_text, multipliers):
        points=np.asarray(samples,float)
        if points.ndim!=2 or points.shape[1]!=3: raise ValueError('Expected collision centres Nx3')
        _,total,rows=recover_contact_force(jacobian_text,multipliers,self.dt)
        result=dict(invalid_reason='',penetration_m=0.,contact_force_n=float(total),contact_rows=int(rows))
        if not np.isfinite(points).all():
            result['invalid_reason']='nonfinite_collision_samples'; return result
        entry=entrance_exemption(points,self.entry,self.direction,self.radial_limit_m)
        exit_=entrance_exemption(points,self.exit,-self.exit_direction,self.radial_limit_m)
        active=points[~(entry|exit_)]
        if not len(active): return result
        distances=np.array([self.queries.nearest_centreline(p)['lateral_error_m'] for p in active])
        inside=distances<=self.inner_radius
        suspect=(self.inner_radius-distances<self.radius-.002)|~inside
        if suspect.any():
            exact=self.queries.wall_distance_m(active[suspect])
            result['penetration_m']=float(max(0.,self.radius-exact.min()))
        if np.any(~inside): result['invalid_reason']='outside_lumen'
        elif result['penetration_m']>.003: result['invalid_reason']='wall_penetration'
        return result
