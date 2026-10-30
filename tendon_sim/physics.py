"""Native continuous beam and active-only cables; optional original colon/guide."""
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
from .bootstrap import load_sofa,PLUGIN_NAMES
from .body_geometry import body_layout

from .mapping25 import beam_index_points
Sofa,SofaRuntime=load_sofa()


def build_physics(root,c,geometry=None,layout=None,native_rate_factor=0.):
    if not np.isfinite(native_rate_factor) or native_rate_factor<0:raise ValueError('Nonnegative native rate factor required')
    d=layout if layout is not None else body_layout(c,entry=geometry['entry_position_m'] if geometry else None,
                  direction=geometry['entry_direction'] if geometry else None,insertion=.020 if geometry else None)
    root.dt=c.dt;root.gravity=[0,0,0]
    root.addObject('RequiredPlugin',name='Dependencies',pluginName=PLUGIN_NAMES)
    root.addObject('FreeMotionAnimationLoop');root.addObject('DefaultVisualManagerLoop')
    root.addObject('GenericConstraintSolver',name='constraints',tolerance=1e-9,maxIterations=500,computeConstraintForces=True)
    if geometry:
        root.addObject('CollisionPipeline');root.addObject('BruteForceBroadPhase');root.addObject('BVHNarrowPhase')
        root.addObject('LocalMinDistance',alarmDistance=.002,contactDistance=.0002,angleCone=.1)
        root.addObject('RuleBasedContactManager',response='FrictionContactConstraint',responseParams='mu=0.1')
    base=root.addChild('InsertionBoundary')
    target=base.addObject('MechanicalObject',name='targets',template='Rigid3d',position=d.get('guide_poses',d['poses']).tolist())
    base.addObject('FixedConstraint',fixAll=True)
    beam=root.addChild('WholeEndoscope')
    beam.addObject('EulerImplicitSolver',rayleighMass=.1,rayleighStiffness=.05)
    beam.addObject('SparseLDLSolver',name='linear',template='CompressedRowSparseMatrixd')
    n=len(d['poses'])
    beam.addObject('EdgeSetTopologyContainer',name='edges',edges=[[i,i+1] for i in range(n-1)])
    state=beam.addObject('MechanicalObject',name='dofs',template='Rigid3d',position=d['poses'].tolist())
    rest_args={}
    if d.get('preinserted'):
        identity=[[0,0,0,0,0,0,1]]*(n-1)
        rest_args=dict(straight=True,edgeList=list(range(n-1)),lengthList=np.diff(d['abscissa']).tolist(),
                       DOF0TransformNode0=identity,DOF1TransformNode1=identity)
    beam.addObject('BeamInterpolation',name='interpolation',radius=c.radius_m,innerRadius=c.inner_radius_m,
                   defaultYoungModulus=[c.young_modulus_pa],defaultPoissonRatio=[.4],**rest_args)
    beam.addObject('AdaptiveBeamForceFieldAndMass',interpolation='@interpolation',massDensity=c.density_kg_m3)
    beam.addObject('UniformMass',name='tip_mass',template='Rigid3d',indices=[n-1],
                   vertexMass='%g 1 [2e-6 0 0,0 3.613333333e-6 0,0 0 3.613333333e-6]'%c.tip_mass_kg)
    pins=np.flatnonzero(d['abscissa']-c.length_m+d['initial_insertion']<-.005).tolist() if geometry else [0]
    pins=pins or [0]
    feeder=beam.addObject('RestShapeSpringsForceField',name='outer_guide',points=pins,external_points=pins,
                         external_rest_shape=target.getLinkPath(),stiffness=1e6,angularStiffness=1e3)
    beam.addObject('GenericConstraintCorrection',linearSolver='@linear')
    cables=[]
    for i,points in enumerate(d['cable_points']):
        node=beam.addChild('active_tendon_%d'%i)
        guides=node.addObject('MechanicalObject',name='guides',position=points.tolist())
        node.addObject('RigidMapping',input=state.getLinkPath(),output=guides.getLinkPath(),
                       rigidIndexPerPoint=d['cable_indices'].tolist(),globalToLocalCoords=True,mapMasses=False)
        rate_args=dict(maxDispVariation=c.cable_speed_m_s*c.dt*native_rate_factor) if native_rate_factor>0 else {}
        cables.append(node.addObject('CableConstraint',name='cable',indices=list(range(len(points))),hasPullPoint=False,
            value=[0.],cableInitialLength=c.active_length_m,maxPositiveDisp=c.cable_limit_m,maxNegativeDisp=c.cable_limit_m,
            minForce=0.,maxForce=c.cable_force_n*c.dt,**rate_args))
    collision=None
    if geometry:
        col=beam.addChild('Collision')
        s=np.linspace(0,c.length_m,int(np.ceil(c.length_m/.005))+1)
        collision=col.addObject('MechanicalObject',name='centres',position=np.column_stack([s,np.zeros_like(s),np.zeros_like(s)]).tolist())
        col.addObject('AdaptiveBeamMapping',input=state.getLinkPath(),output=collision.getLinkPath(),
                      interpolation='@../interpolation',useCurvAbs=False,points=beam_index_points(s,d['abscissa']),mapMasses=False)
        col.addObject('SphereCollisionModel',radius=float(np.sqrt(c.radius_m**2+(.005/2)**2)),group=1)
        wall=root.addChild('OriginalColon')
        wall.addObject('MeshSTLLoader',name='loader',filename=str(geometry['mesh_path']),flipNormals=bool(geometry.get('flipNormals',True)))
        wall.addObject('MeshTopology',src='@loader');wall.addObject('MechanicalObject',src='@loader')
        for component in ['TriangleCollisionModel','LineCollisionModel','PointCollisionModel']:
            wall.addObject(component,moving=False,simulated=False,bothSide=True)
    return dict(root=root,beam=beam,state=state,base=target,feeder=feeder,cables=cables,collision=collision,
                config=c,definition=d,geometry=geometry,native_rate_factor=float(native_rate_factor),shortening=np.zeros(4),feed_m=0.)


def submit_targets(h,q,feed):
    c=h['config'];q=np.clip(np.asarray(q,float),-c.cable_limit_m,c.cable_limit_m)
    feed=float(np.clip(feed,0. if h['geometry'] else -c.feed_limit_m,c.feed_limit_m))
    for cable,value in zip(h['cables'],q):cable.value.value=[float(value)]
    targets=h['definition'].get('guide_poses',h['definition']['poses']).copy();targets[:,:3]+=h['definition']['direction']*feed
    h['base'].position.value=targets.tolist()
    if h['geometry']:
        d=h['definition'];pins=np.flatnonzero(d['abscissa']-c.length_m+d['initial_insertion']+feed<-.005).tolist() or [0]
        h['feeder'].points.value=pins;h['feeder'].external_points.value=pins
    h['shortening']=q;h['feed_m']=feed
