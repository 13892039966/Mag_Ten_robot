"""Single static identity closed oriented mesh, call before Sofa.Simulation.init.
Uses FrictionContactConstraint unchanged. Single closest contact is NOT proof of
legacy multi-primitive or whole-endoscope contact equivalence.
"""
import os,json,hashlib,ctypes
from pathlib import Path
HERE=Path(__file__).resolve().parent
DLL=HERE.parent/'native/libigl_adapter/build/source/libSofaDistanceGrid.so'
ASSETS=HERE.parent/'assets/libigl'
OBJ=ASSETS/'wall_collision_ax2_ang64_m.obj'
_library=None
def replace_wall_pre_init(root,wall_name='OriginalColon',**unused):
 global _library
 import SofaRuntime
 if os.name!='nt':
  for line in Path('/proc/self/maps').read_text().splitlines():
   if 'libSofaDistanceGrid.so' in line:
    loaded=Path(line.split()[-1]).resolve()
    if loaded!=DLL.resolve():raise RuntimeError('Foreign SofaDistanceGrid already loaded; fresh process required: '+str(loaded))
 meta=json.loads((ASSETS/'field_validation.json').read_text());raw=ASSETS/'wall_libigl_4mm_f64.raw'
 for path,key in [(OBJ,'source_mesh_sha256'),(raw,'raw_sha256')]:
  if hashlib.sha256(path.read_bytes()).hexdigest()!=meta[key]:raise RuntimeError('SHA mismatch: '+str(path))
 # Refuse an already loaded same-name plugin from a different experiment.
 if os.name=='nt':
  kernel=ctypes.WinDLL('kernel32');get=kernel.GetModuleHandleW;get.argtypes=[ctypes.c_wchar_p];get.restype=ctypes.c_void_p
  handle=get('SofaDistanceGrid.dll')
  if handle:
   buf=ctypes.create_unicode_buffer(32768);fn=kernel.GetModuleFileNameW;fn.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint];fn(handle,buf,len(buf))
   if Path(buf.value).resolve()!=DLL.resolve():raise RuntimeError('Fresh process required: '+buf.value)
 os.environ.update(CAMPAIGN_HYBRID_OBJ=str(OBJ.resolve()),CAMPAIGN_HYBRID_OBJ_SHA256=meta['source_mesh_sha256'],CAMPAIGN_HYBRID_RAW=str(raw.resolve()),CAMPAIGN_HYBRID_RAW_SHA256=meta['raw_sha256'],CAMPAIGN_HYBRID_TRIANGLES='171776')
 SofaRuntime.PluginRepository.addFirstPath(str(DLL.parent));SofaRuntime.importPlugin(str(DLL))
 _library=ctypes.CDLL(str(DLL));_library.campaignHybridPrepare.restype=ctypes.c_int
 if _library.campaignHybridPrepare()!=1:raise RuntimeError('Native geometry prepare failed')
 old=root.getChild(wall_name)
 if old is None:raise ValueError('Expected wall child '+wall_name)
 root.removeChild(old);removed=0
 for obj in list(root.objects):
  if obj.getClassName() in ['LocalMinDistance','MinProximityIntersection','NewProximityIntersection']:root.removeObject(obj);removed+=1
 if not removed:raise ValueError('Expected proximity intersection')
 root.addObject('DiscreteIntersection',name='distance_grid_intersection')
 wall=root.addChild('OriginalColonDistanceGrid');wall.addObject('MechanicalObject',template='Rigid3d',position=[[0,0,0,0,0,0,1]])
 wall.addObject('FixedProjectiveConstraint',fixAll=True)
 model=wall.addObject('RigidDistanceGridCollisionModel',filename=str(raw),nx=meta['shape_xyz'][0],ny=meta['shape_xyz'][1],nz=meta['shape_xyz'][2],box=[meta['box_min'],meta['box_max']],usePoints=False,moving=False,simulated=False,proximity=.0002,flipNormals=False)
 return {'wall':wall,'model':model}
def get_hybrid_stats():
 _library.campaignHybridStats.restype=ctypes.c_char_p
 return json.loads(_library.campaignHybridStats().decode())
