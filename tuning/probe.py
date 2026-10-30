"""Exploratory checkpoint reconstruction ONLY; never full-route validation.

Solver warm-start state is not recorded and cannot be restored. A candidate
selected here must be rerun from the original entry state for acceptance.
"""
import sys,os,json,argparse,time
from pathlib import Path
for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
import numpy as np
from tendon_sim.physics import Sofa,SofaRuntime,build_physics,submit_targets
from tendon_sim.body_geometry import BodyConfig,body_layout
from tendon_sim.geometry_adapter import load_phantom
from tendon_sim.references import RouteReference
from tendon_sim.live_scene import Trial
from tendon_sim.fast_safety import FastPhantomSafety
from tendon_sim.scheduler import seed_scheduler,actual_thread_count
from tendon_sim.libigl_wall import replace_wall_pre_init
from tendon_sim.physical_recording import RecordingReader

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--time',type=float,default=270.);p.add_argument('--seconds',type=float,default=40.);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.output.exists():raise FileExistsError(a.output)
 cfg=json.loads(a.config.read_text());reader=RecordingReader(a.source/'physical_recording');frame=reader.frame(round(a.time/reader.dt));following=reader.frame(round(a.time/reader.dt)+1)
 c=BodyConfig(**reader.metadata['robot']);assert c.dt==.02
 g,route=load_phantom();d=body_layout(c,entry=g['entry_position_m'],direction=g['entry_direction'],insertion=0.)
 g=dict(g,mesh_path=str(R/'assets/collision/wall_collision_ax2_ang64_m.stl'))
 SofaRuntime.importPlugin('MultiThreading');seed_scheduler(4);root=Sofa.Core.Node('CheckpointProbeNotAcceptance');h=None;trial=None
 try:
  h=build_physics(root,c,g,d,native_rate_factor=0.)
  root.getObject('constraints').multithreading.value=True;h['beam'].getObject('linear').parallelInverseProduct.value=True;root.getObject('FreeMotionAnimationLoop').parallelCollisionDetectionAndFreeMotion.value=True
  root.removeObject(root.getObject('BVHNarrowPhase'));root.addObject('ParallelBVHNarrowPhase',name='parallel_narrow',nbThreads=4)
  replace_wall_pre_init(root);Sofa.Simulation.init(root);seed_scheduler(4);assert actual_thread_count()==4
  h['state'].position.value=frame['poses'].tolist();h['state'].velocity.value=frame['velocities'].tolist()
  submit_targets(h,following['commands'][:4],following['commands'][4]);Sofa.Simulation.animate(root,c.dt)
  warm_error=float(np.linalg.norm(np.asarray(h['state'].position.value)[:,:3]-following['poses'][:,:3],axis=1).max()*1000)
  reference=RouteReference(route,lookahead=cfg.get('lookahead',.025),terminal_goal_offset_m=cfg.get('terminal_goal_offset_m'),terminal_zone_m=cfg.get('terminal_zone_m',.15));reference.seed_from_tip(np.asarray(h['state'].position.value)[-1,:3])
  trial=Trial(h,a.output,reference,seconds=a.seconds,wall_seconds=600.,safety_factory=FastPhantomSafety,controller_config=cfg,name='ProbeControl',listening=True)
  root.addObject(trial)
  delta=(following['commands']-frame['commands'])/(c.dt*np.array([c.cable_speed_m_s]*4+[c.feed_speed_m_s]));trial.previous=np.clip(delta,-1,1);trial.desired=following['commands'].copy()
  trial.recorder.manifest['metadata'].update(checkpoint_probe=True,not_full_route_acceptance=True,source=str(a.source.resolve()),source_time_s=following['t'],solver_warm_start_restored=False,one_step_reconstruction_max_position_error_mm=warm_error)
  trial.recorder._manifest();trial.initialized();minimum=float('inf');started=time.monotonic()
  while not trial.done:
   Sofa.Simulation.animate(root,c.dt);minimum=min(minimum,float(np.linalg.norm(np.asarray(h['state'].position.value)[-1,:3]-route[-1])*1000))
  RecordingReader(a.output/'physical_recording').verify()
  report=dict(scope='CHECKPOINT PROBE ONLY: not a full-route result; requires entry-state confirmation.',parameters=cfg,source=str(a.source.resolve()),source_time_s=following['t'],reconstruction_error_mm=warm_error,minimum_true_exit_mm=minimum,reason=trial.summary['reason'],simulated_probe_s=trial.summary['simulated_s'],wall_s=time.monotonic()-started)
  (a.output/'probe_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
 finally:
  if trial is not None and not trial.done:trial.finish('probe_exception')
  Sofa.Simulation.unload(root)
if __name__=='__main__':main()
