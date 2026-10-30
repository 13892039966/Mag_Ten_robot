"""Independent audit of every saved actuator command and unshifted exit."""
import sys,json,hashlib
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from tendon_sim.physical_recording import RecordingReader
from tendon_sim.geometry_adapter import load_phantom
from tendon_sim.body_geometry import BodyConfig
from dataclasses import asdict

def main():
 run=Path(sys.argv[1]);reader=RecordingReader(run/'physical_recording');reader.verify()
 contract=json.loads((run/'run_contract.json').read_text());summary=json.loads((run/'summary.json').read_text());baseline=json.loads((R/'outputs/dt20_terminal/summary.json').read_text())
 assert not reader.metadata.get('checkpoint_probe',False),'Probe cannot qualify as full route'
 assert contract['robot']==asdict(BodyConfig(dt=.02,feed_limit_m=1.4))
 assert contract['cpu_pool_actual']==4 and contract['control_period_s']==.02 and contract['control_stride']==1
 assert reader.manifest['initial_state_sha256']==baseline['initial_state_sha256']
 assert reader.frame_count==summary['physics_steps']+1
 _,route=load_phantom();previous=None;max_cable_step=max_feed_step=0.;closest=float('inf');max_abs_cable=0.;max_feed=0.
 for i in range(len(reader.manifest['chunks'])):
  chunk=reader._load(i);cmd=chunk['commands'];states=cmd if previous is None else np.vstack([previous,cmd]);step=np.abs(np.diff(states,axis=0))
  if len(step):max_cable_step=max(max_cable_step,float(step[:,:4].max()));max_feed_step=max(max_feed_step,float(step[:,4].max()))
  max_abs_cable=max(max_abs_cable,float(np.abs(cmd[:,:4]).max()));max_feed=max(max_feed,float(cmd[:,4].max()))
  assert np.all(cmd[:,4]>=-1e-12)
  closest=min(closest,float(np.linalg.norm(chunk['poses'][:,-1,:3]-route[-1],axis=1).min()))
  previous=cmd[-1]
 assert max_cable_step<=.001*reader.dt+1e-12 and max_feed_step<=.005*reader.dt+1e-12
 assert max_abs_cable<=.03+1e-12 and max_feed<=1.4+1e-12
 assert summary['peak_force_n']<=2.001 and summary['maximum_relative_edge_stretch']<=.1
 last=reader.frame(reader.frame_count-1);exit_distance=float(np.linalg.norm(last['poses'][-1,:3]-route[-1]));assert abs(last['t']-summary['simulated_s'])<1e-9
 success=summary['reason']=='station_reached' and summary['station_reached'] and exit_distance<=.005 and summary['progress_m']>=contract['route_length_m']-.005
 trace=[json.loads(x) for x in (run/'trace.jsonl').read_text().splitlines()];assert all(not x.get('invalid_reason') for x in trace)
 original=json.loads((R/'BUNDLE_SHA256.json').read_text());fixed=[n for n in original if n.startswith(('assets/','native/','setup/')) or n in ['tendon_sim/physics.py','tendon_sim/body_geometry.py','tendon_sim/model.py','tendon_sim/fast_safety.py','tendon_sim/geometry_adapter.py','tendon_sim/libigl_wall.py']]
 assert all(hashlib.sha256((R/n).read_bytes()).hexdigest()==original[n] for n in fixed)
 report=dict(verified=True,full_route_success=success,reason=summary['reason'],frames=reader.frame_count,dt_s=reader.dt,final_exit_distance_mm=exit_distance*1000,minimum_exit_distance_mm=closest*1000,max_cable_step_mm=max_cable_step*1000,max_feed_step_mm=max_feed_step*1000,max_abs_cable_mm=max_abs_cable*1000,max_feed_mm=max_feed*1000,original_fixed_files_verified=len(fixed),same_original_initial_state=True,scope='Every saved command, original physics/safety/assets hashes, original 5mm exit criterion. Runtime safety remains discrete; no hardware or continuous-collision claim.')
 (run/'limits_and_exit_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
