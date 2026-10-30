"""Audit true exit distance from every saved physical frame, never waypoint error."""
import sys,json,csv
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from tendon_sim.physical_recording import RecordingReader
from tendon_sim.geometry_adapter import load_phantom
_,route=load_phantom();rows=[]
for run in [R/'outputs/dt20_terminal']+sorted((R/'outputs').glob('tune_*')):
 if not (run/'summary.json').exists():continue
 summary=json.loads((run/'summary.json').read_text());contract=json.loads((run/'run_contract.json').read_text())
 audit=run/'true_exit_audit.json'
 if not audit.exists():
  reader=RecordingReader(run/'physical_recording');reader.verify();best=(float('inf'),0.)
  for k in range(len(reader.manifest['chunks'])):
   x=reader._load(k);distance=np.linalg.norm(x['poses'][:,-1,:3]-route[-1],axis=1);j=int(distance.argmin())
   if distance[j]<best[0]:best=(float(distance[j]),float(x['t'][j]))
  last=reader.frame(reader.frame_count-1)
  result=dict(min_exit_distance_mm=best[0]*1000,min_exit_time_s=best[1],final_exit_distance_mm=float(np.linalg.norm(last['poses'][-1,:3]-route[-1])*1000),true_exit_m=route[-1].tolist(),frames=reader.frame_count,dt=reader.dt,verified=True,scope='Every physical frame; unshifted original geometry exit, not control waypoint.')
  audit.write_text(json.dumps(result,indent=2))
 result=json.loads(audit.read_text())
 rows.append(dict(run=run.name,reason=summary['reason'],simulated_s=summary['simulated_s'],wall_s=summary['wall_s'],route_cursor_mm=summary['progress_m']*1000,min_exit_mm=result['min_exit_distance_mm'],final_exit_mm=result['final_exit_distance_mm'],station_reached=summary['station_reached'],peak_rope_n=summary['peak_force_n'],max_stretch=summary['maximum_relative_edge_stretch'],parameters=json.dumps(contract.get('controller_parameters',{}),sort_keys=True)))
with (R/'tuning/experiment_table.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
print(json.dumps(rows,indent=2))
