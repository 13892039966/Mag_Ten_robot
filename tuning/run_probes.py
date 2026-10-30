"""Serial local screening. No checkpoint result counts as full-route success."""
import subprocess,sys,json,time
from pathlib import Path
R=Path(__file__).resolve().parents[1]
if (R/'tuning/probe_invalidated.json').exists():raise RuntimeError('Checkpoint reconstruction invalidated; see tuning/probe_invalidated.json. Use entry-state full runs.')
for name in sys.argv[1:]:
 output=R/'outputs'/('probe_'+name)
 if output.exists():raise FileExistsError(output)
 print('START PROBE',name,time.strftime('%H:%M:%S'),flush=True)
 cmd=[sys.executable,str(R/'tuning/probe.py'),'--source',str(R/'outputs/tune_lookahead25'),'--time','260','--seconds','50','--config',str(R/'tuning/configs'/f'{name}.json'),'--output',str(output)]
 with (R/'tuning'/f'probe_{name}.log').open('w') as f:result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 report={'name':name,'exit_code':result.returncode}
 if (output/'probe_report.json').exists():report.update(json.loads((output/'probe_report.json').read_text()))
 with (R/'tuning/probe_results.jsonl').open('a') as f:f.write(json.dumps(report)+'\n')
 print('END PROBE',json.dumps(report),flush=True)
 if report.get('reconstruction_error_mm',float('inf'))>.5:
  print('INVALID RECONSTRUCTION; STOP SCREENING',flush=True);break
 if report.get('reason')=='station_reached':break
