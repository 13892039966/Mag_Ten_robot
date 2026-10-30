"""Independent one-second exact-time comparison. No SOFA import."""
import json,sys
from pathlib import Path
import numpy as np
from tendon_sim.physical_recording import RecordingReader
def main():
    run=Path(sys.argv[1]);reference=Path(__file__).resolve().parent/'reference/windows_entry_1s'
    a,b=RecordingReader(run/'physical_recording'),RecordingReader(reference/'physical_recording')
    a.verify();b.verify()
    if a.dt!=.005 or b.dt!=.005 or a.frame_count<201:raise ValueError('Complete5ms1s record required')
    if a.metadata['robot']!=b.metadata['robot']:raise ValueError('Robot parameters differ')
    maximum=0.
    for i in range(201):
        x,y=a.frame(i),b.frame(i)
        if x['t']!=y['t']:raise ValueError('Different time grid')
        maximum=max(maximum,float(np.linalg.norm(x['poses'][:,:3]-y['poses'][:,:3],axis=1).max()*1000))
    lines=lambda p:{round(r['t']/.02):r for r in map(json.loads,p.read_text(encoding='utf-8').splitlines())}
    x,y=lines(run/'trace.jsonl'),lines(reference/'trace.jsonl');force=0.
    for k in range(1,51):force=max(force,float(np.abs(np.array(x[k]['force_n'])-y[k]['force_n']).max()))
    result=dict(seconds=1.,frames=201,max_position_mm=maximum,max_rope_force_n=force,passed=maximum<=.5 and force<=.1,
        scope='Windows/Linux one-second entry regression only; no-contact prefix, not long contact equivalence or hardware validation')
    (run/'reference_comparison.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
