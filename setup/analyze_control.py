"""Compare command prediction with observed motion; does not prove infeasibility."""
import argparse,json
from pathlib import Path
import numpy as np

def summarize(rows):
    rows=[r for r in rows if r.get('control_diagnostic')]
    if not rows:return dict(samples=0)
    d=[r['control_diagnostic'] for r in rows]
    action=np.array([x['action'] for x in d]);actual=np.array([x['executed_delta_m'] for x in d])
    requested=action*np.array([.001]*4+[.005])*.02
    pred=np.array([x['predicted_delta_m'] for x in d]);obs=np.array([x['observed_delta_m'] for x in d])
    command=np.array([r['q_m']+[r['feed_m']] for r in rows])
    speeds=np.array([.001]*4+[.005])
    windows=[]
    for i in range(0,len(rows)-19,20):
        p=pred[i:i+20].sum(axis=0);o=obs[i:i+20].sum(axis=0)
        pn=np.linalg.norm(p);on=np.linalg.norm(o)
        windows.append(dict(t=rows[i+19]['t'],predicted_mm=float(pn*1000),observed_mm=float(on*1000),
            residual_mm=float(np.linalg.norm(o-p)*1000),cosine=float(p@o/(pn*on)) if pn>1e-8 and on>1e-8 else None))
    cos=[w['cosine'] for w in windows if w['cosine'] is not None]
    corrections=[x['correction'] for x in d if x.get('correction')]
    return dict(samples=len(rows),start_s=rows[0]['t'],end_s=rows[-1]['t'],
        minimum_error_mm=min(r['error_mm'] for r in rows),final_error_mm=rows[-1]['error_mm'],
        saturated_command_fraction=float(np.mean(np.any(abs(command[:,:4])>=.03-1e-6,axis=1))),
        requested_executed_mismatch_fraction=float(np.mean(np.max(abs(requested-actual),axis=1)>1e-9)),
        max_requested_executed_mismatch_mm=float(np.max(abs(requested-actual))*1000),
        all_actions_small_fraction=float(np.mean(np.max(abs(action),axis=1)<.01)),
        rms_action=np.sqrt(np.mean(action**2,axis=0)).tolist(),
        max_executed_speed_ratio=float(np.max(abs(actual)/(speeds*.02))),
        max_cable_command_mm=float(abs(command[:,:4]).max()*1000),
        minimum_feed_mm=(command[:,4]*1000).min().item(),
        maximum_feed_mm=(command[:,4]*1000).max().item(),
        median_400ms_direction_cosine=float(np.median(cos)) if cos else None,
        median_400ms_prediction_residual_mm=float(np.median([w['residual_mm'] for w in windows])) if windows else None,
        final_correction=corrections[-1] if corrections else None,windows_400ms=windows)

def analyze(path):
    path=Path(path);rows=[json.loads(x) for x in (path/'trace.jsonl').read_text().splitlines()]
    contract=json.loads((path/'run_contract.json').read_text())
    if contract['robot']['dt']!=.02 or contract.get('control_period_s')!=.02 or any(contract['controller_parameters'].get(k,1)!=1 for k in ['cable_speed_scale','feed_speed_scale']):
        raise ValueError('This diagnostic comparison is configured for 20ms and original actuator speeds')
    last=rows[-1]['waypoint_index'];target=[r for r in rows if r['waypoint_index']==last]
    result=dict(run=path.name,contract=json.loads((path/'run_contract.json').read_text()),
        summary=json.loads((path/'summary.json').read_text()),all=summarize(rows),last_waypoint_index=last,
        last_waypoint=summarize(target),last_10s=summarize([r for r in target if r['t']>=rows[-1]['t']-10]),
        scope='Command-step secant diagnostics at dt/control=20ms; window residuals include dynamics and contact. Correlation is not proof of causality or physical infeasibility.')
    (path/'control_analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args()
    result=analyze(args.run)
    print(json.dumps({k:v for k,v in result.items() if k not in ['contract','all']},indent=2))
