"""Independent route/endpoint metrics, without SOFA or old-colon geometry."""
import sys,json,csv,argparse
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
from tendon_sim.physical_recording import RecordingReader
from tendon_sim.recorded_geometry import load_recorded_geometry

def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);args=parser.parse_args();run=args.run
    reader=RecordingReader(run/'physical_recording');reader.verify();geometry,route=load_recorded_geometry(reader.metadata)
    delta=np.diff(route,axis=0);length=np.linalg.norm(delta,axis=1);arc=np.r_[0,np.cumsum(length)]
    def project(point):
        fraction=np.clip(np.sum((np.asarray(point)-route[:-1])*delta,axis=1)/length**2,0,1)
        d=np.linalg.norm(route[:-1]+fraction[:,None]*delta-point,axis=1);i=int(d.argmin())
        return float(arc[i]+fraction[i]*length[i]),float(d[i])
    last=reader.frame(reader.frame_count-1);first=reader.frame(0)
    np.testing.assert_allclose(first['poses'][-1,:3],geometry['entry_position_m'],atol=1e-12,rtol=0)
    trajectory=reader.metadata.get('trajectory')
    completed=0
    if trajectory:
        import hashlib
        from tendon_sim.references import load_trajectory
        path=run/'trajectory.csv'
        assert hashlib.sha256(path.read_bytes()).hexdigest()==trajectory['sha256']
        targets=load_trajectory(path)
        assert len(targets)==trajectory['points']
    rows=[]
    for line in (run/'trace.jsonl').read_text(encoding='utf-8').splitlines():
        try:row=json.loads(line)
        except json.JSONDecodeError:continue
        if row['t']>last['t']+1e-9:continue
        f=reader.frame(int(round(row['t']/reader.dt)))
        np.testing.assert_allclose(f['poses'][-1,:3],row['tip_m'],atol=1e-12,rtol=0)
        if trajectory:
            index=min(completed,len(targets)-1)
            assert row['waypoint_index']==index
            np.testing.assert_array_equal(row['goal_m'],targets[index])
            distance=float(np.linalg.norm(np.array(row['tip_m'])-targets[index]))
            expected=completed<len(targets) and distance<=.005 and not reader.metadata.get('zero_inputs',False)
            assert row['waypoint_completed']==expected
            completed+=int(expected)
            assert row['completed_waypoints']==completed
        station,lateral=project(np.array(row['tip_m']))
        rows.append(dict(t=row['t'],route_cursor_mm=row['progress_m']*1000,actual_station_mm=station*1000,
            centerline_distance_mm=lateral*1000,target_error_mm=row['error_mm'],feed_mm=row['feed_m']*1000))
    station,lateral=project(last['poses'][-1,:3]);end_distance=float(np.linalg.norm(last['poses'][-1,:3]-route[-1]))
    summary=json.loads((run/'summary.json').read_text()) if (run/'summary.json').exists() else None
    if summary:
        assert reader.frame_count==summary['physics_steps']+1
        assert abs(last['t']-summary['simulated_s'])<1e-9
        if summary['station_reached']:assert end_distance<=.005+1e-9
        if trajectory:assert summary['completed_waypoints']==completed
    with (run/'route_diagnostics.csv').open('w',newline='',encoding='utf-8') as output:
        writer=csv.DictWriter(output,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    result=dict(status=summary['reason'] if summary else 'in_progress_committed_prefix',
        waypoint_trace_verified=bool(trajectory),completed_waypoints=completed if trajectory else None,
        task_finished=summary is not None,station_reached=summary['station_reached'] if summary else False,
        raw_frames=reader.frame_count,recorded_end_s=last['t'],route_length_mm=arc[-1]*1000,
        final_actual_station_mm=station*1000,final_centerline_distance_mm=lateral*1000,
        tip_to_exit_distance_mm=end_distance*1000,
        max_centerline_distance_mm=max(x['centerline_distance_mm'] for x in rows),
        initial_state_sha256=reader.manifest['initial_state_sha256'],
        asset=reader.metadata['geometry_asset'],
        scope='Independent geometric projections and saved-frame checks; target error is not centerline lateral error.')
    (run/'independent_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
