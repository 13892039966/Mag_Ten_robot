"""Headless full-body libigl navigation; target platform Ubuntu22.04 x86_64."""
import os
for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import argparse,json,time,platform,hashlib,signal,subprocess,sys
from pathlib import Path

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dt',type=float,choices=[.005,.01,.02],default=.005)
    parser.add_argument('--cpu',type=int,choices=[1,4],default=4)
    duration=parser.add_mutually_exclusive_group()
    duration.add_argument('--seconds',type=float)
    duration.add_argument('--until-terminal',action='store_true')
    parser.add_argument('--zero',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--no-gif',action='store_true',help='Skip automatic route analysis and wall GIF')
    parser.add_argument('--gif-sample-seconds',type=float,default=1.,help='Simulation seconds between GIF frames (default: 1)')
    parser.add_argument('--gif-frame-ms',type=int,default=100,help='GIF frame duration in milliseconds (default: 100)')
    parser.add_argument('--controller-config',type=Path,help='JSON control parameters; physical limits unchanged')
    parser.add_argument('--wall-seconds',type=float,default=None,help='Optional experiment wall-time limit')
    a=parser.parse_args()
    controller_config=json.loads(a.controller_config.read_text()) if a.controller_config else {}
    allowed={'lookahead','position_gain_per_s','velocity_damping','damping_mm_s','smoothing_mm_s','feed_speed_scale','cable_speed_scale'}
    if not isinstance(controller_config,dict) or set(controller_config)-allowed:parser.error('Unsupported controller config keys')
    import math
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in controller_config.values()):parser.error('Controller parameters must be finite and positive')
    if any(controller_config.get(k,1)>1 for k in ['feed_speed_scale','cable_speed_scale']):parser.error('Speed scales cannot exceed original physical speed limits')
    if a.wall_seconds is not None and (not math.isfinite(a.wall_seconds) or a.wall_seconds<=0):parser.error('Invalid wall limit')
    if not 0<a.gif_sample_seconds<float('inf') or a.gif_frame_ms<10:parser.error('GIF sampling must be finite and positive; frame duration must be >=10ms')
    if a.seconds is None and not a.until_terminal:a.seconds=1.
    if a.seconds is not None and not 0<a.seconds<=300:parser.error('seconds must be in (0,300]')
    if a.zero and a.until_terminal:parser.error('Zero input requires bounded duration')
    if a.output.exists():raise FileExistsError(a.output)
    from dataclasses import asdict
    import numpy as np
    from tendon_sim.physics import Sofa,SofaRuntime,build_physics
    from tendon_sim.body_geometry import BodyConfig,body_layout
    from tendon_sim.references import RouteReference
    from tendon_sim.live_scene import Trial
    from tendon_sim.geometry_adapter import load_phantom,DEFAULT_ROOT
    from tendon_sim.fast_safety import FastPhantomSafety
    from tendon_sim.scheduler import seed_scheduler,actual_thread_count
    from tendon_sim.physical_recording import RecordingReader
    from tendon_sim.libigl_wall import replace_wall_pre_init,DLL,ASSETS,OBJ,get_hybrid_stats
    began=time.perf_counter();g,route=load_phantom();original_mesh=g['mesh_path']
    c=BodyConfig(dt=a.dt,feed_limit_m=1.4)
    d=body_layout(c,entry=g['entry_position_m'],direction=g['entry_direction'],insertion=0.)
    reference=RouteReference(route,maximum_station=None,lookahead=controller_config.get('lookahead',.008));reference.seed_from_tip(d['poses'][-1,:3])
    g=dict(g,mesh_path=str(Path(__file__).resolve().parent/'assets/collision/wall_collision_ax2_ang64_m.stl'))
    if a.cpu==4:SofaRuntime.importPlugin('MultiThreading')
    seed_scheduler(a.cpu);root=Sofa.Core.Node('PortableLibiglFullBody');trial=None;stopped=[]
    signal.signal(signal.SIGINT,lambda *_:stopped.append('SIGINT'))
    signal.signal(signal.SIGTERM,lambda *_:stopped.append('SIGTERM'))
    try:
        h=build_physics(root,c,g,d,native_rate_factor=0.)
        if a.cpu==4:
            root.getObject('constraints').multithreading.value=True
            h['beam'].getObject('linear').parallelInverseProduct.value=True
            root.getObject('FreeMotionAnimationLoop').parallelCollisionDetectionAndFreeMotion.value=True
            root.removeObject(root.getObject('BVHNarrowPhase'));root.addObject('ParallelBVHNarrowPhase',name='parallel_narrow',nbThreads=4)
        replace_wall_pre_init(root)
        trial=Trial(h,a.output,reference,seconds=a.seconds,wall_seconds=None,zero=a.zero,until_terminal=a.until_terminal,
            safety_factory=FastPhantomSafety,controller_config=controller_config,name='Control',listening=True)
        root.addObject(trial)
        asset=dict(kind='user_phantom40_complete_wall',mesh_path=original_mesh,geometry_npz=str(DEFAULT_ROOT/'geometry_world_m.npz'),
            mesh_sha256=sha(original_mesh),geometry_npz_sha256=sha(DEFAULT_ROOT/'geometry_world_m.npz'),flipNormals=False)
        trial.recorder.manifest['metadata'].update(wall_time_limit_s=a.wall_seconds,wall_timeout_handling='checked_between_physics_steps',geometry_asset=asset,backend='libigl2.6.0_single_closest',collision_triangles=171776,
            source_sha256={str(p.relative_to(Path(__file__).parent)):sha(p) for p in (Path(__file__).parent/'tendon_sim').glob('*.py')})
        trial.recorder._manifest();Sofa.Simulation.init(root);seed_scheduler(a.cpu)
        assert actual_thread_count()==a.cpu and trial.interval==round(.02/a.dt)
        assert all(not cable.maxDispVariation.isSet() and abs(float(cable.maxForce.value)/a.dt-2)<1e-12 for cable in h['cables'])
        np.testing.assert_allclose(h['state'].position.value,d['poses'],rtol=0,atol=1e-14)
        assert not np.any(h['state'].velocity.value)
        if not trial.recorder.manifest['total_frames']:trial.initialized()
        contract=dict(wall_time_limit_s=a.wall_seconds,wall_timeout_handling='checked_between_physics_steps',runner_sha256=sha(__file__),controller_parameters=controller_config,robot=asdict(c),platform=platform.platform(),python=platform.python_version(),cpu_pool_actual=actual_thread_count(),
            control_period_s=.02,control_stride=trial.interval,route_length_m=reference.length_m,initial_insertion_m=0.,
            native_rate_factor=0.,backend='libigl2.6.0_single_closest',dll_sha256=sha(DLL),obj_sha256=sha(OBJ),
            raw_sha256=sha(ASSETS/'wall_libigl_4mm_f64.raw'),force_units='maxForce=2N*dt, measured force=impulse/dt',
            source_only_linux_runtime_not_prevalidated=True,known_model_limit='Existing duplicate BaseMass warning retained; hardware calibration unverified')
        (a.output/'run_contract.json').write_text(json.dumps(contract,indent=2),encoding='utf-8')
        initial=trial.safety.check(np.asarray(h['collision'].position.value),'',[])
        if initial['invalid_reason']:trial.finish('invalid_initial_state',initial['invalid_reason'])
        setup=time.perf_counter()-began;elapsed=cpu=0.
        while not trial.done:
            if stopped:trial.finish('user_interrupted',stopped[-1]);break
            # Stop before entering animate: onAnimateBegin cannot cancel an
            # already-started physics step after closing the recording.
            if a.wall_seconds is not None and time.monotonic()-trial.started>=a.wall_seconds:
                trial.finish('wall_timeout');break
            w,cp=time.perf_counter(),time.process_time();Sofa.Simulation.animate(root,c.dt)
            elapsed+=time.perf_counter()-w;cpu+=time.process_time()-cp
        reader=RecordingReader(a.output/'physical_recording');reader.verify()
        assert reader.frame_count==trial.ticks+1
        np.testing.assert_array_equal(reader.frame(reader.frame_count-1)['poses'],h['state'].position.value)
        (a.output/'recording_verification.json').write_text(json.dumps(dict(verified=True,frames=reader.frame_count,reason=trial.summary['reason']),indent=2),encoding='utf-8')
        (a.output/'timing.json').write_text(json.dumps(dict(setup_wall_s=setup,animate_including_callbacks_wall_s=elapsed,
            animate_cpu_s=cpu,simulated_s=trial.ticks*c.dt,total_before_unload_wall_s=time.perf_counter()-began,
            seconds_real_per_sim=elapsed/(trial.ticks*c.dt) if trial.ticks else None,query_stats=get_hybrid_stats()),indent=2),encoding='utf-8')
        run_status=0 if trial.summary['reason'] in ('duration_completed','zero_input_completed','station_reached','stalled_at_cable_limit') else 2
    finally:
        if trial is not None and not trial.done:trial.finish('runner_exception')
        Sofa.Simulation.unload(root)

    # Separate processes release SOFA before loading the full rendering mesh.
    # Rendering never changes the simulation's stop reason or physics timing.
    if not a.no_gif:
        scripts=Path(__file__).resolve().parent
        report=dict(status='running',simulation_exit_code=run_status,stages=[])
        report_path=a.output/'postprocess.json'
        for name,command in [
            ('analysis',[sys.executable,str(scripts/'analyze.py'),str(a.output)]),
            ('gif',[sys.executable,str(scripts/'render.py'),str(a.output),
                    '--sample-seconds',str(a.gif_sample_seconds),'--frame-ms',str(a.gif_frame_ms)])]:
            print('Postprocessing: '+name,flush=True)
            report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
            result=subprocess.run(command,check=False)
            report['stages'].append(dict(name=name,exit_code=result.returncode))
            if result.returncode:
                report['status']='failed'
                report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
                return run_status or 3
        report['status']='completed'
        report['gif']=str(a.output/'wholebody_with_colon.gif')
        report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
    return run_status

if __name__=='__main__':raise SystemExit(main())
