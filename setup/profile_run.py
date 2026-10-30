"""Instrument the normal runner without replacing state, inputs, physics or safety."""
import functools
import json
from pathlib import Path
import sys
import time
from collections import defaultdict

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import run
from tendon_sim.physics import Sofa
from SofaRuntime import Timer
from tendon_sim.live_scene import Trial
from tendon_sim.closed_loop import BoundedDLS
from tendon_sim.fast_safety import FastPhantomSafety
from tendon_sim.geometry_adapter import PhantomQueries
from tendon_sim.physical_recording import ChunkRecorder

if '--help' in sys.argv or '--output' not in sys.argv:
    raise SystemExit(run.main())

minimized='--profile-minimized' in sys.argv
if minimized:sys.argv.remove('--profile-minimized')
output=Path(sys.argv[sys.argv.index('--output')+1])
metrics=defaultdict(lambda:dict(calls=0,wall_s=0.,cpu_s=0.,max_wall_s=0.))
steps=0;native_samples=[];windows=[];last_saved=time.perf_counter();started=last_saved


def instrument(owner,name,label):
    original=getattr(owner,name)
    @functools.wraps(original)
    def wrapped(*args,**kwargs):
        w,c=time.perf_counter(),time.process_time()
        try:return original(*args,**kwargs)
        finally:
            wall=time.perf_counter()-w;cpu=time.process_time()-c;m=metrics[label]
            m['calls']+=1;m['wall_s']+=wall;m['cpu_s']+=cpu;m['max_wall_s']=max(m['max_wall_s'],wall)
    setattr(owner,name,wrapped)


for owner,name,label in [
    (Trial,'onAnimateBeginEvent','begin_callback'),(Trial,'onAnimateEndEvent','end_callback'),
    (BoundedDLS,'command','DLS_control'),(FastPhantomSafety,'check','safety'),
    (PhantomQueries,'wall_distance_m','safety_exact_triangles'),
    (ChunkRecorder,'append','record_append'),(ChunkRecorder,'_flush','record_flush'),
    (ChunkRecorder,'_manifest','record_manifest')]:instrument(owner,name,label)
if '--gui' in sys.argv:
    from tendon_sim.gui_view import SofaWindow,SofaViewport
    if minimized:
        original_show=SofaWindow.show
        def show_minimized(self):
            original_show(self);self.window.showMinimized();self.app.processEvents()
        SofaWindow.show=show_minimized
    for owner,name,label in [(SofaWindow,'process','GUI_process'),(SofaViewport,'paintGL','GUI_paint'),(SofaWindow,'screenshot','GUI_screenshot')]:instrument(owner,name,label)


def save():
    if not output.exists():return
    animate=metrics['animate']['wall_s'];begin=metrics['begin_callback']['wall_s'];end=metrics['end_callback']['wall_s']
    report=dict(gui_minimized=minimized,steps=steps,measured_process_wall_s=time.perf_counter()-started,metrics=dict(metrics),
                disjoint_animate=dict(engine_and_native_s=max(0.,animate-begin-end),begin_callback_s=begin,end_callback_s=end),
                native_timer_samples=native_samples,cumulative_windows=windows,
                scope='Instrumented fresh-entry run; interpreter/import startup excluded; metrics are inclusive unless explicitly disjoint; GUI paint and screenshot overlap GUI process. Not a profile of the existing long-run state.')
    temp=output/'profile.tmp';temp.write_text(json.dumps(report,indent=2));temp.replace(output/'profile.json')


original_animate=Sofa.Simulation.animate
Timer.setEnabled('ProfileStep',True)
@functools.wraps(original_animate)
def animate(*args,**kwargs):
    global steps,last_saved
    w,c=time.perf_counter(),time.process_time();Timer.begin('ProfileStep')
    try:return original_animate(*args,**kwargs)
    finally:
        steps+=1
        if steps in (1,50,500,1000,2500,5000,7500) or steps%5000==0:
            try:native_samples.append(dict(step=steps,records=Timer.getRecords('ProfileStep')))
            except Exception as error:native_samples.append(dict(step=steps,error=str(error)))
        Timer.end('ProfileStep')
        m=metrics['animate'];elapsed=time.perf_counter()-w
        m['calls']+=1;m['wall_s']+=elapsed;m['cpu_s']+=time.process_time()-c;m['max_wall_s']=max(m['max_wall_s'],elapsed)
        if steps%2500==0:windows.append(dict(step=steps,process_wall_s=time.perf_counter()-started,metrics=json.loads(json.dumps(dict(metrics)))))
        if time.perf_counter()-last_saved>=2.:save();last_saved=time.perf_counter()
Sofa.Simulation.animate=animate
try:status=run.main()
finally:save()
raise SystemExit(status)
