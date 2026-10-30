"""Actual original wall mesh + unchanged recorded robot; fixed-camera cached render."""
import sys,json,argparse,hashlib,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch
from PIL import Image,GifImagePlugin
from tendon_sim.physical_recording import RecordingReader
from tendon_sim.recorded_geometry import load_recorded_geometry
from tendon_sim.stl_io import read_stl

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path)
    parser.add_argument('--sample-seconds',type=float,default=.1);parser.add_argument('--frame-ms',type=int,default=50)
    a=parser.parse_args();run=a.run;started=time.monotonic()
    reader=RecordingReader(run/'physical_recording');reader.verify()
    phantom=reader.metadata.get('geometry_asset',{}).get('kind')=='user_phantom40_complete_wall'
    if reader.metadata.get('mode')!='colon':raise ValueError('Colon recording required')
    source_hash=sha(run/'physical_recording/manifest.json')
    geometry,recorded_route=load_recorded_geometry(reader.metadata);mesh_path=Path(geometry['mesh_path']);tri=read_stl(mesh_path)*1000
    if reader.metadata.get('trajectory'):
        from tendon_sim.references import load_trajectory
        trajectory_path=run/'trajectory.csv'
        assert sha(trajectory_path)==reader.metadata['trajectory']['sha256']
        recorded_route=load_trajectory(trajectory_path)
    route=recorded_route*1000;entry=geometry['entry_position_m']*1000
    first=reader.frame(0);boundary=reader.metadata['layout']['active_index']
    rows=[json.loads(x) for x in (run/'trace.jsonl').read_text().splitlines()]
    times=np.array([x['t'] for x in rows]);errors=np.array([x['error_mm'] for x in rows]);goals=np.array([x['goal_m'] for x in rows])*1000
    for row in rows:
        f=reader.frame(int(round(row['t']/reader.dt)))
        np.testing.assert_allclose(f['poses'][-1,:3],row['tip_m'],rtol=0,atol=1e-12)
        assert abs(np.linalg.norm(np.array(row['goal_m'])-f['poses'][-1,:3])*1000-row['error_mm'])<1e-9
    tips=np.array([x['tip_m'] for x in rows])*1000
    # Include all actual active nodes throughout motion, not just the tip trajectory.
    active_samples=np.vstack([reader.frame(i)['poses'][boundary:,:3]*1000 for i in range(0,reader.frame_count,20)])
    mesh_low=tri.min(axis=(0,1));mesh_high=tri.max(axis=(0,1))
    full_points=np.vstack([first['poses'][:,:3]*1000,tri.reshape(-1,3),tips])
    full_bounds=(full_points.min(0)-20,full_points.max(0)+20)
    colon_bounds=(mesh_low-15,mesh_high+15)
    head_points=np.vstack([active_samples,goals]);head_bounds=(head_points.min(0)-20,head_points.max(0)+20)
    intersect=np.all(tri.max(1)>=head_bounds[0],axis=1)&np.all(tri.min(1)<=head_bounds[1],axis=1)
    plt.rcParams.update({'font.size':9})
    fig=plt.figure(figsize=(14,8),dpi=100,facecolor='white')
    gs=fig.add_gridspec(2,3,height_ratios=[2.15,1.],hspace=.20,wspace=.10)
    full=fig.add_subplot(gs[0,0],projection='3d');colon=fig.add_subplot(gs[0,1],projection='3d');head=fig.add_subplot(gs[0,2],projection='3d')
    error_ax=fig.add_subplot(gs[1,:]);panels=[full,colon,head]
    for ax,bounds,title in zip(panels,[full_bounds,colon_bounds,head_bounds],
        ['Whole 2 m body / centreline context','User phantom tube and robot' if phantom else 'Original colon and inserted robot','Active head / tube wall' if phantom else 'Active head / wall close-up']):
        low,high=bounds;ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(low[2],high[2]),title=title)
        ax.set_box_aspect(np.maximum(high-low,20));ax.view_init(21,-60)
        ax.set_xticks([]);ax.set_yticks([]);ax.set_zticks([])
        ax.set_facecolor('white')
    full.view_init(22,-65)
    def add_wall(ax,faces,alpha):
        normal=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0]);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-30)
        light=np.array([.25,-.4,.88]);light/=np.linalg.norm(light)
        shade=.65+.35*np.abs(normal@light)
        rgba=np.column_stack([np.array([.87,.48,.43])[None,:]*shade[:,None],np.full(len(faces),alpha)])
        obj=Poly3DCollection(faces,facecolors=rgba,edgecolors='none',linewidths=0,zsort='average',rasterized=True)
        ax.add_collection3d(obj)
    # Exact original triangles are projected once per fixed camera. No decimation.
    add_wall(colon,tri,.13);add_wall(head,tri[intersect],.12)
    def clipped(points,bounds):
        out=np.array(points,float,copy=True);low,high=bounds
        out[np.any((out<low)|(out>high),axis=1)]=np.nan
        return out
    for ax,bounds in zip(panels,[full_bounds,colon_bounds,head_bounds]):
        rr=clipped(route,bounds);ax.plot(*rr.T,'--',color='#6e5b58',lw=.8,alpha=.6)
        ax.plot([entry[0]],[entry[1]],[entry[2]],'o',color='#247947',ms=4)
    full.legend(handles=[Patch(color='#287abd',label='Passive 1800 mm'),Patch(color='#e68c24',label='Active 200 mm')],loc='lower left',fontsize=8)
    colon.legend(handles=[Patch(facecolor='#d78f83',alpha=.4,label='Original STL wall (transparent)')],loc='lower left',fontsize=8)
    error_ax.plot(times,errors,color='#334466',lw=1.3)
    error_ax.set(xlabel='Recorded simulation time (s)',ylabel='Tip reference error (mm)',xlim=(0,reader.frame(reader.frame_count-1)['t']))
    diagnostics=run/'route_diagnostics.csv'
    if phantom and diagnostics.exists():
        metrics=np.atleast_1d(np.genfromtxt(diagnostics,delimiter=',',names=True))
        error_ax.lines[0].set_label('Distance to current target')
        error_ax.plot(metrics['t'],metrics['centerline_distance_mm'],color='#278260',lw=1.3,label='Distance to centreline')
        error_ax.set_ylabel('Distance (mm)');error_ax.legend(loc='upper left',fontsize=8)
    if reader.metadata.get('trajectory',{}).get('mode')=='ordered_waypoints':
        radius_mm=reader.metadata['trajectory']['arrival_radius_m']*1000
        error_ax.axhline(radius_mm,color='#888888',ls='--',lw=.8,label='Arrival radius: %g mm'%radius_mm)
        error_ax.legend(loc='upper left',fontsize=8)
    error_ax.grid(alpha=.2)
    dynamics=[]
    for ax in panels:
        passive,=ax.plot([],[],[],color='#287abd',lw=3,animated=True)
        active,=ax.plot([],[],[],color='#e68c24',lw=4,animated=True)
        dot,=ax.plot([],[],[],'o',color='#c93131',ms=4,animated=True)
        dynamics.append((passive,active,dot))
    cursor=error_ax.axvline(0,color='#ba3737',lw=1,animated=True)
    title=fig.suptitle('',fontsize=13);title.set_animated(True)
    fig.text(.5,.015,'Blue: passive shaft | Orange: active head | Red: reference | Same recorded physics; wall added for context',ha='center',fontsize=9,color='#444444')
    fig.subplots_adjust(left=.055,right=.985,top=.91,bottom=.09)
    print('Rendering cached original wall: %d triangles, local %d'%(len(tri),int(intersect.sum())),flush=True)
    fig.canvas.draw();background=fig.canvas.copy_from_bbox(fig.bbox)
    def setline(line,points):line.set_data(points[:,0],points[:,1]);line.set_3d_properties(points[:,2])
    def update(index):
        frame=reader.frame(index);xyz=frame['poses'][:,:3]*1000;t=frame['t'];k=min(len(rows)-1,max(0,int(np.searchsorted(times,t,side='right')-1)))
        fig.canvas.restore_region(background)
        for ax,parts,bounds in zip(panels,dynamics,[full_bounds,colon_bounds,head_bounds]):
            setline(parts[0],clipped(xyz[:boundary+1],bounds));setline(parts[1],clipped(xyz[boundary:],bounds))
            setline(parts[2],goals[k:k+1])
            for artist in parts:ax.draw_artist(artist)
        cursor.set_xdata([t,t]);error_ax.draw_artist(cursor)
        scene_name='PHANTOM TUBE ID40mm' if phantom else 'ORIGINAL COLON'
        kind=('PREVIEW: SOFA PHYSICS + ' if reader.metadata.get('is_preview') else 'RECORDED SOFA PHYSICS + ')+scene_name
        title.set_text(kind+' | t=%.2fs | error=%.2fmm'%(t,errors[k]));fig.draw_artist(title)
        return np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy()
    stride=max(1,int(round(a.sample_seconds/reader.dt)));indices=list(range(0,reader.frame_count,stride))
    if indices[-1]!=reader.frame_count-1:indices.append(reader.frame_count-1)
    output=run/'wholebody_with_colon.gif'
    with output.open('wb') as stream:
        for n,i in enumerate(indices):
            im=Image.fromarray(update(i))
            if n==0:
                indexed=im.quantize(colors=192,method=2);palette=indexed.copy();blocks,_=GifImagePlugin.getheader(indexed,info={'loop':0})
                for block in blocks:stream.write(block)
                im.save(run/'colon_context_first.png')
            else:indexed=im.quantize(palette=palette,dither=0)
            for block in GifImagePlugin.getdata(indexed,duration=a.frame_ms,disposal=1):stream.write(block)
            if n%50==0:print('Rendered %d/%d'%(n+1,len(indices)),flush=True)
        stream.write(b';')
    Image.fromarray(update(reader.frame_count-1)).save(run/'colon_context_final.png');plt.close(fig)
    with Image.open(output) as gif:
        count=gif.n_frames
        for i in range(count):gif.seek(i);gif.load()
    assert count==len(indices)
    assert sha(run/'physical_recording/manifest.json')==source_hash
    result=dict(mesh_path=str(mesh_path.resolve()),mesh_sha256=sha(mesh_path),original_mesh_triangles=len(tri),
        local_mesh_triangles=int(intersect.sum()),mesh_decimated=False,original_manifest_sha256=source_hash,
        raw_frames=reader.frame_count,gif_frames=count,sample_stride_steps=stride,frame_duration_ms=a.frame_ms,
        playback_speed=(stride*reader.dt)/(a.frame_ms/1000),render_wall_s=time.monotonic()-started,
        renderer_sha256=sha(__file__),scope='Original wall geometry and unchanged recorded poses; fixed-camera projection cached for rendering only. No physics rerun.')
    (run/'colon_render_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
