"""Actual SOFA physics with every-step full-body recording and observable progress."""
import os,sys,json,time,datetime,uuid,atexit,hashlib
from pathlib import Path
from dataclasses import asdict
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
from .physics import Sofa,SofaRuntime,build_physics,submit_targets
from .body_geometry import BodyConfig
from .references import FarReference,RouteReference
from .physical_recording import ChunkRecorder
from .closed_loop import BoundedDLS,DLSConfig,slew_targets
from .logging_output import AsyncKeyboardOutput
from .actuator_gauge import differential_lengths,differential_jacobian
from .run_limits import checked_limit,reached_limit,control_stride
from .released_active import released_active_segment


class WorldModel:
    """Local active CC sensitivity; feed direction is an approximation under contact."""
    def __init__(self,original,h):
        self.original,self.h,self.config=original,h,original.config
    def jacobian(self,q,feed):
        p=np.asarray(self.h['state'].position.value)
        j=self.original.jacobian(q,feed)
        index=self.h['definition']['active_index'];scale=1.
        if self.h['geometry']:
            free_index,free_length=released_active_segment(self.h['definition']['abscissa'],index,self.h['feeder'].points.value)
            if free_index>index:index=free_index;scale=free_length/self.config.active_length_m
        r=Rotation.from_quat(p[index,3:]).as_matrix()
        j[:,:4]=scale*(r@j[:,:4])
        j[:,4]=Rotation.from_quat(p[-1,3:]).as_matrix()[:,0]
        return differential_jacobian(j)


class Trial(Sofa.Core.Controller):
    def __init__(self,h,output,reference,visual=False,seconds=None,wall_seconds=300.,zero=False,until_terminal=False,safety_factory=None,controller_config=None,**kwargs):
        super().__init__(**kwargs)
        self.h,self.reference,self.output=h,reference,Path(output)
        self.output.mkdir(parents=True,exist_ok=True)
        self.controller_config=dict(controller_config or {})
        dls={k:v for k,v in self.controller_config.items() if k in DLSConfig.__dataclass_fields__}
        self.policy=BoundedDLS(config=DLSConfig(**dls),robot_config=h['config']);self.policy.model=WorldModel(self.policy.model,h)
        self.policy.speeds[:4]*=self.controller_config.get('cable_speed_scale',1.)
        self.policy.speeds[4]*=self.controller_config.get('feed_speed_scale',1.)
        self.interval=control_stride(self.policy.config.control_period_s,h['config'].dt)
        self.status_interval=max(1,int(round(1./h['config'].dt)))
        self.ticks=0;self.done=False;self.started=time.monotonic();self.wall_seconds=checked_limit(wall_seconds)
        self.duration=None if until_terminal else min(reference.duration_s,checked_limit(seconds)) if seconds is not None else reference.duration_s
        self.zero=zero;self.desired=np.zeros(5);self.previous=np.zeros(5);self.last={}
        self.peak_stretch=0.;self.peak_force=0.;self.max_move=0.;self.errors=[]
        self.safety=None;self.contact=dict(contact_force_n=0.,contact_rows=0,penetration_m=0.,invalid_reason='')
        self.peak_contact=0.;self.peak_penetration=0.
        self.error_history=[]
        if h['geometry']:
            if safety_factory is None:
                from .fast_safety import FastPhantomSafety as ColonSafety
                safety_factory=ColonSafety
            self.safety=safety_factory(h['geometry'],h['config'].radius_m,h['config'].dt)
        self.trace=(self.output/'trace.jsonl').open('w',encoding='utf-8',buffering=1)
        self.emit=AsyncKeyboardOutput(self.output/'progress.log')
        d=h['definition'];c=h['config']
        metadata=dict(controller_parameters=self.controller_config,robot=asdict(c),model_label='active%gmm_passive%gmm'%(c.active_length_m*1000,c.passive_length_m*1000),mode='colon' if h['geometry'] else 'free',
            layout=dict(active_index=d['active_index'],entry=d['entry'].tolist(),direction=d['direction'].tolist(),
                        initial_insertion=d['initial_insertion']),
            physics='native_SOFA_BeamAdapter',dt=c.dt,gravity=[0,0,0],
            controller='bounded DLS, released active CC Jacobian after last guide pin, actual free-junction rotation; approximate feed column',
            outside_guide='original upstream -5mm release boundary' if h['geometry'] else 'base only',
            reference_duration_s=reference.duration_s,requested_seconds=self.duration,wall_time_limit_s=self.wall_seconds,until_terminal=until_terminal,zero_inputs=zero,
            control_period_s=self.policy.config.control_period_s,control_stride_steps=self.interval,
            contacts=bool(h['geometry']),learning=False,
            native_rate_factor=h.get('native_rate_factor',0.),command_slew_speed_m_s=c.cable_speed_m_s,
            prepared_centerline_start=bool(d.get('preinserted')),natural_beam_shape='straight',
            required_initial_shortening_m=d.get('required_initial_shortening_m',np.zeros(4)).tolist(),
            source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},
            actuator_allocation='differential tendon command integrator, feed for axial travel; every-step slew and range limits')
        self.recorder=ChunkRecorder(self.output/'physical_recording',c.dt,metadata,len(d['poses']))
        self.view=None
        if visual:raise ValueError('Headless transfer package')
        atexit.register(self.interrupted)
    def initialized(self):
        self.record();self.update_status()
        self.h['root'].animate.value=True
        self.emit('[LIVE] initialized; auto-running; '+str(self.output))
    def actual_q(self):
        return np.array([float(x.cableInitialLength.value)-float(x.cableLength.value) for x in self.h['cables']])
    def record(self):
        self.recorder.append(self.ticks*self.h['config'].dt,np.asarray(self.h['state'].position.value),
            np.asarray(self.h['state'].velocity.value),np.r_[self.h['shortening'],self.h['feed_m']])
    def onSimulationInitDoneEvent(self,event):
        if self.recorder.manifest['total_frames']==0:self.initialized()
    def onAnimateBeginEvent(self,event):
        if self.done:return
        try:
            if self.recorder.manifest['total_frames']==0:self.initialized()
            if reached_limit(time.monotonic()-self.started,self.wall_seconds):self.finish('wall_timeout');return
            p=np.asarray(self.h['state'].position.value);v=np.asarray(self.h['state'].velocity.value)
            if self.ticks%self.interval==0 and not self.zero:
                goal,_=self.reference.target(self.ticks*self.h['config'].dt,p[-1,:3])
                self.last=self.policy.command(differential_lengths(self.actual_q()),self.h['feed_m'],p[-1,:3],v[-1,:3],goal,self.previous)
                self.previous=self.last['action']
                # Slack tendons do not track guide length; integrate the bounded
                # command instead of resetting it to measured slack geometry.
                q_next=self.h['shortening']+self.policy.speeds[:4]*self.policy.config.control_period_s*self.previous[:4]
                self.desired=np.r_[differential_lengths(q_next),self.last['feed_target_m']]
            q,feed=slew_targets(self.h['shortening'],self.h['feed_m'],self.desired[:4],self.desired[4],self.h['config'])
            submit_targets(self.h,q,feed)
        except Exception as e:self.finish('controller_exception',repr(e))
    def onAnimateEndEvent(self,event):
        if self.done:return
        self.ticks+=1;c=self.h['config'];t=self.ticks*c.dt
        p=np.asarray(self.h['state'].position.value)
        forces=np.array([float(x.force.value)/c.dt for x in self.h['cables']])
        if not np.isfinite(p).all() or not np.isfinite(forces).all():self.finish('nonfinite_physics');return
        self.record()
        if self.safety:
            self.contact=self.safety.check(np.asarray(self.h['collision'].position.value),self.h['collision'].constraint.value,
                                          self.h['root'].getObject('constraints').constraintForces.value)
            self.peak_contact=max(self.peak_contact,self.contact['contact_force_n'])
            self.peak_penetration=max(self.peak_penetration,self.contact['penetration_m'])
            if self.contact['invalid_reason']:self.finish(self.contact['invalid_reason']);return
        stretch=np.max(np.abs(np.linalg.norm(np.diff(p[:,:3],axis=0),axis=1)/np.diff(self.h['definition']['abscissa'])-1))
        self.peak_stretch=max(self.peak_stretch,float(stretch));self.peak_force=max(self.peak_force,float(np.abs(forces).max()))
        self.max_move=max(self.max_move,float(np.linalg.norm(p[-1,:3]-self.h['definition']['poses'][-1,:3])))
        if stretch>.1 or self.peak_force>c.cable_force_n+.001:self.finish('physical_limit_violation');return
        if self.ticks%self.interval==0:
            goal,phase=self.reference.target(t,p[-1,:3]);error=float(np.linalg.norm(goal-p[-1,:3])*1000)
            row=dict(t=t,phase=phase,error_mm=error,tip_m=p[-1,:3].tolist(),goal_m=goal.tolist(),
                     force_n=forces.tolist(),feed_m=self.h['feed_m'],q_m=self.h['shortening'].tolist(),
                     controller_us=self.last.get('controller_us'),waypoint_index=self.reference.current_index if isinstance(self.reference,RouteReference) else None,waypoint_completed=False,completed_waypoints=getattr(self.reference,'completed_count',None),true_exit_distance_mm=float(np.linalg.norm(p[-1,:3]-self.h['geometry']['exit_position_m'])*1000) if self.h['geometry'] else None,progress_m=getattr(self.reference,'progress_m',None),**self.contact)
            arrived=False
            if isinstance(self.reference,RouteReference) and not self.zero:
                arrived=self.reference.observe(p[-1,:3])
                row.update(waypoint_completed=arrived,completed_waypoints=self.reference.completed_count,progress_m=self.reference.progress_m)
            self.errors.append((phase,error));self.trace.write(json.dumps(row)+'\n')
            if self.view:
                saturated=np.any(np.abs(self.h['shortening'])>=c.cable_limit_m-1e-6)
                status=phase+' error %.2fmm | feed %.1fmm | q %smm %s'%(error,self.h['feed_m']*1000,
                    np.round(self.h['shortening']*1000,2),'CABLE LIMIT' if saturated else '')
                self.view.update(p,t,goal,status)
                Sofa.Simulation.updateVisual(self.h['root'])
            if isinstance(self.reference,RouteReference) and not self.zero:
                if arrived:
                    self.error_history=[]
                else:
                    self.error_history.append((t,error))
                    self.error_history=[x for x in self.error_history if x[0]>=t-5.02]
                    if (t>6. and self.error_history[-1][0]-self.error_history[0][0]>=5.
                        and self.error_history[0][1]-min(x[1] for x in self.error_history)<1. and error>5.
                        and np.any(np.abs(self.h['shortening'])>=c.cable_limit_m-1e-6)):
                        self.finish('stalled_at_cable_limit');return
        if self.ticks%self.status_interval==0:self.update_status()
        if isinstance(self.reference,RouteReference) and not self.zero and self.reference.reached(p[-1,:3]):
            at_exit=self.h['geometry'] is not None and np.linalg.norm(p[-1,:3]-self.h['geometry']['exit_position_m'])<=.005
            self.finish('station_reached' if at_exit else 'trajectory_completed')
        elif self.duration is not None and t>=self.duration-1e-9:self.finish('zero_input_completed' if self.zero else 'duration_completed')
    def update_status(self):
        status=dict(steps=self.ticks,simulated_s=self.ticks*self.h['config'].dt,wall_s=time.monotonic()-self.started,
                    feed_mm=self.h['feed_m']*1000,tip_displacement_mm=self.max_move*1000,done=self.done)
        temp=self.output/'progress.tmp';temp.write_text(json.dumps(status),encoding='utf-8');temp.replace(self.output/'progress.json')
        self.emit('[LIVE] '+json.dumps(status))
    def finish(self,reason,detail=None):
        if self.done:return
        self.done=True;self.h['root'].animate.value=False
        self.recorder.close(complete=reason in ['duration_completed','station_reached','trajectory_completed','zero_input_completed'],reason=reason)
        self.trace.close()
        phases={}
        for phase in sorted(set(p for p,e in self.errors)):
            a=np.array([e for p,e in self.errors if p==phase]);phases[phase]=dict(rmse_mm=float(np.sqrt(np.mean(a*a))),final_error_mm=float(a[-1]),max_error_mm=float(a.max()))
        self.summary=dict(reason=reason,detail=detail,physics_steps=self.ticks,simulated_s=self.ticks*self.h['config'].dt,
            wall_s=time.monotonic()-self.started,maximum_tip_displacement_mm=self.max_move*1000,
            peak_force_n=self.peak_force,maximum_relative_edge_stretch=self.peak_stretch,phases=phases,
            peak_contact_force_n=self.peak_contact,peak_wall_penetration_m=self.peak_penetration,
            progress_m=getattr(self.reference,'progress_m',None),recorded_frames=self.recorder.manifest['total_frames'],
            station_target_m=getattr(self.reference,'maximum_station',None),
            trajectory_completed=getattr(self.reference,'complete',False),completed_waypoints=getattr(self.reference,'completed_count',None),
            station_reached=reason=='station_reached',
            initial_state_sha256=self.recorder.manifest['initial_state_sha256'],
            scope='Physical simulation result, not calibrated hardware accuracy or full-route success.')
        (self.output/'summary.json').write_text(json.dumps(self.summary,indent=2),encoding='utf-8')
        self.update_status();self.emit('[RESULT] '+json.dumps(self.summary));self.emit.close()
        if self.view:self.view.update(np.asarray(self.h['state'].position.value),self.ticks*self.h['config'].dt,status='STOPPED '+reason)
    def interrupted(self):
        if not self.done:self.finish('interrupted')
    def onKeypressedEvent(self,event):
        if self.view and event.get('key') in ['1','2']:
            self.view.set_camera(np.asarray(self.h['state'].position.value)[-1,:3],event['key']=='2')

