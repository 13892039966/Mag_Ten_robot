import math
import numpy as np


class FarReference:
    def __init__(self,origin):
        self.origin=np.asarray(origin,float)
        self.stages=[('far_y_positive',14.,[.030,.008,0.]),('far_y_negative',16.,[.030,-.008,0.]),
                     ('far_z_positive',14.,[.015,0.,.008]),('far_z_negative',14.,[.030,0.,-.008]),
                     ('circle_prepare',10.,[.020,.005,0.])]
        self.circle_start_s=1.+sum(s[1] for s in self.stages)
        self.circle_period_s=30.
        self.duration_s=self.circle_start_s+self.circle_period_s+14.

    def target(self,t,tip=None):
        if not math.isfinite(t) or t<0:raise ValueError('Invalid time')
        if t<1.:return self.origin.copy(),'rest'
        start=1.
        for name,duration,offset in self.stages:
            if t<start+duration:return self.origin+offset,name
            start+=duration
        if t<self.circle_start_s+self.circle_period_s:
            a=2*np.pi*(t-self.circle_start_s)/self.circle_period_s
            return self.origin+[.020,.005*np.cos(a),.005*np.sin(a)],'far_circle'
        return self.origin.copy(),'return_to_start'

    def preview(self):
        a=np.linspace(0,2*np.pi,100)
        circle=self.origin+[.020,0.,0.]+np.column_stack([np.zeros_like(a),.005*np.cos(a),.005*np.sin(a)])
        return np.vstack([self.origin]+[self.origin+s[2] for s in self.stages]+[circle])


class RouteReference:
    def __init__(self,route,maximum_station=None,lookahead=.008,terminal_goal_offset_m=None,terminal_zone_m=.15):
        self.route=np.asarray(route,float)
        if self.route.ndim!=2 or self.route.shape[1]!=3 or len(self.route)<2 or not np.isfinite(self.route).all():
            raise ValueError('Finite Nx3 route required')
        self.delta=np.diff(self.route,axis=0);self.segment=np.linalg.norm(self.delta,axis=1)
        if np.any(self.segment<=0):raise ValueError('No duplicate consecutive route points')
        self.arc=np.r_[0.,np.cumsum(self.segment)];self.length_m=float(self.arc[-1])
        self.maximum_station=self.length_m if maximum_station is None else min(float(maximum_station),self.length_m)
        if not 0<self.maximum_station<=self.length_m or lookahead<=0:raise ValueError('Invalid route range')
        self.lookahead=lookahead;self.progress_m=0.;self.duration_s=300.
        self.terminal_goal_offset_m=np.zeros(3) if terminal_goal_offset_m is None else np.asarray(terminal_goal_offset_m,float)
        self.terminal_zone_m=float(terminal_zone_m)
        if self.terminal_goal_offset_m.shape!=(3,) or not np.isfinite(self.terminal_goal_offset_m).all() or np.linalg.norm(self.terminal_goal_offset_m)>.02:raise ValueError('Terminal control target offset must be finite and <=20mm')
        if not np.isfinite(self.terminal_zone_m) or self.terminal_zone_m<=0:raise ValueError('Invalid terminal zone')

    def point(self,s):
        return np.array([np.interp(s,self.arc,self.route[:,j]) for j in range(3)])

    def seed_from_tip(self,tip):
        delta=np.asarray(tip,float)-self.route[:-1]
        u=np.clip(np.sum(delta*self.delta,axis=1)/self.segment**2,0.,1.)
        k=int(np.argmin(np.linalg.norm(self.route[:-1]+u[:,None]*self.delta-np.asarray(tip,float),axis=1)))
        self.progress_m=min(self.maximum_station,float(self.arc[k]+u[k]*self.segment[k]))

    def target(self,t,tip):
        q=np.asarray(tip,float)
        candidates=np.flatnonzero((self.arc[:-1]<=self.progress_m+.060)&(self.arc[1:]>=max(0.,self.progress_m-.010)))
        a=self.route[candidates];d=self.delta[candidates]
        u=np.clip(np.sum((q-a)*d,axis=1)/self.segment[candidates]**2,0.,1.)
        k=int(np.argmin(np.linalg.norm(a+u[:,None]*d-q,axis=1)))
        station=self.arc[candidates[k]]+u[k]*self.segment[candidates[k]]
        self.progress_m=min(self.maximum_station,max(self.progress_m,float(station)))
        goal=self.point(min(self.maximum_station,self.progress_m+self.lookahead))
        # Control waypoint compensation only: point(), route, and reached() retain
        # the true geometry and unchanged 5mm acceptance criterion.
        weight=np.clip(1.-(self.maximum_station-self.progress_m)/self.terminal_zone_m,0.,1.)
        weight=weight*weight*(3.-2.*weight)
        return goal+weight*self.terminal_goal_offset_m,'colon_route'

    def reached(self,tip):
        return self.progress_m>=self.maximum_station-.005 and np.linalg.norm(tip-self.point(self.maximum_station))<=.005

    def preview(self):
        return np.array([self.point(s) for s in np.linspace(0.,self.maximum_station,max(2,int(self.maximum_station/.005)+1))])
