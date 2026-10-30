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


def load_trajectory(path):
    """Read original ordered world-coordinate CSV points in metres, without resampling."""
    data = np.genfromtxt(path, delimiter=',', names=True, encoding='utf-8')
    if not data.dtype.names or not {'x_m', 'y_m', 'z_m'} <= set(data.dtype.names):
        raise ValueError('Trajectory CSV requires x_m,y_m,z_m columns in world metres')
    return np.column_stack([np.atleast_1d(data[k]) for k in ('x_m', 'y_m', 'z_m')])


class RouteReference:
    """Hold each external waypoint until the actual tip is within 5mm."""
    arrival_radius_m = .005

    def __init__(self, route):
        self.route = np.array(route, dtype=float, copy=True)
        if self.route.ndim != 2 or self.route.shape[1] != 3 or len(self.route) < 2 or not np.isfinite(self.route).all():
            raise ValueError('Finite Nx3 route with at least two points required')
        self.segment = np.linalg.norm(np.diff(self.route, axis=0), axis=1)
        self.arc = np.r_[0., np.cumsum(self.segment)]
        self.length_m = float(self.arc[-1])
        self.maximum_station = self.length_m
        self.current_index = 0
        self.completed_count = 0
        self.progress_m = 0.
        self.duration_s = 300.

    @property
    def complete(self):
        return self.completed_count == len(self.route)

    def target(self, t, tip=None):
        return self.route[self.current_index].copy(), 'colon_route'

    def observe(self, tip):
        """Called once per completed control period; advance at most one point."""
        tip = np.asarray(tip, dtype=float)
        if tip.shape != (3,) or not np.isfinite(tip).all():
            raise ValueError('Finite tip xyz required')
        if self.complete or np.linalg.norm(tip - self.route[self.current_index]) > self.arrival_radius_m:
            return False
        self.progress_m = float(self.arc[self.current_index])
        self.completed_count += 1
        if not self.complete:
            self.current_index += 1
        return True

    def reached(self, tip):
        return self.complete and np.linalg.norm(np.asarray(tip) - self.route[-1]) <= self.arrival_radius_m

    def preview(self):
        return self.route.copy()
