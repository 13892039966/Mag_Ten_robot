"""Finite, explicitly logged recovery schedule; no waypoint skips or safety bypass."""
import numpy as np

class StallRecovery:
    def __init__(self):
        self.attempts={};self.total=0;self.phase='tracking';self.until=0.;self.events=[]
    def trigger(self, t, waypoint):
        count=self.attempts.get(int(waypoint),0)
        if self.total>=6 or count>=2:return False
        self.total+=1;self.attempts[int(waypoint)]=count+1
        self.phase='hold_feed' if count==0 else 'retract_and_release'
        self.until=float(t)+(2. if count==0 else 1.)
        self.events.append(dict(t=float(t),waypoint_index=int(waypoint),attempt=count+1,phase=self.phase,until_s=self.until))
        return True
    def apply(self,t,submitted,desired,speeds,period):
        submitted=np.asarray(submitted,float);desired=np.array(desired,float,copy=True)
        if self.phase!='tracking' and t>=self.until-1e-9:
            if self.phase=='retract_and_release':
                self.phase='settle';self.until=float(t)+2.
            else:self.phase='tracking'
            self.events.append(dict(t=float(t),phase=self.phase,until_s=self.until))
        if self.phase in ('hold_feed','settle'):
            desired[4]=submitted[4]
        elif self.phase=='retract_and_release':
            fraction=min(1.,float(np.min(speeds[:4]))*period/max(float(np.max(abs(submitted[:4]))),1e-12))
            desired[:4]=submitted[:4]*(1.-fraction)
            desired[4]=max(0.,submitted[4]-speeds[4]*period)
        return desired
