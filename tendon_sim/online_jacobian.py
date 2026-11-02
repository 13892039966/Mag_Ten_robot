"""Conservative command-to-tip secant correction; not a contact mechanics model.

Nonoverlapping 0.4 s windows at the 20 ms control rate. Accumulate the
changing geometric prediction, then correct its residual with a regularized
rank-one update. Correlated inputs identify only their excited direction.
"""
import numpy as np
from .actuator_gauge import differential_jacobian

class WindowedCorrection:
    def __init__(self, window=20, gain=.15, regularization_m=5e-5, max_relative_norm=.5):
        self.window=window; self.gain=gain; self.regularization_m=regularization_m
        self.max_relative_norm=max_relative_norm
        self.residual=np.zeros((3,5)); self.previous=None
        self.dx=np.zeros(5); self.dy=np.zeros(3); self.predicted=np.zeros(3)
        self.count=0; self.updates=0; self.skipped=0; self.relative_norm=0.
    def observe(self, command, tip, base):
        command=np.asarray(command,float);tip=np.asarray(tip,float);base=np.asarray(base,float)
        if self.previous is not None:
            old_command,old_tip,old_base=self.previous
            dx=command-old_command
            self.dx+=dx;self.dy+=tip-old_tip;self.predicted+=old_base @ dx
            self.count+=1
            if self.count>=self.window:
                n=float(self.dx @ self.dx)
                # Reject unexcited windows and abrupt state/contact transients.
                if n>self.regularization_m**2 and np.linalg.norm(self.dy)<.005:
                    error=self.dy-self.predicted-self.residual @ self.dx
                    self.residual+=self.gain*np.outer(error,self.dx)/(n+self.regularization_m**2)
                    self.residual=differential_jacobian(self.residual)
                    bound=self.max_relative_norm*np.linalg.norm(base)
                    size=np.linalg.norm(self.residual)
                    if size>bound:self.residual*=bound/size
                    self.updates+=1
                else:self.skipped+=1
                self.dx[:]=0;self.dy[:]=0;self.predicted[:]=0;self.count=0
        self.previous=(command.copy(),tip.copy(),base.copy())
        self.relative_norm=float(np.linalg.norm(self.residual)/max(np.linalg.norm(base),1e-12))
    def correct(self, base):
        return np.asarray(base,float)+self.residual
    def report(self):
        return dict(updates=self.updates,skipped=self.skipped,relative_norm=self.relative_norm,
                    window_s=.4,gain=self.gain,regularization_m=self.regularization_m,
                    max_relative_norm=self.max_relative_norm)
