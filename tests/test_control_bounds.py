import unittest
import numpy as np
from tendon_sim.closed_loop import BoundedDLS
from tendon_sim.body_geometry import BodyConfig
from tendon_sim.online_jacobian import WindowedCorrection

class CommandBounds(unittest.TestCase):
    def test_saturated_command_with_slack_measurement(self):
        c=BodyConfig(dt=.02,feed_limit_m=1.4);policy=BoundedDLS(robot_config=c)
        submitted=np.array([.03,0.,-.03,0.,0.])
        result=policy.command(np.zeros(4),0.,np.zeros(3),np.zeros(3),np.array([0.,.1,0.]),np.zeros(5),submitted_targets=submitted,feed_min=0.)
        action=result['action'];next_command=submitted+policy.speeds*.02*action
        self.assertLessEqual(action[0],1e-10);self.assertGreaterEqual(action[2],-1e-10)
        self.assertAlmostEqual(float(np.sum(action[:4])),0.,places=8)
        self.assertTrue(np.all(np.abs(next_command[:4])<=.03+1e-10));self.assertGreaterEqual(next_command[4],0.)
    def test_equality_solver_rejects_common_mode_preference(self):
        policy=BoundedDLS(robot_config=BodyConfig(dt=.02,feed_limit_m=1.4))
        result=policy.command(np.zeros(4),.1,np.zeros(3),np.zeros(3),np.zeros(3),np.array([1.,1.,1.,1.,0.]),submitted_targets=np.array([0.,0.,0.,0.,.1]),feed_min=0.)
        np.testing.assert_allclose(result['action'],np.zeros(5),atol=1e-8)

    def test_secant_correction_reduces_synthetic_model_error(self):
        base=np.zeros((3,5));base[:,:3]=np.eye(3)
        # Feed-only excitation avoids confusing differential projection with truth.
        base[:,4]=[1.,0.,0.];true=base.copy();true[:,4]=[.7,.1,0.]
        estimator=WindowedCorrection();command=np.zeros(5)
        before=np.linalg.norm(true[:,4]-base[:,4])
        for i in range(401):
            command[4]=i*1e-5;estimator.observe(command,true@command,base)
        self.assertGreater(estimator.updates,0)
        self.assertLess(np.linalg.norm(true[:,4]-estimator.correct(base)[:,4]),before*.2)
        self.assertLessEqual(estimator.relative_norm,.5+1e-12)
    def test_no_excitation_does_not_change_model(self):
        estimator=WindowedCorrection();base=np.ones((3,5))
        for _ in range(50):estimator.observe(np.zeros(5),np.zeros(3),base)
        np.testing.assert_array_equal(estimator.correct(base),base)

class FiniteRecovery(unittest.TestCase):
    def test_finite_schedule_and_retraction_limits(self):
        from tendon_sim.stall_recovery import StallRecovery
        r=StallRecovery();submitted=np.array([.03,-.03,0.,0.,.7]);speeds=np.array([.001]*4+[.005])
        self.assertTrue(r.trigger(10,78))
        hold=r.apply(10,submitted,submitted+speeds*.02,speeds,.02)
        self.assertEqual(hold[4],submitted[4])
        self.assertTrue(r.trigger(30,78))
        retreat=r.apply(30,submitted,submitted,speeds,.02)
        self.assertAlmostEqual(retreat[4],.6999)
        self.assertLess(abs(retreat[0]),abs(submitted[0]));self.assertLess(abs(retreat[1]),abs(submitted[1]))
        self.assertTrue(np.all(abs(retreat-submitted)<=speeds*.02+1e-12))
        self.assertFalse(r.trigger(50,78))
