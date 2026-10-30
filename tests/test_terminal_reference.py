import unittest
import numpy as np
from tendon_sim.references import RouteReference

class TerminalReference(unittest.TestCase):
 def test_waypoint_offset_does_not_move_exit_or_acceptance(self):
  route=np.array([[0.,0.,0.],[1.,0.,0.]])
  ref=RouteReference(route,lookahead=.025,terminal_goal_offset_m=[0.,0.,-.01],terminal_zone_m=.2)
  goal,_=ref.target(0.,[.5,0.,0.]);np.testing.assert_allclose(goal,[.525,0.,0.])
  ref.seed_from_tip([1.,0.,0.]);goal,_=ref.target(0.,[1.,0.,0.])
  np.testing.assert_allclose(goal,[1.,0.,-.01]);np.testing.assert_allclose(ref.point(1.),route[-1])
  self.assertTrue(ref.reached([1.,0.,0.]));self.assertFalse(ref.reached(goal))
 def test_default_reference_unchanged(self):
  r=RouteReference([[0,0,0],[1,0,0]])
  np.testing.assert_allclose(r.target(0,[.5,0,0])[0],[.508,0,0])
 def test_offset_bound(self):
  with self.assertRaises(ValueError):RouteReference([[0,0,0],[1,0,0]],terminal_goal_offset_m=[0,0,.021])
