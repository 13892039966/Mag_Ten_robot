import unittest
import numpy as np
from tendon_sim.terminal_heading import heading_waypoint
from tendon_sim.references import RouteReference

class HeadingFeedback(unittest.TestCase):
 def test_aligned_heading_keeps_true_goal(self):
  goal=np.array([1.,2.,3.]);np.testing.assert_array_equal(heading_waypoint(goal,[0,2,0],[0,1,0],1.,.06,.03),goal)
 def test_correction_points_against_heading_error_and_is_bounded(self):
  goal=np.zeros(3);tip=np.array([0.,1.,.2]);exit_=np.array([0.,1.,-.2])
  corrected=heading_waypoint(goal,tip,exit_,1.,.15,.025)
  self.assertLess(corrected[2],0);self.assertAlmostEqual(float(np.linalg.norm(corrected)),.025)
  np.testing.assert_array_equal(heading_waypoint(goal,tip,exit_,0.,.15,.025),goal)
 def test_virtual_goal_cannot_change_reference_acceptance(self):
  ref=RouteReference([[0,0,0],[1,0,0]]);ref.seed_from_tip([1,0,0]);true=ref.point(1.)
  shifted=heading_waypoint(true,[1,0,0],[1,0,-1],1.,.06,.03)
  self.assertTrue(ref.reached(true));self.assertFalse(ref.reached(shifted));np.testing.assert_array_equal(ref.point(1.),true)
