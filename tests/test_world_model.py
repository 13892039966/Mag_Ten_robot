import unittest
from types import SimpleNamespace
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from tendon_sim.body_geometry import BodyConfig,body_layout
from tendon_sim.cc_predictor import CCFourTendon
from tendon_sim.live_scene import WorldModel
from tendon_sim.actuator_gauge import differential_jacobian

class WorldSensitivity(unittest.TestCase):
 def model(self):
  c=BodyConfig(dt=.02,feed_limit_m=1.4);d=body_layout(c,direction=[0.,1.,0.],insertion=0.)
  h=dict(config=c,definition=d,geometry=None,state=SimpleNamespace(position=SimpleNamespace(value=d['poses'])))
  return c,d,WorldModel(CCFourTendon(c),h)
 def test_default_rotates_original_sensitivity(self):
  c,d,m=self.model();q=np.array([.002,-.001,-.002,.001]);j=CCFourTendon(c).jacobian(q,0.)
  rotation=Rotation.from_quat(d['poses'][-1,3:]).as_matrix();j[:,:4]=rotation@j[:,:4];j[:,4]=rotation[:,0]
  np.testing.assert_allclose(m.jacobian(q,0.),differential_jacobian(j),atol=1e-12)
 def test_distal_chain_rule_and_physical_config_unchanged(self):
  c,d,m=self.model();m.distal_length_m=.1;q=np.array([.002,-.001,-.002,.001]);actual=m.jacobian(q,0.)
  length=m.last_effective_length_m;fraction=length/c.active_length_m;local=CCFourTendon(replace(c,active_length_m=length,cable_limit_m=min(c.cable_limit_m,.49*length)))
  rotation=Rotation.from_quat(d['poses'][-1,3:]).as_matrix();step=1e-7
  numerical=np.column_stack([rotation@(local.tip_position((q+np.eye(4)[i]*step)*fraction)-local.tip_position((q-np.eye(4)[i]*step)*fraction))/(2*step) for i in range(4)])
  expected=np.column_stack([numerical,rotation[:,0]])
  np.testing.assert_allclose(actual,differential_jacobian(expected),atol=1e-7)
  self.assertEqual(c.active_length_m,.2);self.assertEqual(c.cable_force_n,2.);self.assertEqual(c.cable_limit_m,.03)
