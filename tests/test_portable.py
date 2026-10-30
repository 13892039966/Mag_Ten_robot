import unittest,json,hashlib
from pathlib import Path
import numpy as np
from scipy.sparse import csr_matrix
from tendon_sim.body_geometry import BodyConfig,body_layout
from tendon_sim.closed_loop import slew_targets
from tendon_sim.cc_predictor import CCFourTendon
from tendon_sim.run_limits import control_stride
from tendon_sim.force_adapter import contact_force
B=Path(__file__).resolve().parents[1]
class Portable(unittest.TestCase):
 def test_defaults(self):
  c=BodyConfig();self.assertEqual((c.active_length_m,c.passive_length_m,c.cable_limit_m,c.dt),(.2,1.8,.03,.005));self.assertEqual((c.cable_speed_m_s,c.cable_force_n,c.feed_speed_m_s),(.001,2.,.005));self.assertEqual(control_stride(.02,.01),2);self.assertEqual(control_stride(.02,.005),4);self.assertEqual(body_layout(c)['poses'].shape,(113,7))
 def test_units(self):
  for dt in [.005,.01,.02]:np.testing.assert_allclose(contact_force(csr_matrix([[1.,0,0]]),[2*dt],dt)[0],[2,0,0])
 def test_slew(self):
  c=BodyConfig();q,f=slew_targets(np.zeros(4),0,np.ones(4),1,c);np.testing.assert_allclose(q,.001*c.dt);self.assertAlmostEqual(f,.005*c.dt)
 def test_jacobian(self):
  m=CCFourTendon(BodyConfig());q=np.array([.001,.002,-.001,-.002]);e=1e-7;j=np.column_stack([(m.tip_position(q+np.eye(4)[i]*e)-m.tip_position(q-np.eye(4)[i]*e))/(2*e) for i in range(4)]);np.testing.assert_allclose(m.jacobian(q,0)[:,:4],j,atol=1e-7)
 def test_hashes(self):
  for d in ['assets/original']:
   p=B/d;m=json.loads((p/'manifest.json').read_text());files=m['files'] if 'files'in m else {x['file']:x['sha256'] for x in m['fixtures']}
   for n,h in files.items():self.assertEqual(hashlib.sha256((p/n).read_bytes()).hexdigest(),h)
