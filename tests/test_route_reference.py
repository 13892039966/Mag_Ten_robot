import unittest

import numpy as np

from tendon_sim.references import RouteReference


class RouteReferenceTests(unittest.TestCase):
    def test_same_lookahead_rule_near_exit(self):
        ref = RouteReference([[0, 0, 0], [1, 0, 0]], lookahead=.025)
        for station in [.5, .85, .98, 1.]:
            ref.seed_from_tip([station, 0, 0])
            goal, _ = ref.target(0, [station, 0, 0])
            np.testing.assert_allclose(goal, [min(1., station + .025), 0, 0])

    def test_true_exit_acceptance(self):
        ref = RouteReference([[0, 0, 0], [1, 0, 0]])
        ref.seed_from_tip([1., 0, 0])
        self.assertTrue(ref.reached([1., 0, 0]))
        self.assertFalse(ref.reached([1., 0, .006]))
