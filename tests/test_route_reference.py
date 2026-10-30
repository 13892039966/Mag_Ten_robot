import tempfile
import unittest
from pathlib import Path

import numpy as np

from tendon_sim.references import RouteReference, load_trajectory


class RouteReferenceTests(unittest.TestCase):
    def test_fixed_target_never_advances_by_projection_or_time(self):
        ref = RouteReference([[0, 0, 0], [.01, 0, 0], [.02, 0, 0]])
        for t in [0, 1, 100]:
            np.testing.assert_array_equal(ref.target(t, [.02, 0, 0])[0], [0, 0, 0])
        self.assertFalse(ref.observe([.02, 0, 0]))
        self.assertEqual(ref.current_index, 0)

    def test_5mm_boundary_and_one_point_per_observation(self):
        ref = RouteReference([[0, 0, 0], [.001, 0, 0], [.002, 0, 0]])
        self.assertFalse(ref.observe([0, .005001, 0]))
        self.assertTrue(ref.observe([0, .005, 0]))
        self.assertEqual(ref.completed_count, 1)
        self.assertEqual(ref.current_index, 1)
        # Reading or recording the target cannot complete more points.
        for _ in range(3):
            ref.target(0, [0, 0, 0])
        self.assertEqual(ref.completed_count, 1)
        self.assertTrue(ref.observe([0, 0, 0]))
        self.assertFalse(ref.complete)
        self.assertTrue(ref.observe([0, 0, 0]))
        self.assertTrue(ref.reached([0, 0, 0]))
        self.assertFalse(ref.observe([0, 0, 0]))
        self.assertFalse(ref.reached([0, .01, 0]))

    def test_original_order_and_coordinates_preserved(self):
        points = np.array([[0, 0, 0], [.1, 0, 0], [0, 0, 0], [0, 0, 0]])
        ref = RouteReference(points)
        np.testing.assert_array_equal(ref.preview(), points)
        ref.observe(points[0])
        self.assertFalse(ref.observe(points[-1]))
        self.assertEqual(ref.current_index, 1)
        ref.observe(points[1])
        ref.observe(points[2])
        self.assertFalse(ref.complete)
        ref.observe(points[3])
        self.assertTrue(ref.complete)

    def test_invalid_points_rejected(self):
        for points in [[], [[0, 0, 0]], [[0, 0], [1, 0]], [[0, 0, 0], [float('nan'), 0, 0]]]:
            with self.assertRaises(ValueError):
                RouteReference(points)

    def test_external_csv_no_resampling(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.csv'
            path.write_text('x_m,y_m,z_m\n0,0,0\n.003,0,0\n.018,.002,0\n')
            points = load_trajectory(path)
            np.testing.assert_array_equal(points, [[0, 0, 0], [.003, 0, 0], [.018, .002, 0]])
            np.testing.assert_array_equal(RouteReference(points).preview(), points)
            path.write_text('x,y,z\n0,0,0\n1,0,0\n')
            with self.assertRaises(ValueError):
                load_trajectory(path)
