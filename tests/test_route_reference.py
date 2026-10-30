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

    def test_3mm_boundary_and_one_point_per_observation(self):
        ref = RouteReference([[0, 0, 0], [.001, 0, 0], [.002, 0, 0]], arrival_radius_m=.003)
        self.assertFalse(ref.observe([0, .003001, 0]))
        self.assertTrue(ref.observe([0, .003, 0]))
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

    def test_default_sampled_trajectory_spacing_and_endpoints(self):
        root = Path(__file__).resolve().parents[1]
        original = load_trajectory(root / 'assets/original/centerline_world_m.csv')
        path = root / 'assets/trajectories/centerline_10mm.csv'
        sampled = load_trajectory(path)
        data = np.genfromtxt(path, delimiter=',', names=True)
        np.testing.assert_allclose(np.diff(data['s_m'])[:-1], .01, rtol=0, atol=1e-12)
        self.assertGreater(np.diff(data['s_m'])[-1], 0)
        self.assertLessEqual(np.diff(data['s_m'])[-1], .01)
        np.testing.assert_array_equal(sampled[[0, -1]], original[[0, -1]])
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(original, axis=0), axis=1))]
        expected = np.column_stack([np.interp(data['s_m'], arc, original[:, i]) for i in range(3)])
        np.testing.assert_allclose(sampled, expected, atol=1e-14)
        self.assertEqual(len(sampled), 135)
        self.assertEqual(RouteReference.arrival_radius_m, .005)

    def test_configurable_arrival_radius(self):
        points = [[0, 0, 0], [.01, 0, 0]]
        strict = RouteReference(points, arrival_radius_m=.003)
        relaxed = RouteReference(points)
        self.assertFalse(strict.observe([0, .004, 0]))
        self.assertTrue(relaxed.observe([0, .005, 0]))
        for value in [0, -.001, float('inf'), float('nan')]:
            with self.assertRaises(ValueError):
                RouteReference(points, arrival_radius_m=value)
