import unittest

from escape_robot_perception.depth_geometry import project_pixel


class DepthGeometryTest(unittest.TestCase):
    def test_principal_point_has_zero_lateral_offset(self):
        self.assertEqual(project_pixel(320, 240, 2.0, 600, 600, 320, 240), (0.0, 0.0, 2.0))

    def test_optical_axes_and_metric_units(self):
        self.assertEqual(project_pixel(380, 180, 2.0, 600, 600, 320, 240), (0.2, -0.2, 2.0))

    def test_nonpositive_depth_is_rejected(self):
        for depth in (0.0, -1.0):
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                project_pixel(320, 240, depth, 600, 600, 320, 240)

    def test_invalid_focal_lengths_are_rejected(self):
        for fx, fy in ((0.0, 600), (600, 0.0), (-1.0, 600), (600, -1.0)):
            with self.subTest(fx=fx, fy=fy), self.assertRaises(ValueError):
                project_pixel(320, 240, 2.0, fx, fy, 320, 240)

    def test_nonfinite_inputs_are_rejected(self):
        for index in range(7):
            for value in (float("nan"), float("inf"), float("-inf")):
                values = [320, 240, 2.0, 600, 600, 320, 240]
                values[index] = value
                with self.subTest(index=index, value=value), self.assertRaises(ValueError):
                    project_pixel(*values)


if __name__ == "__main__":
    unittest.main()
