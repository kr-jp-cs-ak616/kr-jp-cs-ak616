"""SDF 치수 계산과 차동구동 치수 추정 검사."""
import math
import unittest
from wheel_geometry_audit import audit_model, fit_geometry, interpolate_odom, assess_estimate


SDF = '''<sdf version="1.5"><model name="test">
<link name="left"><collision name="tire"><pose>0 .144 .023 0 0 0</pose>
<geometry><cylinder><radius>.033</radius></cylinder></geometry></collision></link>
<link name="right"><collision name="tire"><pose>0 -.144 .023 0 0 0</pose>
<geometry><cylinder><radius>.033</radius></cylinder></geometry></collision></link>
<joint name="L"><child>left</child></joint><joint name="R"><child>right</child></joint>
<plugin filename="libgazebo_ros_diff_drive.so"><left_joint>L</left_joint><right_joint>R</right_joint>
<wheel_separation>.287</wheel_separation><wheel_diameter>.066</wheel_diameter></plugin>
</model></sdf>'''


class GeometryTests(unittest.TestCase):
    def test_sdf_geometry_and_world_default(self):
        result = audit_model(SDF)
        self.assertAlmostEqual(result['geometry_separation_m'], .288)
        self.assertAlmostEqual(result['configured_separation_m'], .287)
        self.assertEqual(result['odometry_source_label'], 'WORLD')
        self.assertFalse(result['source_explicit'])

    def test_explicit_encoder(self):
        source = SDF.replace('</plugin>', '<odometry_source>0</odometry_source></plugin>')
        self.assertEqual(audit_model(source)['odometry_source_label'], 'ENCODER')

    def test_relative_frames_rejected(self):
        with self.assertRaises(ValueError):
            audit_model(SDF.replace('<pose>', '<pose relative_to="other">', 1))

    def test_negative_dimension_rejected(self):
        with self.assertRaises(ValueError):
            audit_model(SDF.replace('<radius>.033', '<radius>-.033'))

    def test_fit_distinguishes_radii_and_separation(self):
        rl, rr, width = .0328, .0332, .289
        segments = []
        for dl, dr in [(1, 1), (-1, 1), (1, -1), (.3, 1.2), (1.2, .3)]*10:
            segments.append((dl, dr, (rl*dl+rr*dr)/2, (rr*dr-rl*dl)/width))
        result = fit_geometry(segments)
        self.assertAlmostEqual(result['left_radius_m'], rl, places=8)
        self.assertAlmostEqual(result['right_radius_m'], rr, places=8)
        self.assertAlmostEqual(result['effective_separation_m'], width, places=8)
        self.assertLess(result['yaw_rmse_deg'], 1e-8)
        self.assertFalse(result['applied'])

    def test_single_curvature_rejected(self):
        with self.assertRaises(ValueError):
            fit_geometry([(1, 2, .0495, .033/.288)]*30)

    def test_stationary_data_rejected(self):
        with self.assertRaises(ValueError):
            fit_geometry([(0, 0, 0, 0)]*20)

    def test_reference_interpolation_wraps_yaw(self):
        samples = [(1, 0, 0, math.radians(179)), (1.1, .01, 0, math.radians(-179))]
        self.assertAlmostEqual(abs(interpolate_odom(samples, 1.05)[2]), math.pi)
        self.assertIsNone(interpolate_odom(samples, 2))

    def test_reference_gap_rejected(self):
        self.assertIsNone(interpolate_odom([(1, 0, 0, 0), (2, 0, 0, 0)], 1.5))

    def test_user_contact_measurement_is_flagged(self):
        estimate = {'left_radius_m': .030126839, 'right_radius_m': .031132987,
                    'effective_separation_m': .20591389, 'distance_rmse_m': .0094904,
                    'yaw_rmse_deg': 4.28784}
        segments = [(1, 1, .033, 0)]*10+[(-1, 1, 0, .23)]*10+[(1, -1, 0, -.23)]*10
        quality = assess_estimate(estimate, audit_model(SDF), segments)
        self.assertFalse(quality['screening_pass'])
        self.assertTrue(any('잔차' in text for text in quality['reasons']))
        self.assertTrue(any('축간거리' in text for text in quality['reasons']))

    def test_exact_independent_contact_free_data_passes_screen(self):
        segments = [(1, 1, .033, 0)]*20+[(-1, 1, 0, .066/.288)]*10+[(1, -1, 0, -.066/.288)]*10
        estimate = fit_geometry(segments)
        self.assertTrue(assess_estimate(estimate, audit_model(SDF), segments)['screening_pass'])

    def test_missing_turn_direction_is_flagged(self):
        segments = [(1, 1, .033, 0)]*20+[(-1, 1, 0, .066/.288)]*10
        estimate = fit_geometry(segments)
        quality = assess_estimate(estimate, audit_model(SDF), segments)
        self.assertFalse(quality['screening_pass'])


if __name__ == '__main__':
    unittest.main()
