"""새 바퀴 프로필이 형상과 일치하고 원본 물리를 보존하는지 검사한다."""
import math
import unittest
import xml.etree.ElementTree as ET
from prepare_house_wheels_010 import wheel_profile
from prepare_house_nav2_frontier import configure
from wheel_geometry_audit import audit_model
from test_wheel_geometry import SDF
from test_house_nav2_frontier import config


MODEL = SDF.replace('name="test"', 'name="turtlebot3_waffle_pi"').replace(
    '<link name="left">', '''<link name="base_link"><collision name="base_collision">
<pose>-.064 0 .048 0 0 0</pose><geometry><box><size>.265 .265 .089</size></box></geometry>
</collision></link><link name="left">''').replace(
    '</plugin>', '<max_wheel_torque>20</max_wheel_torque><publish_odom_tf>true</publish_odom_tf></plugin>')


class WheelProfileTests(unittest.TestCase):
    def test_plugin_matches_measured_geometry(self):
        text, report = wheel_profile(MODEL)
        audit = audit_model(text)
        self.assertAlmostEqual(audit['configured_separation_m'], audit['geometry_separation_m'])
        self.assertEqual(audit['configured_diameter_m'], .066)
        self.assertEqual(audit['odometry_source_label'], 'WORLD')
        self.assertTrue(audit['source_explicit'])
        self.assertEqual(report['speed_m_s'], .10)

    def test_torque_tf_and_body_are_preserved(self):
        text, _ = wheel_profile(MODEL)
        root = ET.fromstring(text)
        self.assertEqual(root.findtext('model/plugin/max_wheel_torque'), '20')
        self.assertEqual(root.findtext('model/plugin/publish_odom_tf'), 'true')
        self.assertEqual(root.findtext('model/link/collision/geometry/box/size'), '.265 .265 .089')
        self.assertEqual(audit_model(MODEL)['configured_separation_m'], .287)

    def test_nonstandard_geometry_and_odom_are_rejected(self):
        for text in (MODEL.replace('.144', '.150'), MODEL.replace('.033', '.035'),
                     MODEL.replace('.265 .265', '.400 .400'),
                     MODEL.replace('</plugin>', '<odometry_source>0</odometry_source></plugin>')):
            with self.assertRaises(ValueError):
                wheel_profile(text)

    def test_wheel_accel_exceeds_controller_combined_command_bound(self):
        _, report = wheel_profile(MODEL)
        c = configure(config(), waffle_footprint=True)
        smoother = c['velocity_smoother']['ros__parameters']
        required = smoother['max_accel'][0]+.288*smoother['max_accel'][2]/2
        self.assertAlmostEqual(required, report['commanded_wheel_acceleration_bound_m_s2'])
        self.assertGreater(report['changes']['max_wheel_acceleration'], required)
        self.assertAlmostEqual(report['outer_wheel_rad_s'], (.1+.288*.2/2)/.033)
        self.assertAlmostEqual(report['straight_wheel_rad_s']*.033, .1)

    def test_both_wheels_reconstruct_commanded_chassis_motion(self):
        _, report = wheel_profile(MODEL)
        for v, yaw in ((.1, 0), (0, .2), (.1, -.2)):
            left, right = (v-yaw*.288/2)/.033, (v+yaw*.288/2)/.033
            self.assertAlmostEqual(.033*(left+right)/2, v)
            self.assertAlmostEqual(.033*(right-left)/.288, yaw)

    def test_mapping_yaw_limit_is_validated(self):
        for value in (0, -.1, .31, math.nan):
            with self.assertRaises(ValueError):
                configure(config(), yaw_rate=value)


if __name__ == '__main__':
    unittest.main()
