import unittest
from prepare_house_slam_ab import local_only_lua


class ComparisonConfigTests(unittest.TestCase):
    def test_preserves_sensor_options(self):
        source = 'include "map_builder.lua"\nTRAJECTORY_BUILDER_2D.use_imu_data = true\nreturn options\n'
        modified = local_only_lua(source)
        addition = '\n-- 원인 분리 시험: 지도 전체 최적화만 중지합니다.\nPOSE_GRAPH.optimize_every_n_nodes = 0\n'
        self.assertEqual(modified.replace(addition, '', 1), source)

    def test_override_is_before_return(self):
        modified = local_only_lua('POSE_GRAPH.optimize_every_n_nodes = 90\nreturn options\n')
        self.assertLess(modified.index('optimize_every_n_nodes = 0'), modified.index('return options'))
        self.assertGreater(modified.index('optimize_every_n_nodes = 0'), modified.index('optimize_every_n_nodes = 90'))

    def test_missing_return_is_rejected(self):
        with self.assertRaises(ValueError):
            local_only_lua('options = {}\n')

    def test_multiple_returns_are_rejected(self):
        with self.assertRaises(ValueError):
            local_only_lua('return options\nreturn options\n')


if __name__ == '__main__':
    unittest.main()
