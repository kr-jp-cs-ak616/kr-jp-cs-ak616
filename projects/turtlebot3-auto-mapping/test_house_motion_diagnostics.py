"""망설임 관측 분류와 공유 가능한 진단 JSON의 순수 로직 검사."""
import json
import math
import unittest
from tools.diagnostics.diagnose_house_motion import classify_motion, compact_state, clean_numbers


def sample(**updates):
    value = dict(clock_age=.1, scan_age=.1, command_age=.1, odom_age=.1,
                 command=[0., 0.], actual=[0., 0.], explorer={'operation': 'nav_result'})
    value.update(updates)
    return value


class MotionDiagnosticsTests(unittest.TestCase):
    def test_clock_stall_is_not_called_collision(self):
        self.assertEqual(classify_motion(sample(clock_age=3))[0], 'clock_wait')

    def test_scan_receive_and_stamp_errors_are_distinct(self):
        self.assertEqual(classify_motion(sample(scan_age=1))[0], 'scan_wait')
        self.assertEqual(classify_motion(sample(scan_stamp_age=-1))[0], 'scan_time_mismatch')

    def test_input_pause_and_planner_wait_are_distinct(self):
        for state, expected in (({'wait_reason': 'TF 대기'}, 'input_wait'),
                                ({'paused': True}, 'paused'), ({'operation': 'plan_result'}, 'planning')):
            self.assertEqual(classify_motion(sample(explorer=state))[0], expected)

    def test_rotation_and_long_rotation_are_distinct(self):
        self.assertEqual(classify_motion(sample(command=[0., .1]))[0], 'rotating')
        self.assertEqual(classify_motion(sample(command=[0., .1], translation_10s=.01))[0], 'long_rotation')

    def test_movement_and_unresponsive_motion_are_distinct(self):
        self.assertEqual(classify_motion(sample(command=[.1, 0.], actual=[.09, 0.]))[0], 'moving')
        self.assertEqual(classify_motion(sample(command=[.1, 0.]))[0], 'little_motion')
        self.assertEqual(classify_motion(sample(command=[.1, 0.], odom_age=2))[0], 'odom_wait')

    def test_no_command_and_zero_command_and_no_goal_are_distinct(self):
        self.assertEqual(classify_motion(sample(command_age=3))[0], 'command_missing')
        self.assertEqual(classify_motion(sample())[0], 'zero_command')
        self.assertEqual(classify_motion(sample(explorer={}))[0], 'goal_wait')

    def test_samples_do_not_duplicate_full_map_or_all_goals(self):
        state = dict(map={'data': [1, 2]}, report={'state': 'ready', 'goals': [{}, {}]})
        small = compact_state(state)
        self.assertNotIn('map', small)
        self.assertEqual(small['report']['candidate_count'], 2)

    def test_missing_topics_produce_standard_json_without_infinity(self):
        result = clean_numbers({'age': math.inf, 'values': [math.nan, 1.]})
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), {'age': None, 'values': [None, 1.]})


if __name__ == '__main__':
    unittest.main()
