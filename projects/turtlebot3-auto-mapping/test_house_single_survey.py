"""단일 목표 직렬 실행과 막힌 면 재관측의 실제 로직을 검사한다."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import MagicMock
import time
from house_nav2_frontier import FrontierMap, single_target


def closed_room():
    w, h = 40, 40
    data = [100 if x in (0, w-1) or y in (0, h-1) else 0
            for y in range(h) for x in range(w)]
    return FrontierMap(data, w, h, .1)


def explorer_method(name, **scope):
    # ROS 없이도 실제 액션 직렬화 메서드를 실행해 취소 중 새 목표가 나가지 않는지 검사한다.
    tree = ast.parse(Path(__file__).with_name('house_nav2_frontier.py').read_text(encoding='utf-8-sig'))
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == 'Explorer')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(time=time, **scope)
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<Explorer method>', 'exec'), namespace)
    return namespace[name]


class SingleSurveyTests(unittest.TestCase):
    def test_interest_is_automatic_unknown_point_with_separate_free_goal(self):
        grid = closed_room()
        for y in range(1, 39):
            for x in range(30, 39):
                i = y*grid.width+x
                grid.data[i], grid.free[i] = -1, 0
        report = grid.select((2., 2., 0.))
        self.assertTrue(report['goals'])
        for goal in report['goals']:
            self.assertEqual(grid.data[grid.cell(goal['interest'])], -1)
            self.assertTrue(grid.free[grid.cell(goal['point'])])

    def test_single_target_keeps_only_one_pending_goal(self):
        self.assertEqual(single_target({'goals': [1, 2, 3]}), [1])
        self.assertEqual(single_target({'goals': []}), [])

    def test_occupied_walls_generate_free_observation_goals(self):
        grid = closed_room()
        report = grid.wall_checks((2., 2., 0.))
        self.assertEqual(report['state'], 'inspection_ready')
        for goal in report['goals']:
            self.assertTrue(grid.body_clear(goal['point']))
            self.assertGreaterEqual(grid.data[grid.cell(goal['inspection_point'])], 65)
            self.assertNotEqual(goal['point'][:2], goal['inspection_point'])

    def test_checked_wall_sections_are_not_repeated_forever(self):
        grid, checked = closed_room(), []
        for _ in range(60):
            report = grid.wall_checks((2., 2., 0.), checked)
            if report['state'] == 'inspection_done':
                break
            self.assertTrue(report['goals'])
            goal = single_target(report)[0]
            point = goal['inspection_point']
            self.assertTrue(all(math.dist(point, p) >= .8 for p in checked))
            checked.append(point)
        self.assertEqual(report['state'], 'inspection_done')

    def test_closed_wall_never_becomes_a_passage_to_other_room(self):
        grid = closed_room()
        for y in range(grid.height):
            i = y*grid.width+20
            grid.data[i], grid.free[i] = 100, 0
        report = grid.wall_checks((1., 2., 0.))
        self.assertTrue(report['goals'])
        self.assertTrue(all(g['point'][0] < 2.0 for g in report['goals']))

    def test_final_wall_checks_are_opt_in_to_survey_phase(self):
        grid = closed_room()
        self.assertEqual(grid.survey((2., 2., 0.))['state'], 'no_frontiers')
        self.assertEqual(grid.survey((2., 2., 0.), inspect=True)['state'], 'inspection_ready')

    def test_unobserved_start_cannot_trigger_wall_motion(self):
        grid = FrontierMap([-1]*400, 20, 20, .1)
        self.assertEqual(grid.wall_checks((1., 1., 0.))['state'], 'inspection_start_unobserved')
        self.assertFalse(grid.survey((1., 1., 0.), inspect=True)['goals'])

    def test_starting_second_action_is_rejected_before_sending(self):
        node = MagicMock()
        node.operation = {'stage': 'nav_result'}
        with self.assertRaises(RuntimeError):
            explorer_method('begin_plan')(node, {'point': (1, 2, 0)})
        node.planner.send_goal_async.assert_not_called()
        node.clear_path.assert_not_called()

    def test_cancel_waits_for_terminal_result_without_dispatching_navigation(self):
        poll = explorer_method('poll_operation')
        handle = NS(accepted=True, get_result_async=lambda: NS(done=lambda: False))
        node = MagicMock()
        node.operation = dict(stage='plan_send', future=NS(done=lambda: True, result=lambda: handle))
        node.cancel_reason, node.paused = '관심 지점 변경', False
        poll(node)
        self.assertEqual(node.operation['stage'], 'plan_result')
        node.nav.send_goal_async.assert_not_called()
        node.operation['future'] = NS(done=lambda: True, result=lambda: NS(status=5))
        poll(node)
        self.assertIsNone(node.operation)
        self.assertIsNone(node.cancel_reason)
        node.nav.send_goal_async.assert_not_called()
        node.clear_path.assert_called_once()

    def test_clear_path_replaces_previous_route_with_empty_message(self):
        node = MagicMock()
        message = NS(header=NS(frame_id=None), poses=[])
        explorer_method('clear_path', NavPath=lambda: message)(node)
        node.path_pub.publish.assert_called_once_with(message)
        self.assertEqual(message.header.frame_id, 'map')
        self.assertEqual(message.poses, [])


if __name__ == '__main__':
    unittest.main()
