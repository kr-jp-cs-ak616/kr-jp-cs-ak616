"""ROS와 분리한 목표 생성·완료 판정·Nav2 설정 검사."""
import copy
import json
import math
from pathlib import Path
import unittest
from house_nav2_frontier import FrontierMap, Completion, transform, valid_nav_path, goal_visuals, footprint_bounds, arrival_yaw
from prepare_house_nav2_frontier import configure, WAFFLE_PI_FOOTPRINT
from prepare_house_revisit import revisit_lua, localization_config


def rooms(door=True):
    w, h = 100, 65
    data = [100 if x in (0, 99) or y in (0, 64) or (x == 50 and not (door and 25 <= y <= 39))
            else -1 if x >= 78 else 0 for y in range(h) for x in range(w)]
    return FrontierMap(data, w, h, .1)


def config():
    return dict(controller_server=dict(ros__parameters=dict(controller_plugins=['FollowPath'],
                    FollowPath=dict(plugin='dwb_core::DWBLocalPlanner'), progress_checker_plugin='progress_checker',
                    goal_checker_plugins=['general_goal_checker'], general_goal_checker={})),
                planner_server=dict(ros__parameters=dict(planner_plugins=['GridBased'],
                    GridBased=dict(plugin='nav2_navfn_planner/NavfnPlanner', allow_unknown=True))),
                local_costmap=dict(local_costmap=dict(ros__parameters=dict(robot_radius=.22, footprint='[]'))),
                global_costmap=dict(global_costmap=dict(ros__parameters=dict(robot_radius=.22, static_layer={}))),
                behavior_server=dict(ros__parameters={}), bt_navigator=dict(ros__parameters={}),
                velocity_smoother=dict(ros__parameters={}))


class FrontierGoalTests(unittest.TestCase):
    def test_frontier_direction_gain_ignores_unknown_behind_robot(self):
        grid = rooms()
        i = grid.cell((7., 3.))
        self.assertGreater(grid.information_gain(i, heading=0.), 0)
        self.assertEqual(grid.information_gain(i, heading=math.pi), 0)

    def test_preferred_region_changes_which_frontier_is_selected(self):
        grid = rooms()
        lower = grid.select((2., 3., 0.), preferred=(7.8, 1.0))['goals'][0]
        upper = grid.select((2., 3., 0.), preferred=(7.8, 5.5))['goals'][0]
        self.assertLess(lower['frontier'][1], upper['frontier'][1])
        for goal in (lower, upper):
            self.assertTrue(grid.disk_clear(grid.cell(goal['point'])))
            self.assertGreater(goal['frontier_gain_m2'], 0)

    def test_goal_records_frontier_approach_distance(self):
        goal = rooms().select((2., 3., 0.))['goals'][0]
        self.assertAlmostEqual(goal['stand_off_m'], math.dist(goal['point'][:2], goal['frontier']))

    def test_arrival_yaw_uses_last_meaningful_path_segment(self):
        self.assertAlmostEqual(arrival_yaw([(0, 0), (1, 0), (1.2, 0)], math.pi/2), 0.)
        self.assertAlmostEqual(arrival_yaw([(0, 0), (0, .2), (0, .2)], 0.), math.pi/2)

    def test_arrival_yaw_falls_back_for_tiny_path(self):
        self.assertEqual(arrival_yaw([], .7), .7)
        self.assertEqual(arrival_yaw([(0, 0), (.01, 0)], .7), .7)
    def corridor(self, width=.40, origin=(0., 0., 0.)):
        res, w, h = .025, 48, 160
        data = [100 if abs((x+.5)*res-.60) >= width/2 or y == 0 else
                -1 if y >= 110 else 0 for y in range(h) for x in range(w)]
        return FrontierMap(data, w, h, res, origin, footprint=footprint_bounds(WAFFLE_PI_FOOTPRINT))

    def test_narrow_corridor_gets_goals_without_a_030m_empty_circle(self):
        grid = self.corridor()
        report = grid.select((.60, 1., math.pi/2))
        self.assertEqual(report['state'], 'ready')
        self.assertTrue(report['goals'])
        for goal in report['goals']:
            self.assertFalse(grid.disk_clear(grid.cell(goal['point'])))
            self.assertTrue(grid.body_clear(goal['point']))

    def test_too_narrow_corridor_does_not_force_movement(self):
        grid = self.corridor(.25)
        report = grid.select((.60, 1., math.pi/2))
        self.assertEqual(report['state'], 'no_candidates')
        self.assertFalse(report['start_body_clear'])
        self.assertGreater(report['frontiers'], 0)

    def test_footprint_rotation_and_rear_offset_are_checked(self):
        grid = self.corridor()
        self.assertTrue(grid.body_clear((.60, 1., math.pi/2)))
        self.assertFalse(grid.body_clear((.60, 1., 0.)))
        self.assertFalse(grid.body_clear((.60, 1., math.pi)))

    def test_narrow_corridor_with_rotated_map_origin(self):
        origin = (3., -2., .71)
        grid = self.corridor(origin=origin)
        position = transform((.6, 1.), origin)
        report = grid.select((*position, math.pi/2+origin[2]))
        self.assertEqual(report['state'], 'ready')
        self.assertTrue(all(grid.body_clear(g['point']) for g in report['goals']))

    def test_unknown_and_obstacle_under_goal_body_are_not_cleared(self):
        for value in (-1, 100):
            grid = self.corridor()
            grid.data[grid.cell((.60, .9))] = value
            grid.free[grid.cell((.60, .9))] = 0
            self.assertFalse(grid.body_clear((.60, 1., math.pi/2)))

    def test_padding_and_goal_footprint_validation(self):
        bounds = footprint_bounds(WAFFLE_PI_FOOTPRINT)
        self.assertAlmostEqual(bounds[0], -.215)
        self.assertAlmostEqual(bounds[3], .17)
        self.assertIsNone(footprint_bounds('[]'))
        for text, padding in (('[[0,0],[1,0],[1,1],[0,1]]', .01),
                              ('[[0,0]]', .01), (WAFFLE_PI_FOOTPRINT, -.01)):
            with self.assertRaises(ValueError):
                footprint_bounds(text, padding)

    def test_wide_room_works_with_new_rectangular_body_model(self):
        source = rooms()
        grid = FrontierMap(source.data, source.width, source.height, source.resolution,
                           footprint=footprint_bounds(WAFFLE_PI_FOOTPRINT))
        report = grid.select((2., 3., 0.))
        self.assertEqual(report['state'], 'ready')
        self.assertTrue(all(grid.body_clear(g['point']) for g in report['goals']))

    def test_selected_boundary_is_visible_from_every_goal(self):
        grid = rooms()
        for goal in grid.select((2, 3, 0))['goals']:
            self.assertTrue(grid.visible_edge(grid.cell(goal['point']), grid.cell(goal['frontier'])))
            self.assertGreater(goal['expected_gain_m2'], 0)

    def test_wall_hides_unknown_from_information_gain(self):
        w = 60
        data = [100 if x in (0, 59) or y in (0, 59) or x == 30 else
                -1 if x > 30 else 0 for y in range(60) for x in range(w)]
        grid = FrontierMap(data, w, 60, .1)
        self.assertEqual(grid.information_gain(grid.cell((2., 3.))), 0.)
        self.assertFalse(grid.visible_edge(grid.cell((2., 3.)), grid.cell((4., 3.))))

    def test_bigger_visible_unknown_has_more_gain(self):
        grid = rooms()
        a = grid.information_gain(grid.cell((7., 3.)))
        filled = [0 if v == -1 else v for v in grid.data]
        other = FrontierMap(filled, grid.width, grid.height, grid.resolution)
        self.assertGreater(a, other.information_gain(other.cell((7., 3.))))

    def test_revisit_restores_global_without_rewriting_sensor_weights(self):
        source = 'include "map_builder.lua"\noptions = {}\nreturn options\n'
        self.assertIn('optimize_every_n_nodes > 0', revisit_lua(source))
        self.assertIn('include "map_builder.lua"', revisit_lua(source))
        with self.assertRaises(ValueError):
            revisit_lua('POSE_GRAPH.optimize_every_n_nodes = 0\nreturn options\n')

    def test_amcl_mode_uses_saved_map_qos_and_preserves_navigation(self):
        original = config()
        original['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] = .08
        result = localization_config(original, {'amcl': {'ros__parameters': {'min_particles': 500}}})
        self.assertEqual(result['amcl']['ros__parameters']['min_particles'], 500)
        self.assertEqual(result['amcl']['ros__parameters']['base_frame_id'], 'base_footprint')
        self.assertFalse(result['amcl']['ros__parameters']['set_initial_pose'])
        self.assertTrue(result['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_subscribe_transient_local'])
        self.assertEqual(result['controller_server'], original['controller_server'])

    def test_next_room_frontier_is_selected_through_open_door(self):
        report = rooms().select((2, 3, 0))
        self.assertEqual(report['state'], 'ready')
        self.assertTrue(all(g['point'][0] > 6 for g in report['goals']))

    def test_closed_wall_is_not_treated_as_passage(self):
        report = rooms(False).select((2, 3, 0))
        self.assertEqual(report['state'], 'no_frontiers')
        self.assertFalse(report['goals'])

    def test_goals_are_known_free_with_body_clearance(self):
        grid = rooms()
        for g in grid.select((2, 3, 0))['goals']:
            i = grid.cell(g['point'])
            self.assertTrue(grid.free[i])
            self.assertTrue(grid.disk_clear(i))
            self.assertTrue(all(math.isfinite(x) for x in g['point']))

    def test_unknown_center_is_waiting_not_mapping_complete(self):
        grid = rooms()
        grid.data[grid.cell((2, 3))] = -1
        grid = FrontierMap(grid.data, grid.width, grid.height, grid.resolution)
        self.assertEqual(grid.select((2, 3, 0))['state'], 'start_unobserved')

    def test_footprint_unknown_near_start_does_not_block_all_global_goals(self):
        grid = rooms()
        grid.data[grid.cell((2.15, 3))] = -1
        grid = FrontierMap(grid.data, grid.width, grid.height, grid.resolution)
        self.assertEqual(grid.select((2, 3, 0))['state'], 'ready')

    def test_no_safe_goal_is_not_completion(self):
        data = [-1]*400
        for y in range(8, 12):
            for x in range(8, 12):
                data[y*20+x] = 0
        report = FrontierMap(data, 20, 20, .05).select((.5, .5, 0))
        self.assertEqual(report['state'], 'no_candidates')
        self.assertGreater(report['frontiers'], 0)
        self.assertFalse(Completion(15).update(report, 100, 1))

    def test_failed_goal_is_temporarily_excluded_then_retried(self):
        grid = rooms()
        first = grid.select((2, 3, 0))['goals'][0]
        blocked = [dict(point=first['point'][:2], signature=first['signature'], time=0, success=False)]
        newer = grid.select((2, 3, 0), blocked, 1)['goals']
        self.assertNotIn(first['point'], [g['point'] for g in newer])
        later = grid.select((2, 3, 0), blocked, 121)['goals']
        self.assertIn(first['point'], [g['point'] for g in later])

    def test_reached_unchanged_goal_is_not_repeated(self):
        grid = rooms()
        first = grid.select((2, 3, 0))['goals'][0]
        reached = [dict(point=first['point'][:2], signature=first['signature'], time=0, success=True)]
        self.assertNotIn(first['point'], [g['point'] for g in grid.select((2, 3, 0), reached, 1000)['goals']])

    def test_rotated_map_origin_round_trip(self):
        grid = FrontierMap([0]*400, 20, 20, .1, (4, -2, math.pi/2))
        for i in range(400):
            self.assertEqual(grid.cell(grid.world(i)), i)

    def test_odom_origin_can_be_reprojected_after_map_correction(self):
        original = (.7, -.3)
        map_frame = (2, 3, .5)
        self.assertLess(math.dist(transform(transform(original, map_frame), map_frame, inverse=True), original), 1e-8)

    def test_completion_needs_stable_time_and_distinct_maps(self):
        gate = Completion(60)
        report = dict(state='no_frontiers', area=20)
        self.assertFalse(gate.update(report, 0, 1))
        self.assertFalse(gate.update(report, 61, 1))
        self.assertFalse(gate.update(report, 62, 2))
        self.assertTrue(gate.update(report, 63, 3))

    def test_new_frontier_clears_completion_hold(self):
        gate = Completion(15)
        gate.update(dict(state='no_frontiers', area=20), 0, 1)
        gate.update(dict(state='ready', area=20), 10, 2)
        self.assertFalse(gate.update(dict(state='no_frontiers', area=20), 20, 3))

    def test_nav2_empty_or_truncated_path_is_rejected(self):
        self.assertFalse(valid_nav_path([], (1, 2, 0)))
        self.assertFalse(valid_nav_path([(0, 0), (.5, 2)], (1, 2, 0)))
        self.assertFalse(valid_nav_path([(math.nan, 0), (1, 2)], (1, 2, 0)))
        self.assertTrue(valid_nav_path([(0, 0), (1, 2)], (1, 2, 0)))

    def test_user_report_map_produces_coordinate_candidates(self):
        case = json.loads((Path(__file__).parent/'testdata'/'frontier_follow_corner.json').read_text(encoding='utf-8'))
        g = case['grid']
        grid = FrontierMap(g['data'], g['width'], g['height'], g['resolution'], g['origin'])
        report = grid.select(case['pose'])
        self.assertEqual(report['state'], 'ready')
        self.assertTrue(report['goals'])


class NavConfigTests(unittest.TestCase):
    def test_original_config_is_preserved(self):
        source = config()
        saved = copy.deepcopy(source)
        configure(source)
        self.assertEqual(source, saved)

    def test_requested_speed_and_stopped_threshold(self):
        c = configure(config())
        f = c['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(f['max_vel_x'], .10)
        self.assertEqual(f['max_speed_xy'], .10)
        self.assertLess(f['trans_stopped_velocity'], f['max_vel_x'])
        self.assertEqual(c['velocity_smoother']['ros__parameters']['max_velocity'][0], .10)
        self.assertEqual(f['max_vel_theta'], .20)
        self.assertEqual(c['velocity_smoother']['ros__parameters']['max_velocity'][2], .20)
        self.assertEqual(c['behavior_server']['ros__parameters']['max_rotational_vel'], .20)

    def test_unknown_space_is_not_a_nav2_shortcut(self):
        c = configure(config())
        self.assertFalse(c['planner_server']['ros__parameters']['GridBased']['allow_unknown'])
        self.assertFalse(c['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_subscribe_transient_local'])
        self.assertEqual(c['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'], '/map')

    def test_empty_footprint_uses_conservative_radius(self):
        c = configure(config())
        self.assertEqual(c['local_costmap']['local_costmap']['ros__parameters']['robot_radius'], .28)

    def test_explicit_polygon_is_preserved(self):
        source = config()
        polygon = '[[-0.2,-0.2],[0.2,-0.2],[0.2,0.2],[-0.2,0.2]]'
        source['local_costmap']['local_costmap']['ros__parameters']['footprint'] = polygon
        self.assertEqual(configure(source)['local_costmap']['local_costmap']['ros__parameters']['footprint'], polygon)

    def test_waffle_polygon_and_padding_match_both_costmaps(self):
        c = configure(config(), waffle_footprint=True)
        for name in ('local_costmap', 'global_costmap'):
            params = c[name][name]['ros__parameters']
            self.assertEqual(params['footprint'], WAFFLE_PI_FOOTPRINT)
            self.assertEqual(params['footprint_padding'], .01)
        self.assertEqual(c['controller_server']['ros__parameters']['FollowPath']['max_vel_x'], .10)

    def test_polygon_controller_checks_full_footprint_and_keeps_other_critics(self):
        c = config()
        c['controller_server']['ros__parameters']['FollowPath'].update(
            critics=['Oscillation', 'BaseObstacle', 'GoalAlign'], **{'BaseObstacle.scale': .03})
        follow = configure(c, waffle_footprint=True)['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(follow['critics'], ['Oscillation', 'ObstacleFootprint', 'GoalAlign'])
        self.assertEqual(follow['ObstacleFootprint.scale'], .03)

    def test_waffle_polygon_encloses_standard_sdf_body_and_wheels(self):
        points = json.loads(WAFFLE_PI_FOOTPRINT)
        minx, maxx = min(p[0] for p in points), max(p[0] for p in points)
        miny, maxy = min(p[1] for p in points), max(p[1] for p in points)
        self.assertLess(minx, -.064-.265/2)
        self.assertGreater(maxx, -.064+.265/2)
        self.assertLess(miny, -.144-.018/2)
        self.assertGreater(maxy, .144+.018/2)

    def test_wrong_speed_or_controller_is_rejected(self):
        with self.assertRaises(ValueError):
            configure(config(), .8)
        source = config()
        source['controller_server']['ros__parameters']['FollowPath']['plugin'] = 'other_controller'
        with self.assertRaises(ValueError):
            configure(source)


class GoalVisualTests(unittest.TestCase):
    def test_preferred_region_is_visible_without_pretending_it_is_a_nav_goal(self):
        specs = goal_visuals([], None, preferred=(1., 2.))
        self.assertTrue(any(s['ns'] == 'preferred_region' for s in specs))
        self.assertFalse(any(s['ns'] == 'selected' for s in specs))

    def test_candidates_are_distinct_from_active_goal(self):
        goals = [dict(point=(1., 2., 0.)), dict(point=(3., 4., .5))]
        specs = goal_visuals(goals, goals[0], 'nav_result')
        self.assertEqual(len([s for s in specs if s['ns'] == 'candidates']), 2)
        selected = next(s for s in specs if s['ns'] == 'selected_label')
        self.assertEqual(selected['text'], 'GOAL x=1.00 y=2.00')
        self.assertGreater(selected['color'][1], selected['color'][0])

    def test_path_check_is_not_labeled_as_sent_nav_goal(self):
        specs = goal_visuals([], dict(point=(1, 2, 0)), 'plan_result')
        self.assertTrue(next(s['text'] for s in specs if s['kind'] == 'text').startswith('CHECK'))

    def test_paused_or_finished_goal_is_not_shown_as_active(self):
        for flag, prefix in (('paused', 'PAUSED'), ('done', 'DONE')):
            specs = goal_visuals([], dict(point=(1, 2, 0)), 'nav_result', **{flag: True})
            self.assertTrue(next(s['text'] for s in specs if s['kind'] == 'text').startswith(prefix))

    def test_home_is_distinguished_and_empty_batch_has_no_old_candidates(self):
        specs = goal_visuals([], dict(point=(1, 2, 0)), 'nav_result', home=True)
        self.assertFalse(any(s['ns'] == 'candidates' for s in specs))
        self.assertTrue(next(s['text'] for s in specs if s['kind'] == 'text').startswith('HOME'))

    def test_marker_ids_are_unique_and_candidate_count_is_bounded(self):
        goals = [dict(point=(float(i), 2., 0.)) for i in range(50)]
        specs = goal_visuals(goals, goals[0], 'nav_result')
        self.assertEqual(len(specs), 42)
        ids = [(s['ns'], s['id']) for s in specs]
        self.assertEqual(len(ids), len(set(ids)))


if __name__ == '__main__':
    unittest.main()
