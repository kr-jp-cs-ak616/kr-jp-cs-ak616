"""현재 Waffle Pi Nav2 설정을 복사해 자동 목표 탐색용 0.10m/s 설정을 만든다."""
import argparse
import copy
from datetime import datetime
from pathlib import Path


WAFFLE_PI_FOOTPRINT = '[[-0.205,-0.16],[0.10,-0.16],[0.10,0.16],[-0.205,0.16]]'


def configure(original, speed=.10, waffle_footprint=False, yaw_rate=.20):
    if not 0 < speed <= .26:
        raise ValueError('speed 단위는 m/s이며 0 초과~0.26 범위여야 합니다.')
    if not 0 < yaw_rate <= .30:
        raise ValueError('이번 매핑 프로필의 yaw_rate 범위는 0 초과~0.30rad/s입니다.')
    config = copy.deepcopy(original)
    for value in config.values():
        if isinstance(value, dict) and isinstance(value.get('ros__parameters'), dict):
            value['ros__parameters']['use_sim_time'] = True
    controller = config['controller_server']['ros__parameters']
    ids = controller.get('controller_plugins', ['FollowPath'])
    if len(ids) != 1 or 'DWBLocalPlanner' not in controller[ids[0]].get('plugin', ''):
        raise ValueError('이 설정 도구는 단일 DWBLocalPlanner 구성만 지원합니다. 기존 YAML을 확인하세요.')
    follow = controller[ids[0]]
    follow.update(max_vel_x=speed, max_speed_xy=speed, max_vel_y=0., min_vel_x=0.,
                  min_speed_xy=0., min_speed_theta=0., max_vel_theta=yaw_rate,
                  acc_lim_x=.10, decel_lim_x=-.10, acc_lim_theta=.30, decel_lim_theta=-.30,
                  trans_stopped_velocity=.02, xy_goal_tolerance=.15)
    if waffle_footprint:
        # 사각 몸체는 중심 셀만 검사하는 BaseObstacle 대신 전체 윤곽으로 검사한다.
        critics = follow.get('critics', ['RotateToGoal', 'Oscillation', 'BaseObstacle',
                                       'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'])
        follow['critics'] = list(dict.fromkeys('ObstacleFootprint' if c == 'BaseObstacle' else c for c in critics))
        if 'ObstacleFootprint' not in follow['critics']:
            follow['critics'].append('ObstacleFootprint')
        follow.setdefault('ObstacleFootprint.scale', follow.get('BaseObstacle.scale', .02))
    checker = controller.get('progress_checker_plugin', 'progress_checker')
    controller.setdefault(checker, {}).update(required_movement_radius=.10, movement_time_allowance=30.)
    for checker_id in controller.get('goal_checker_plugins', ['general_goal_checker']):
        controller.setdefault(checker_id, {}).update(xy_goal_tolerance=.15, yaw_goal_tolerance=.30)
    for name in ('local_costmap', 'global_costmap'):
        params = config[name][name]['ros__parameters']
        params.update(use_sim_time=True, robot_base_frame='base_footprint')
        if waffle_footprint:
            params.update(footprint=WAFFLE_PI_FOOTPRINT, footprint_padding=.01)
        # 기존 윤곽이 있으면 보존한다. 원형 모델은 보수적인 기존 시험 반경을 유지한다.
        if not params.get('footprint') or str(params.get('footprint')).strip() == '[]':
            params['robot_radius'] = max(.28, float(params.get('robot_radius', .28)))
        static = params.get('static_layer')
        if static is not None:
            static.update(map_subscribe_transient_local=False, lethal_cost_threshold=65,
                          trinary_costmap=True, track_unknown_space=True, map_topic='/map')
    config['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] = True
    planner = config['planner_server']['ros__parameters']
    for plugin_id in planner.get('planner_plugins', ['GridBased']):
        plugin = planner[plugin_id]
        if 'NavfnPlanner' not in plugin.get('plugin', ''):
            raise ValueError('이 설정 도구는 NavfnPlanner 구성을 사용합니다. 다른 플래너는 별도 확인이 필요합니다.')
        plugin.update(allow_unknown=False, tolerance=.15)
    for name in ('bt_navigator', 'behavior_server'):
        if name in config:
            config[name]['ros__parameters']['robot_base_frame'] = 'base_footprint'
    behavior = config.get('behavior_server', {}).get('ros__parameters', {})
    behavior.update(max_rotational_vel=yaw_rate, min_rotational_vel=min(.08, yaw_rate), rotational_acc_lim=.30)
    smoother = config.get('velocity_smoother', {}).get('ros__parameters')
    if smoother is not None:
        smoother.update(max_velocity=[speed, 0., yaw_rate], min_velocity=[-speed, 0., -yaw_rate],
                        max_accel=[.10, 0., .30], max_decel=[-.10, 0., -.30])
    return config


def main():
    import yaml
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path)
    parser.add_argument('--speed', type=float, default=.10)
    parser.add_argument('--yaw-rate', type=float, default=.20)
    parser.add_argument('--waffle-footprint', action='store_true',
                        help='표준 Gazebo Waffle Pi 몸체·바퀴를 포함하는 사각 윤곽과 DWB 윤곽 충돌 검사를 사용')
    args = parser.parse_args()
    root = Path.home()/'ros2_practice_ws'
    source = args.source
    if source is None:
        candidates = [root/'nav_house_base_footprint.yaml', root/'nav_explore_waffle_pi.yaml']
        source = next((p for p in candidates if p.is_file()), None)
    if source is None:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('turtlebot3_navigation2'))
        source = next((p for p in (share/'param/humble/waffle_pi.yaml', share/'param/waffle_pi.yaml')
                       if p.is_file()), None)
    if source is None or not source.is_file():
        raise SystemExit('Waffle Pi 설정을 찾지 못했습니다. --source /절대경로/설정.yaml 을 지정하세요.')
    original = yaml.safe_load(source.read_text(encoding='utf-8'))
    config = configure(original, args.speed, args.waffle_footprint, args.yaw_rate)
    target = root/'nav_house_frontier.yaml'
    if source.resolve() == target.resolve():
        raise SystemExit('원본과 출력 파일은 서로 달라야 합니다.')
    root.mkdir(parents=True, exist_ok=True)
    if target.exists():
        backup = target.with_suffix('.yaml.backup_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        backup.write_bytes(target.read_bytes())
        print('기존 출력 백업:', backup)
    target.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding='utf-8')
    print('기준 원본:', source)
    print('생성한 Nav2 설정:', target)
    print('선속도 상한:', args.speed, 'm/s / 원본 설정은 보존했습니다.')
    print('회전 상한:', args.yaw_rate, 'rad/s / 가속도 0.10m/s², 각가속도 0.30rad/s²')
    if args.waffle_footprint:
        print('표준 Gazebo Waffle Pi 윤곽:', WAFFLE_PI_FOOTPRINT, '/ 여유 0.01m')
        print('로컬·전역 비용지도에 적용, DWB ObstacleFootprint 사용. robot_radius는 사각 윤곽 모드에서 사용하지 않습니다.')
        print('표준 모델의 x=-0.1965~0.0685m 몸체와 y=±0.153m 바퀴를 포함합니다. 부착물·개조 모델에는 별도 치수가 필요합니다.')


if __name__ == '__main__':
    main()
