"""속도 명령·실제 움직임·Nav2 로그를 함께 관측한다. 주행 명령이나 설정 변경은 하지 않는다."""
import argparse
from collections import Counter, deque
from datetime import datetime
import json
import math
from pathlib import Path
import time


def classify_motion(s):
    if s.get('clock_age', math.inf) > 2:
        return 'clock_wait', '시뮬레이션 시간 갱신 중단 또는 수신 대기'
    if s.get('scan_age', math.inf) > .7:
        return 'scan_wait', '라이다 수신 공백'
    if s.get('scan_stamp_age') is not None and not -.3 <= s['scan_stamp_age'] <= .7:
        return 'scan_time_mismatch', '라이다 시각과 시뮬레이션 시각 불일치'
    state = s.get('explorer') or {}
    if state.get('wait_reason'):
        return 'input_wait', state['wait_reason']
    if state.get('paused'):
        return 'paused', '자동 탐색 일시정지: 취소 이유와 사건 로그 확인'
    stage = state.get('operation')
    if stage and stage.startswith('plan'):
        return 'planning', 'Nav2 경로 확인 응답 대기'
    if s.get('command_age', math.inf) > 1:
        return 'command_missing', '최근 /cmd_vel 수신 없음'
    x, yaw = s['command']
    if abs(x) < .005 and abs(yaw) > .01:
        if s.get('translation_10s') is not None and s['translation_10s'] < .05:
            return 'long_rotation', '10초 이상 위치 변화가 작은 회전: 방향 정렬·복구 로그와 대조'
        return 'rotating', '직진 명령 없이 회전 중'
    if abs(x) < .005 and abs(yaw) <= .01:
        if not stage:
            return 'goal_wait', '활성 목표 없음: 후보·완료·준비 상태 확인'
        return 'zero_command', 'Nav2 목표 진행 중이지만 최종 속도 명령이 0에 가까움'
    if s.get('odom_age', math.inf) > 1:
        return 'odom_wait', '이동 명령은 있으나 최근 오도메트리 수신 없음'
    if abs(x) >= .02 and abs(s['actual'][0]) < .005:
        return 'little_motion', '전진 명령에 비해 실제 속도가 작음: 접촉·물리·명령 경합 확인'
    return 'moving', '이동 중'


def compact_state(state):
    if not state:
        return None
    result = {k: state.get(k) for k in ('clock', 'paused', 'done', 'operation', 'wait_reason',
              'cancel_reason', 'goal', 'pose', 'feedback', 'preferred_odom', 'auto_interest_odom',
              'inspection_requested', 'events_log')}
    report = state.get('report') or {}
    result['report'] = {k: report.get(k) for k in ('state', 'frontiers', 'area', 'body_model',
                                                'start_body_clear', 'reachable_cells', 'pending', 'exploration_state')}
    result['report']['candidate_count'] = len(report.get('goals', []))
    return result


def clean_numbers(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean_numbers(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_numbers(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.seconds) or not 5 <= args.seconds <= 3600:
        raise SystemExit('--seconds는 5~3600초 범위의 실제 시간입니다.')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PolygonStamped
    from nav_msgs.msg import Odometry, Path as PathMessage
    from nav2_msgs.msg import Costmap
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from rcl_interfaces.msg import Log

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('house_motion_diagnostics', parameter_overrides=[Parameter('use_sim_time', value=True)])
    base = Path.home()/'ros2_practice_ws'
    output = base/'diagnostics'/('motion_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output.parent.mkdir(parents=True, exist_ok=True)
    received, messages = {}, {}
    samples, logs, motion = [], deque(maxlen=1000), deque(maxlen=120)
    counts = Counter()
    latest_state = None
    clock_value, clock_wall = None, -math.inf

    def on_message(name, message):
        messages[name], received[name] = message, time.monotonic()

    def on_clock(message):
        nonlocal clock_value, clock_wall
        stamp = message.clock.sec+message.clock.nanosec/1e9
        if clock_value != stamp:
            clock_value, clock_wall = stamp, time.monotonic()

    def callback_for(name):
        def callback(message):
            on_message(name, message)
        return callback

    def on_log(message):
        relevant = any(n in message.name for n in ('controller', 'planner', 'behavior', 'bt_navigator', 'house_nav2_frontier'))
        if relevant and (message.level >= 30 or any(k in message.msg for k in ('Goal', 'goal', 'progress', 'recover', 'Spin'))):
            logs.append(dict(stamp=message.stamp.sec+message.stamp.nanosec/1e9,
                             node=message.name, level=message.level, message=message.msg))

    for topic, kind in (('/cmd_vel', Twist), ('/cmd_vel_nav', Twist), ('/odom', Odometry),
                        ('/scan', LaserScan), ('/plan', PathMessage),
                        ('/local_costmap/costmap_raw', Costmap),
                        ('/local_costmap/published_footprint', PolygonStamped)):
        node.create_subscription(kind, topic, callback_for(topic), qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(Log, '/rosout', on_log, 200)
    state_path = base/'house_nav2_frontier_state.json'
    print(f'{args.seconds:g}초간 읽기 전용 관측합니다. Gazebo·Nav2·자동 탐색기를 실행 상태로 두세요.')
    print('결과 저장:', str(output)+'.json')
    start, next_sample = time.monotonic(), time.monotonic()+1
    try:
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now < next_sample:
                continue
            next_sample = now+1
            try:
                state = json.loads(state_path.read_text(encoding='utf-8'))
                state_age = time.time()-state_path.stat().st_mtime
                latest_state = state
            except (OSError, ValueError):
                state, state_age = None, None
            cmd, odom = messages.get('/cmd_vel'), messages.get('/odom')
            position = None if odom is None else (odom.pose.pose.position.x, odom.pose.pose.position.y)
            if position is not None and clock_value is not None:
                if motion and clock_value < motion[-1][0]:
                    motion.clear()
                motion.append((clock_value, position))
            previous = next((p for t, p in reversed(motion) if clock_value is not None and clock_value-t >= 10), None)
            snapshot = dict(elapsed_wall=now-start, clock=clock_value, clock_age=now-clock_wall,
                            scan_age=now-received.get('/scan', -math.inf),
                            command_age=now-received.get('/cmd_vel', -math.inf),
                            odom_age=now-received.get('/odom', -math.inf),
                            command=[0., 0.] if cmd is None else [cmd.linear.x, cmd.angular.z],
                            actual=[0., 0.] if odom is None else [odom.twist.twist.linear.x, odom.twist.twist.angular.z],
                            translation_10s=math.dist(position, previous) if previous is not None else None,
                            explorer=compact_state(state) if state_age is not None and state_age <= 8 else None,
                            explorer_file_age=state_age)
            before = messages.get('/cmd_vel_nav')
            scan = messages.get('/scan')
            snapshot['scan_stamp_age'] = None if scan is None or clock_value is None else clock_value-(scan.header.stamp.sec+scan.header.stamp.nanosec/1e9)
            snapshot['command_before_smoothing'] = None if before is None else [before.linear.x, before.angular.z]
            label, explanation = classify_motion(snapshot)
            snapshot.update(classification=label, explanation=explanation)
            counts[label] += 1
            samples.append(snapshot)
            if len(samples) % 5 == 0:
                print(f"[{len(samples)}] {explanation} / 명령={snapshot['command']} / 실제={snapshot['actual']}")
    except KeyboardInterrupt:
        pass
    finally:
        report = dict(note='단일 관측의 분류는 원인 확정이 아닙니다. 센서 시각·Nav2 경고와 함께 확인하세요.',
                      counts=dict(counts), samples=samples, rosout=list(logs), final_explorer_state=latest_state,
                      command_publishers=[dict(name=p.node_name, namespace=p.node_namespace)
                                          for p in node.get_publishers_info_by_topic('/cmd_vel')])
        plan = messages.get('/plan')
        if plan is not None:
            report['latest_plan'] = dict(frame=plan.header.frame_id,
                                         stamp=plan.header.stamp.sec+plan.header.stamp.nanosec/1e9,
                                         points=[[p.pose.position.x, p.pose.position.y] for p in plan.poses])
        cost = messages.get('/local_costmap/costmap_raw')
        if cost is not None:
            meta = cost.metadata
            report['local_costmap'] = dict(frame=cost.header.frame_id,
                stamp=cost.header.stamp.sec+cost.header.stamp.nanosec/1e9,
                width=meta.size_x, height=meta.size_y, resolution=meta.resolution,
                origin=[meta.origin.position.x, meta.origin.position.y],
                origin_quaternion=[meta.origin.orientation.x, meta.origin.orientation.y,
                                   meta.origin.orientation.z, meta.origin.orientation.w], data=list(cost.data))
        polygon = messages.get('/local_costmap/published_footprint')
        if polygon is not None:
            report['footprint'] = dict(frame=polygon.header.frame_id,
                stamp=polygon.header.stamp.sec+polygon.header.stamp.nanosec/1e9,
                points=[[p.x, p.y] for p in polygon.polygon.points])
        Path(str(output)+'.json').write_text(json.dumps(clean_numbers(report), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        print('관측 종료. 이 파일을 보내주세요:', str(output)+'.json')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
