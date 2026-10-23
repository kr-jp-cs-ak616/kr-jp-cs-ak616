"""House 시각 고정 진단 자료를 한 파일에 수집한다. 이동·설정 변경 없음."""
import json
import os
from pathlib import Path
import subprocess
import time
from collections import defaultdict

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import LaserScan


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def summarize(values):
    stamps = [entry[1] for entry in values]
    return {'samples': len(values), 'first_stamp': stamps[0], 'last_stamp': stamps[-1],
            'min_stamp': min(stamps), 'max_stamp': max(stamps),
            'backward_steps': sum(b < a-0.001 for a, b in zip(stamps, stamps[1:])),
            'distinct_stamps': len(set(stamps)), 'last_receive_wall': values[-1][0]}


def run(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        return result.stdout+result.stderr
    except (subprocess.TimeoutExpired, OSError) as exc:
        return str(exc)


def main():
    rclpy.init()
    node = rclpy.create_node('house_state_collector')
    samples = defaultdict(list)
    edges = defaultdict(list)
    parents = defaultdict(set)
    external_tf = defaultdict(list)
    subscriptions = []
    tf = Buffer(cache_time=Duration(seconds=15))
    listener = TransformListener(tf, node)

    def header_callback(label):
        def callback(msg):
            samples[label].append((time.monotonic(), seconds(msg.header.stamp)))
        return callback

    def clock_callback(msg):
        samples['/clock'].append((time.monotonic(), seconds(msg.clock)))

    def tf_callback(label):
        def callback(msg):
            for transform in msg.transforms:
                parent = transform.header.frame_id
                child = transform.child_frame_id
                parents[child].add(parent)
                edges[(label, parent, child)].append((time.monotonic(), seconds(transform.header.stamp)))
        return callback

    for label, msg_type in [('/scan', LaserScan), ('/odom', Odometry),
                            ('/local_costmap/costmap', OccupancyGrid),
                            ('/local_costmap/published_footprint', PolygonStamped)]:
        subscriptions.append(node.create_subscription(msg_type, label, header_callback(label), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(TFMessage, '/tf', tf_callback('/tf'), qos_profile_sensor_data))
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(TFMessage, '/tf_static', tf_callback('/tf_static'), static_qos))
    print('20초간 관측합니다. Gazebo·SLAM·Nav2 실행, 탐색 종료 상태로 두세요.', flush=True)
    end = time.monotonic()+20
    last_probe = 0.0
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic()-last_probe >= 1:
                last_probe = time.monotonic()
                for target, source in [('odom', 'base_footprint'), ('odom', 'base_link'), ('map', 'odom')]:
                    try:
                        transform = tf.lookup_transform(target, source, Time())
                        external_tf[target+' <- '+source].append({
                            'stamp': seconds(transform.header.stamp),
                            'xyz': [transform.transform.translation.x, transform.transform.translation.y,
                                    transform.transform.translation.z]})
                    except Exception as exc:
                        external_tf[target+' <- '+source].append({'error': str(exc)})
        report = {
            'note': '설정 변경 없이 관측. external_tf는 Nav2 내부 버퍼가 아니라 새 관측 노드의 버퍼.',
            'environment': {key: os.environ.get(key) for key in (
                'ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI')},
            'topic_samples': {key: summarize(value) for key, value in samples.items()},
            'tf_edges': {' | '.join(key): summarize(value) for key, value in edges.items()},
            'tf_parents': {key: sorted(value) for key, value in parents.items()},
            'external_tf': dict(external_tf),
            'tf_graph': tf.all_frames_as_yaml(),
        }
        report['publishers'] = {}
        for topic in ('/clock', '/tf', '/tf_static', '/local_costmap/published_footprint'):
            report['publishers'][topic] = [
                {'node': endpoint.node_namespace+'/'+endpoint.node_name,
                 'gid': list(endpoint.endpoint_gid), 'qos': str(endpoint.qos_profile)}
                for endpoint in node.get_publishers_info_by_topic(topic)]
        print('실행 설정과 패키지 버전을 수집합니다. 잠시 기다려 주세요.', flush=True)
        report['node_list'] = run(['ros2', 'node', 'list'])
        report['parameters'] = {}
        for name in ('/local_costmap/local_costmap', '/global_costmap/global_costmap',
                     '/controller_server', '/behavior_server', '/cartographer_node'):
            report['parameters'][name] = run(['ros2', 'param', 'dump', name])
        report['packages'] = run(['dpkg-query', '-W', '-f=${Package} ${Version}\n',
                                 'ros-humble-nav2-*', 'ros-humble-tf2*', 'ros-humble-rclcpp*',
                                 'ros-humble-rmw*', 'ros-humble-fastrtps*', 'ros-humble-cyclonedds*'])
        report['processes'] = run(['ps', '-eo', 'pid,ppid,pcpu,pmem,comm', '--sort=-pcpu'])
        destination = Path.home()/'ros2_practice_ws/house_state_report.txt'
        destination.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print('저장 완료:', destination)
        print('이 파일 전체를 전달해 주세요. 로봇 이동이나 매개변수 변경은 하지 않았습니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
