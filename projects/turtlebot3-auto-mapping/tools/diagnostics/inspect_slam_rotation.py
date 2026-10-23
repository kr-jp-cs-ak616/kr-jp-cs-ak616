"""설정 변경·속도 명령 없이 SLAM 회전 상태를 관측한다. Ubuntu ROS2 Humble용."""
import json
import math
from pathlib import Path
import re
import time


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def lua_settings(text):
    text = re.sub(r'--[^\n]*', '', text)
    result = {}
    for key in ['map_frame', 'tracking_frame', 'published_frame', 'odom_frame',
                'provide_odom_frame', 'publish_frame_projected_to_2d', 'use_odometry',
                'TRAJECTORY_BUILDER_2D.use_imu_data', 'num_subdivisions_per_laser_scan']:
        matches = re.findall(r'(?m)^\s*'+re.escape(key)+r'\s*=\s*([^,\n]+)', text)
        if matches:
            result[key] = matches[-1].strip().strip('\"\'')
    return result


def active_configs():
    configs = []
    proc = Path('/proc')
    if not proc.exists():
        return configs
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        matched = False
        try:
            args = (entry/'cmdline').read_bytes().decode(errors='replace').strip('\x00').split('\x00')
            if not args or Path(args[0]).name != 'cartographer_node':
                continue
            matched = True
            def argument(name):
                return args[args.index(name)+1] if name in args else None
            directory = argument('-configuration_directory')
            basename = argument('-configuration_basename')
            item = {'pid': int(entry.name), 'directory': directory, 'basename': basename}
            if directory and basename:
                source = Path(directory)/basename
                if not source.is_absolute():
                    source = (entry/'cwd').resolve()/source
                item['path'] = str(source)
                item['settings'] = lua_settings(source.read_text())
            configs.append(item)
        except (OSError, ValueError, IndexError) as exc:
            if matched:
                configs.append({'pid': int(entry.name), 'read_error': str(exc)})
    return configs


class PoseStats:
    def __init__(self):
        self.first = self.last = None
        self.samples = 0
        self.max_step_m = self.max_step_deg = self.total_yaw = 0.0
        self.signed_yaw = 0.0
        self.backwards = 0
        self.last_rx = None

    def add(self, stamp, x, y, yaw):
        value = (stamp, x, y, yaw)
        if self.last:
            if stamp < self.last[0]:
                self.backwards += 1
            self.max_step_m = max(self.max_step_m, math.hypot(x-self.last[1], y-self.last[2]))
            step = abs(math.degrees(angle_delta(yaw, self.last[3])))
            self.max_step_deg = max(self.max_step_deg, step)
            self.total_yaw += step
            self.signed_yaw += math.degrees(angle_delta(yaw, self.last[3]))
        self.first = self.first or value
        self.last = value
        self.samples += 1
        self.last_rx = time.monotonic()

    def summary(self, clock):
        if self.last is None:
            return {'samples': 0}
        return {'samples': self.samples, 'last_stamp': self.last[0],
                'clock_age_sec': None if clock is None else clock-self.last[0],
                'receive_gap_sec': time.monotonic()-self.last_rx,
                'max_step_m': self.max_step_m, 'max_step_deg': self.max_step_deg,
                'total_yaw_deg': self.total_yaw, 'backward_stamps': self.backwards,
                'signed_yaw_deg': self.signed_yaw,
                'net_move_m': math.hypot(self.last[1]-self.first[1], self.last[2]-self.first[2])}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Imu
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from rcl_interfaces.srv import GetParameters

    rclpy.init()
    node = Node('inspect_slam_rotation', parameter_overrides=[])
    clock = [None]
    odom = PoseStats()
    edges, parents = {}, {}
    scan_stamps, scan_ages, scan_meta = [], [], {}
    imu_z = []
    agreement = []

    def stamp(message):
        return message.sec+message.nanosec/1e9

    def on_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odom.add(stamp(msg.header.stamp), p.x, p.y, yaw_of(q))
        key = msg.header.frame_id.strip('/')+' -> '+msg.child_frame_id.strip('/')
        if key in edges and edges[key].last:
            t, x, y, yaw = edges[key].last
            if abs(t-stamp(msg.header.stamp)) < .06:
                agreement.append({'yaw_error_deg': abs(math.degrees(angle_delta(yaw_of(q), yaw))),
                                  'xy_error_m': math.hypot(p.x-x, p.y-y)})

    def on_tf(msg, static):
        for tf in msg.transforms:
            parent, child = tf.header.frame_id.strip('/'), tf.child_frame_id.strip('/')
            parents.setdefault(child, set()).add(parent)
            key = parent+' -> '+child
            if static:
                key = '정적 '+key
            if child not in ('odom', 'base_footprint', 'base_link', 'base_scan', 'imu_link'):
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            edges.setdefault(key, PoseStats()).add(stamp(tf.header.stamp), p.x, p.y, yaw_of(q))

    def on_scan(msg):
        t = stamp(msg.header.stamp)
        scan_stamps.append(t)
        if clock[0] is not None:
            scan_ages.append(clock[0]-t)
        scan_meta.update(frame=msg.header.frame_id, rays=len(msg.ranges), scan_time=msg.scan_time,
                         time_increment=msg.time_increment, angle_increment=msg.angle_increment,
                         range_min=msg.range_min, range_max=msg.range_max)

    subscriptions = [
        node.create_subscription(Clock, '/clock', lambda m: clock.__setitem__(0, stamp(m.clock)), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data),
        node.create_subscription(Imu, '/imu', lambda m: imu_z.append(m.angular_velocity.z), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/tf', lambda m: on_tf(m, False), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/tf_static', lambda m: on_tf(m, True),
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE)),
    ]
    configs = active_configs()
    client = node.create_client(GetParameters, '/cartographer_node/get_parameters')
    future = None
    if client.wait_for_service(timeout_sec=2):
        request = GetParameters.Request()
        request.names = ['use_sim_time']
        future = client.call_async(request)
    print('20초 동안 설정과 센서를 관측합니다. 이 코드는 주행 명령을 보내지 않습니다.', flush=True)
    print('회전 중 문제를 확인하려면 기존 벽 추종기를 저속으로 실행한 상태에서 관측하세요.', flush=True)
    try:
        deadline = time.monotonic()+20
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        sim = None
        if future is not None and future.done() and future.exception() is None:
            values = future.result().values
            sim = values[0].bool_value if values else None
        publishers = {}
        for topic in ['/tf', '/tf_static', '/odom', '/scan', '/clock']:
            publishers[topic] = [i.node_namespace.rstrip('/')+'/'+i.node_name
                                 for i in node.get_publishers_info_by_topic(topic)]
        report = {'running_configs': configs, 'cartographer_use_sim_time': sim,
                  'clock': clock[0], 'odom': odom.summary(clock[0]),
                  'edges': {k: v.summary(clock[0]) for k, v in edges.items()},
                  'multiple_parents': {k: sorted(v) for k, v in parents.items() if len(v) > 1},
                  'scan': scan_meta,
                  'scan_rate_hz': (len(scan_stamps)-1)/(scan_stamps[-1]-scan_stamps[0])
                      if len(scan_stamps) > 1 and scan_stamps[-1] > scan_stamps[0] else None,
                  'scan_age_max_sec': max(scan_ages, default=None),
                  'imu_z_max_abs': max((abs(x) for x in imu_z), default=None),
                  'odom_tf_yaw_error_max_deg': max((v['yaw_error_deg'] for v in agreement), default=None),
                  'odom_tf_xy_error_max_m': max((v['xy_error_m'] for v in agreement), default=None),
                  'publishers': publishers,
                  'note': 'TF 발행 노드 목록만으로 각 연결의 중복 발행 여부를 확정할 수 없습니다. Lua include 파일의 재정의는 직접 확인이 필요합니다.'}
        path = Path.home()/'ros2_practice_ws'/'slam_rotation_report.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print('Cartographer 프로세스 수:', len(configs), '/ use_sim_time:', sim)
        for item in configs:
            print('실행 설정:', item.get('path', item))
            print(json.dumps(item.get('settings', {}), ensure_ascii=False))
        print('서로 다른 부모가 관측된 TF:', report['multiple_parents'])
        for key in ['map -> odom', 'odom -> base_footprint', 'odom -> base_link']:
            if key in report['edges']:
                s = report['edges'][key]
                print(f"{key}: 최대 1회 변화={s['max_step_m']:.3f}m/{s['max_step_deg']:.2f}도, "
                      f"누적 회전={s['total_yaw_deg']:.1f}도, 부호 있는 회전={s['signed_yaw_deg']:.1f}도, 시각차={s['clock_age_sec']}")
        print('scan 주기 정보:', scan_meta, '/ 관측 Hz:', report['scan_rate_hz'])
        print('odom/TF 각도차 최댓값:', report['odom_tf_yaw_error_max_deg'])
        print('TF 발행 노드:', publishers['/tf'])
        print('상세 결과 저장:', path)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
