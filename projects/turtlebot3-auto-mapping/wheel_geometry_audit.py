"""Gazebo Waffle Pi의 바퀴 기하와 주행 오도메트리를 비교한다. 주행 명령은 보내지 않는다."""
import argparse
import bisect
import json
import math
from pathlib import Path
import statistics
import time
import xml.etree.ElementTree as ET


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose6(element):
    if element is None:
        return (0.,)*6
    if element.get('relative_to'):
        raise ValueError('relative_to가 있는 SDF는 자동 해석하지 않습니다.')
    values = tuple(map(float, element.text.split()))
    if len(values) != 6:
        raise ValueError('SDF pose는 6개 값이어야 합니다.')
    return values


def point_in_model(link, collision):
    p = pose6(link.find('pose'))
    c = pose6(collision.find('pose'))
    cr, sr, cp, sp, cy, sy = math.cos(p[3]), math.sin(p[3]), math.cos(p[4]), math.sin(p[4]), math.cos(p[5]), math.sin(p[5])
    matrix = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
              (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))
    return tuple(p[i]+sum(matrix[i][j]*c[j] for j in range(3)) for i in range(3))


def audit_model(text):
    root = ET.fromstring(text)
    model = root if root.tag == 'model' else root.find('model')
    if model is None:
        raise ValueError('최상위 model이 없는 SDF입니다.')
    plugins = [p for p in model.findall('plugin') if 'gazebo_ros_diff_drive' in p.get('filename', '')]
    if len(plugins) != 1:
        raise ValueError('차동구동 플러그인이 정확히 하나여야 합니다.')
    plugin = plugins[0]
    wheels = {}
    for side in ('left', 'right'):
        joint_name = plugin.findtext(side+'_joint')
        joint = next((j for j in model.findall('joint') if j.get('name') == joint_name), None)
        if joint is None:
            raise ValueError('바퀴 조인트를 찾지 못했습니다: '+str(joint_name))
        link = next((l for l in model.findall('link') if l.get('name') == joint.findtext('child')), None)
        if link is None:
            raise ValueError('바퀴 링크를 찾지 못했습니다.')
        candidates = [c for c in link.findall('collision') if c.find('geometry/cylinder/radius') is not None]
        if len(candidates) != 1:
            raise ValueError('바퀴 충돌 형상이 단일 원통이 아니므로 수동 확인이 필요합니다.')
        collision = candidates[0]
        wheels[side] = {'joint': joint_name, 'center': point_in_model(link, collision),
                        'radius': float(collision.findtext('geometry/cylinder/radius'))}
    left, right = wheels['left']['center'], wheels['right']['center']
    # 지면상의 중심 거리. 바퀴 축 방향과 타이어 변형에 따른 유효 거리는 별도 측정 대상이다.
    separation = math.hypot(left[0]-right[0], left[1]-right[1])
    configured = float(plugin.findtext('wheel_separation', '0.34'))
    if min(separation, configured, wheels['left']['radius'], wheels['right']['radius']) <= 0:
        raise ValueError('바퀴 치수는 양수여야 합니다.')
    source = int(plugin.findtext('odometry_source', '1'))
    return {'model': model.get('name'), 'wheels': wheels, 'geometry_separation_m': separation,
            'configured_separation_m': configured,
            'separation_difference_percent': 100*(separation/configured-1),
            'configured_diameter_m': float(plugin.findtext('wheel_diameter', '0.15')),
            'odometry_source': source,
            'odometry_source_label': {0: 'ENCODER', 1: 'WORLD'}.get(source, 'UNKNOWN'),
            'source_explicit': plugin.find('odometry_source') is not None,
            'axle_center_model': [(left[i]+right[i])/2 for i in range(3)]}


def fit_geometry(segments):
    """좌우 회전각·기준 전진거리·기준 회전각으로 rL,rR,B를 최소제곱 추정한다."""
    if len(segments) < 8:
        raise ValueError('유효 주행 구간이 8개 미만입니다.')
    if sum(abs(s[2]) for s in segments) < .5 or sum(abs(s[3]) for s in segments) < .5:
        raise ValueError('전진거리 0.5m와 회전량 0.5rad 이상의 자료가 필요합니다.')
    rows, targets = [], []
    for dl, dr, distance, yaw in segments:
        rows.extend(((dl/2, dr/2, 0), (dl, -dr, yaw)))
        targets.extend((distance, 0))
    scales = [math.sqrt(sum(r[j]**2 for r in rows)) for j in range(3)]
    if min(scales) < 1e-8:
        raise ValueError('바퀴 회전 또는 기준 회전 자료가 부족합니다.')
    normalized = [[r[j]/scales[j] for j in range(3)] for r in rows]
    matrix = [[sum(r[i]*r[j] for r in normalized) for j in range(3)] +
              [sum(r[i]*b for r, b in zip(normalized, targets))] for i in range(3)]
    for i in range(3):
        pivot = max(range(i, 3), key=lambda k: abs(matrix[k][i]))
        matrix[i], matrix[pivot] = matrix[pivot], matrix[i]
        divisor = matrix[i][i]
        if abs(divisor) < 1e-5:
            raise ValueError('한 가지 곡률만으로는 치수를 구분할 수 없습니다. 직진과 좌우 회전이 필요합니다.')
        matrix[i] = [v/divisor for v in matrix[i]]
        for k in range(3):
            if k != i:
                factor = matrix[k][i]
                matrix[k] = [a-factor*b for a, b in zip(matrix[k], matrix[i])]
    rl, rr, width = [matrix[i][3]/scales[i] for i in range(3)]
    if not .015 < min(rl, rr) <= max(rl, rr) < .06 or not .15 < width < .5:
        raise ValueError('추정 치수가 Waffle Pi의 합리적 시험 범위를 벗어났습니다. 적용하지 마세요.')
    distance_errors = [(rl*dl+rr*dr)/2-ds for dl, dr, ds, yaw in segments]
    angle_errors = [(rr*dr-rl*dl)/width-yaw for dl, dr, ds, yaw in segments]
    return {'left_radius_m': rl, 'right_radius_m': rr, 'effective_separation_m': width,
            'distance_rmse_m': math.sqrt(statistics.mean(e*e for e in distance_errors)),
            'yaw_rmse_deg': math.degrees(math.sqrt(statistics.mean(e*e for e in angle_errors))),
            'segments': len(segments), 'applied': False}


def assess_estimate(estimate, model, segments):
    """보수적인 시험용 선별 기준. 접촉 검출이나 보정 승인 기준을 대신하지 않는다."""
    reasons = []
    groups = {'straight': 0, 'left_spin': 0, 'right_spin': 0, 'arc': 0}
    for dl, dr, ds, yaw in segments:
        denominator = max(abs(dl)+abs(dr), 1e-8)
        if dl*dr > 0 and abs(dl-dr)/denominator < .15:
            groups['straight'] += 1
        elif abs(dl+dr)/denominator < .3:
            groups['left_spin' if dr > dl else 'right_spin'] += 1
        else:
            groups['arc'] += 1
    if groups['straight'] < 5 or groups['left_spin'] < 3 or groups['right_spin'] < 3:
        reasons.append('직진 및 좌우 제자리 회전 구간이 충분하지 않습니다.')
    if estimate['distance_rmse_m'] > .005 or estimate['yaw_rmse_deg'] > 2:
        reasons.append('거리 또는 회전 잔차가 시험용 선별 기준(5mm, 2도)을 초과했습니다.')
    for side in ('left', 'right'):
        nominal = model['wheels'][side]['radius']
        if abs(estimate[side+'_radius_m']/nominal-1) > .1:
            reasons.append(side+' 반경이 형상 반경과 10% 이상 다릅니다.')
    if abs(estimate['effective_separation_m']/model['geometry_separation_m']-1) > .1:
        reasons.append('유효 축간거리가 형상 거리와 10% 이상 다릅니다.')
    return {'screening_pass': not reasons, 'reasons': reasons, 'motion_groups': groups,
            'note': '시험용 선별 기준이며 통과해도 접촉 없음·반복 측정·기준 모델 확인이 필요합니다.'}


def interpolate_odom(samples, t):
    stamps = [s[0] for s in samples]
    i = bisect.bisect_left(stamps, t)
    if i == 0 or i >= len(samples):
        return None
    a, b = samples[i-1], samples[i]
    if b[0]-a[0] > .15 or b[0] <= a[0]:
        return None
    f = (t-a[0])/(b[0]-a[0])
    return (a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2]), a[3]+f*wrap(b[3]-a[3]))


def collect(model, duration):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = Node('wheel_geometry_audit')
    odom, joints = [], []
    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        odom.append((m.header.stamp.sec+m.header.stamp.nanosec/1e9, p.x, p.y, yaw))
    def on_joint(m):
        try:
            indices = [m.name.index(model['wheels'][s]['joint']) for s in ('left', 'right')]
            joints.append((m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                           m.position[indices[0]], m.position[indices[1]]))
        except (ValueError, IndexError):
            pass
    subscriptions = [node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data),
                     node.create_subscription(JointState, '/joint_states', on_joint, qos_profile_sensor_data)]
    try:
        print(f'{duration:.0f}초 수동 관측합니다. 이 프로그램은 로봇을 움직이지 않습니다.', flush=True)
        deadline = time.monotonic()+duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        report = {'odom_messages': len(odom), 'joint_messages': len(joints), 'estimate': None}
        raw_path = Path.home()/'ros2_practice_ws'/('wheel_geometry_samples_'+str(time.time_ns())+'.json')
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        raw_path.write_text(json.dumps({'model': model, 'odom': odom, 'joints': joints},
                                       ensure_ascii=False), encoding='utf-8')
        report['raw_samples_path'] = str(raw_path)
        if model['odometry_source'] != 1:
            report['reason'] = 'ENCODER 또는 불명확한 odom을 기준으로 같은 엔코더 치수를 보정하지 않습니다. 외부 거리·각도 기준이 필요합니다.'
            return report
        if len(odom) < 20 or len(joints) < 20:
            report['reason'] = '센서 수신 부족: /odom, /joint_states, DDS 및 Gazebo 재생 상태를 확인하세요.'
            return report
        unwrapped, previous = [], None
        left = right = 0.
        for sample in joints:
            if previous:
                if sample[0] <= previous[0] or sample[0]-previous[0] > .15:
                    report['reason'] = '바퀴 센서의 시각 역행 또는 수신 공백이 있어 추정하지 않습니다.'
                    return report
                left += wrap(sample[1]-previous[1])
                right += wrap(sample[2]-previous[2])
            unwrapped.append((sample[0], left, right))
            previous = sample
        if any(b[0] <= a[0] for a, b in zip(odom, odom[1:])):
            report['reason'] = '오도메트리 시각이 역행하거나 중복되었습니다.'
            return report
        segments, first = [], unwrapped[0]
        for last in unwrapped[1:]:
            if last[0]-first[0] < .8:
                continue
            a, b = interpolate_odom(odom, first[0]), interpolate_odom(odom, last[0])
            if a and b:
                angle = wrap(b[2]-a[2])
                heading = a[2]+angle/2
                ds = (b[0]-a[0])*math.cos(heading)+(b[1]-a[1])*math.sin(heading)
                # 모델 좌표의 축 중심과 odom 기준 위치가 XY에서 같은 기본 모델만 추정한다.
                if abs(angle) < .5 and (abs(ds) > .005 or abs(angle) > .01):
                    chord_factor = angle/(2*math.sin(angle/2)) if abs(angle) > 1e-6 else 1
                    segments.append((last[1]-first[1], last[2]-first[2], ds*chord_factor, angle))
            first = last
        if math.hypot(*model['axle_center_model'][:2]) > .001:
            report['reason'] = '모델 원점과 바퀴축 중심이 다릅니다. odom 기준점 변환 확인 후 보정해야 합니다.'
            return report
        try:
            report['estimate'] = fit_geometry(segments)
            report['quality'] = assess_estimate(report['estimate'], model, segments)
        except ValueError as exc:
            report['reason'] = str(exc)
        report['reference_note'] = '설치 SDF의 WORLD 설정을 기준으로 한 후보 추정입니다. 실제 스폰 모델과 odom 발행원이 같은지 확인해야 합니다.'
        return report
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdf', type=Path)
    parser.add_argument('--observe', type=float, default=0)
    args = parser.parse_args()
    if not 0 <= args.observe <= 60:
        parser.error('--observe는 0~60초여야 합니다.')
    if args.sdf is None:
        from ament_index_python.packages import get_package_share_directory
        args.sdf = Path(get_package_share_directory('turtlebot3_gazebo'))/'models/turtlebot3_waffle_pi/model.sdf'
    report = audit_model(args.sdf.read_text())
    report['source_path'] = str(args.sdf)
    report['runtime_model_verified'] = False
    print('설치 모델 후보:', args.sdf)
    print(f"바퀴 중심 거리: 형상={report['geometry_separation_m']:.6f}m / 설정={report['configured_separation_m']:.6f}m / 차이={report['separation_difference_percent']:.3f}%")
    print(f"반경: 좌={report['wheels']['left']['radius']:.6f}m / 우={report['wheels']['right']['radius']:.6f}m / 설정 지름={report['configured_diameter_m']:.6f}m")
    print('odom source:', report['odometry_source_label'], '/ 명시 설정:', report['source_explicit'])
    print('주의: 현재 스폰된 모델과 설치 SDF의 일치는 아직 확인하지 않았습니다.')
    if args.observe:
        report['observation'] = collect(report, args.observe)
        print(json.dumps(report['observation'], ensure_ascii=False, indent=2))
    path = Path.home()/'ros2_practice_ws'/'wheel_geometry_report.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('저장:', path)
    print('구동 설정과 TF는 변경하지 않았습니다.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, ET.ParseError) as exc:
        raise SystemExit('바퀴 검사 중단: '+str(exc))
