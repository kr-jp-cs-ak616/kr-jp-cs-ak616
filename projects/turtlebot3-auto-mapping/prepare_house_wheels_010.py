"""설치된 표준 Gazebo Waffle Pi를 보존하고 0.10m/s 매핑용 바퀴 구동 설정을 생성한다."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
from wheel_geometry_audit import audit_model


def wheel_profile(text):
    audit = audit_model(text)
    if audit['model'] != 'turtlebot3_waffle_pi':
        raise ValueError('표준 turtlebot3_waffle_pi 모델이 필요합니다.')
    if abs(audit['geometry_separation_m']-.288) > .0001 or any(
            abs(audit['wheels'][s]['radius']-.033) > .0001 for s in ('left', 'right')):
        raise ValueError('바퀴 형상이 표준 지름 0.066m, 중심 거리 0.288m와 다릅니다.')
    if audit['odometry_source'] != 1:
        raise ValueError('현재 프로필은 기존 WORLD 오도메트리 시뮬레이션에 적용합니다.')
    root = ET.fromstring(text)
    model = root if root.tag == 'model' else root.find('model')
    body = model.find("link[@name='base_link']/collision[@name='base_collision']")
    if body is None or body.findtext('geometry/box/size') is None:
        raise ValueError('표준 몸체 상자 형상을 찾지 못했습니다.')
    size = list(map(float, body.findtext('geometry/box/size').split()))
    pose = list(map(float, body.findtext('pose', '0 0 0 0 0 0').split()))
    if len(size) != 3 or len(pose) != 6 or max(abs(a-b) for a, b in zip(size, (.265, .265, .089))) > .0001 or abs(pose[0]+.064) > .0001 or any(abs(pose[i]) > .0001 for i in (1, 3, 4, 5)):
        raise ValueError('몸체 형상이 이번 사각 윤곽의 기준과 다릅니다.')
    plugin = next(p for p in model.findall('plugin') if 'gazebo_ros_diff_drive' in p.get('filename', ''))
    changes = dict(wheel_separation=.288, wheel_diameter=.066,
                   max_wheel_acceleration=.20, odometry_source=1)
    for name, value in changes.items():
        element = plugin.find(name)
        if element is None:
            element = ET.SubElement(plugin, name)
        element.text = str(value)
    # 토크·접촉·마찰·관성·스캔 시각·TF 설정은 원본 그대로 둔다.
    speed, yaw, radius, separation = .10, .20, .033, .288
    report = dict(source_audit=audit, changes=changes, speed_m_s=speed, yaw_rad_s=yaw,
                  straight_wheel_rad_s=speed/radius,
                  outer_wheel_rad_s=(speed+separation*yaw/2)/radius,
                  outer_wheel_rpm=(speed+separation*yaw/2)/radius*60/(2*math.pi),
                  commanded_wheel_acceleration_bound_m_s2=.10+separation*.30/2,
                  note='속도와 가속도는 이번 매핑의 운용값입니다. 제조사 최대 토크·최대 가속도 제원으로 해석하지 마세요.')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode'), report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdf', type=Path)
    args = parser.parse_args()
    if args.sdf is None:
        from ament_index_python.packages import get_package_share_directory
        args.sdf = Path(get_package_share_directory('turtlebot3_gazebo'))/'models/turtlebot3_waffle_pi/model.sdf'
    source = args.sdf.expanduser().resolve()
    text = source.read_text(encoding='utf-8-sig')
    modified, report = wheel_profile(text)
    target = Path.home()/'ros2_practice_ws/house_waffle_pi_010'
    if source.parent == target.resolve() or target.resolve().is_relative_to(source.parent):
        raise ValueError('출력 폴더 밖의 설치 원본 SDF를 사용하세요.')
    if target.exists():
        backup = target.with_name(target.name+'_backup_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        shutil.copytree(target, backup)
        print('기존 생성 모델 백업:', backup)
    shutil.copytree(source.parent, target, dirs_exist_ok=True)
    (target/'model.sdf').write_text(modified, encoding='utf-8')
    report.update(source=str(source), output=str(target/'model.sdf'),
                  source_sha256=hashlib.sha256(text.encode()).hexdigest(),
                  output_sha256=hashlib.sha256(modified.encode()).hexdigest())
    (target/'wheel_profile.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('새 스폰 모델:', target/'model.sdf')
    print('바퀴 지름 0.066m / 축간 0.288m / 바퀴 접선 가속도 상한 0.20m/s² / 기존 WORLD odom 유지')
    print('직진 바퀴 속도:', round(report['straight_wheel_rad_s'], 3), 'rad/s')
    print('새 Gazebo 세션은 house_waffle_pi_010.launch.py로 시작하세요. 원본 설치 모델은 보존했습니다.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, ET.ParseError) as error:
        raise SystemExit('바퀴 설정 생성 중단: '+str(error))
