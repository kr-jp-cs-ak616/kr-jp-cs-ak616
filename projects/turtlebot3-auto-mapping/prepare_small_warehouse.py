"""AWS 소형 창고 원본 자산을 검사하고 로컬 Gazebo 실행 경로를 기록한다."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote
import xml.etree.ElementTree as ET


def inspect_assets(source):
    source = Path(source).expanduser().resolve()
    world = source/'worlds/small_warehouse/small_warehouse.world'
    models = source/'models'
    if not world.is_file() or not models.is_dir():
        raise ValueError('ros2 분기의 worlds/small_warehouse/small_warehouse.world와 models 폴더가 필요합니다.')
    root = ET.parse(world).getroot()
    if root.find('world') is None:
        raise ValueError('SDF world 요소가 없습니다.')
    missing, model_names = set(), set()
    files = [world]+sorted(models.rglob('*.sdf'))
    for path in files:
        parsed = ET.parse(path).getroot()
        for node in parsed.iter('uri'):
            uri = (node.text or '').strip()
            target = None
            if uri.startswith('model://'):
                suffix = unquote(uri[len('model://'):])
                model_names.add(suffix.split('/')[0])
                target = models/suffix
                if '/' not in suffix and target.is_dir() and not (target/'model.config').is_file():
                    missing.add(str(target/'model.config'))
            elif uri.startswith('file://'):
                suffix = unquote(uri[len('file://'):])
                target = source/suffix if suffix.startswith('models/') else path.parent/suffix
            elif uri and not re.match(r'^[a-zA-Z]+://', uri):
                target = path.parent/unquote(uri)
            elif uri:
                raise ValueError('로컬에서 해석할 수 없는 URI: '+uri)
            if target is not None:
                resolved = target.resolve()
                if not resolved.is_relative_to(source):
                    raise ValueError('자산 폴더 밖을 참조합니다: '+uri)
                if not resolved.exists():
                    missing.add(str(resolved.relative_to(source)))
    if missing:
        raise ValueError('누락된 모델/메시: '+', '.join(sorted(missing)[:12]))
    try:
        result = subprocess.run(['git', '-C', str(source), 'rev-parse', 'HEAD'],
                                capture_output=True, text=True, timeout=10, check=False)
        commit = result.stdout.strip() if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        commit = None
    return dict(source=str(source), world=str(world), models=str(models),
                source_url='https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros2',
                source_commit=commit, world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                sdf_files_checked=len(files), model_names=sorted(model_names),
                spawn=[0., 0., .01, 0.],
                note='참조 파일 존재·XML 검사입니다. 스폰 위치의 실제 충돌이나 Gazebo 렌더링 검증은 아닙니다.')


def create_roofless_world(original, target):
    """AWS RoofB 인스턴스만 제외한 실행용 월드를 생성한다."""
    original, target = Path(original).resolve(), Path(target).resolve()
    if original == target:
        raise ValueError('원본 월드를 덮어쓸 수 없습니다.')
    tree = ET.parse(original)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('SDF world 요소가 없습니다.')
    removed = []
    for child in list(world):
        if child.tag not in ('model', 'include'):
            continue
        if any((uri.text or '').strip() == 'model://aws_robomaker_warehouse_RoofB_01'
               for uri in child.iter('uri')):
            removed.append(child.get('name') or child.findtext('name') or 'RoofB_01')
            world.remove(child)
    if not removed:
        raise ValueError('예상한 AWS RoofB 지붕을 찾지 못했습니다. 원본 월드를 확인하세요.')
    target.parent.mkdir(parents=True, exist_ok=True)
    tree.write(target, encoding='utf-8', xml_declaration=True)
    return removed


def apply_vm_lite(world_path):
    """파생 월드의 소품과 그림자를 줄이고 실시간 목표를 0.5배로 제한한다."""
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('SDF world 요소가 없습니다.')
    prefixes = ('model://aws_robomaker_warehouse_Cluttering',
                'model://aws_robomaker_warehouse_Bucket_',
                'model://aws_robomaker_warehouse_TrashCan',
                'model://aws_robomaker_warehouse_PalletJack',
                'model://aws_robomaker_warehouse_Lamp_')
    removed = []
    for child in list(world):
        if child.tag in ('model', 'include') and any(
                (uri.text or '').strip().startswith(prefixes) for uri in child.iter('uri')):
            removed.append(child.get('name') or child.findtext('name') or 'prop')
            world.remove(child)
    def set_text(parent, tag, value):
        element = parent.find(tag)
        if element is None:
            element = ET.SubElement(parent, tag)
        element.text = value
    scene = world.find('scene')
    if scene is None:
        scene = ET.SubElement(world, 'scene')
    set_text(scene, 'shadows', 'false')
    for light in world.findall('light'):
        set_text(light, 'cast_shadows', 'false')
    physics = world.find('physics')
    if physics is None:
        raise ValueError('physics 설정이 없습니다.')
    # 적분 간격은 유지한다. 0.001초 × 500Hz = 실시간 대비 최대 0.5배.
    set_text(physics, 'max_step_size', '0.001')
    set_text(physics, 'real_time_update_rate', '500')
    set_text(physics, 'real_time_factor', '0.5')
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    return removed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=Path.home()/'ros2_practice_ws/assets/aws-robomaker-small-warehouse-world')
    parser.add_argument('--vm-lite', action='store_true', help='소품 제외·그림자 끄기·실시간 목표 0.5배')
    args = parser.parse_args()
    report = inspect_assets(args.source)
    target = Path.home()/'ros2_practice_ws/warehouse_setup.json'
    original = Path(report['world'])
    filename = 'small_warehouse_vm_lite.world' if args.vm_lite else 'small_warehouse_no_roof.world'
    roofless = target.parent/'warehouse_worlds'/filename
    removed = create_roofless_world(original, roofless)
    props_removed = apply_vm_lite(roofless) if args.vm_lite else []
    report.update(source_world=str(original), source_world_sha256=report['world_sha256'],
                  world=str(roofless), world_sha256=hashlib.sha256(roofless.read_bytes()).hexdigest(),
                  roof_removed=removed, vm_lite=args.vm_lite, props_removed=props_removed)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('소형 창고 검사 완료:', report['world'])
    print('제외한 지붕:', ', '.join(removed))
    if args.vm_lite:
        print('VM 경량 모드: 소품', len(props_removed), '개 제외, 그림자 끄기, 실시간 목표 0.5배')
    print('검사한 SDF 수:', report['sdf_files_checked'], '/ 원본 커밋:', report['source_commit'])
    print('실행 경로 기록:', target)
    print('이제 ros2 launch ~/ros2_practice_ws/small_warehouse_waffle_pi.launch.py 로 실행하세요.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, ET.ParseError) as error:
        raise SystemExit('창고 준비 중단: '+str(error))
