"""현재 Cartographer 설정을 보존하고 전체 최적화만 끄는 비교 설정을 만든다."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil


def local_only_lua(text):
    pattern = r'(?m)^(\s*return\s+options\s*;?\s*(?:--[^\n]*)?)$'
    matches = list(re.finditer(pattern, text))
    if len(matches) != 1:
        raise ValueError('최상위 return options가 정확히 하나인 Lua만 자동 처리합니다.')
    match = matches[0]
    addition = ('\n-- 원인 분리 시험: 지도 전체 최적화만 중지합니다.\n'
                'POSE_GRAPH.optimize_every_n_nodes = 0\n')
    return text[:match.start()]+addition+text[match.start():]


def find_active_source():
    results = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            args = (entry/'cmdline').read_bytes().decode(errors='replace').strip('\x00').split('\x00')
        except OSError:
            continue
        if not args or Path(args[0]).name != 'cartographer_node':
            continue
        try:
            directory = args[args.index('-configuration_directory')+1]
            filename = args[args.index('-configuration_basename')+1]
        except (ValueError, IndexError):
            raise ValueError('Cartographer 실행 인자에서 설정 경로를 찾지 못했습니다.')
        source = Path(directory)/filename
        if not source.is_absolute():
            source = (entry/'cwd').resolve()/source
        results.append(source.resolve())
    if len(results) != 1:
        raise ValueError(f'실행 중인 Cartographer가 {len(results)}개입니다. 1개를 유지하거나 --source로 Lua 경로를 지정하세요.')
    return results[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path)
    args = parser.parse_args()
    source = args.source.expanduser().resolve() if args.source else find_active_source()
    text = source.read_text(encoding='utf-8')
    modified = local_only_lua(text)
    target = Path.home()/'ros2_practice_ws'/'house_slam_ab_config'
    if source.name in ('house_baseline.lua', 'house_local_only.lua'):
        raise ValueError('기존 비교용 파일이 아닌 원래 설정을 --source로 지정하세요.')
    if target.resolve() == source.parent.resolve() or target.resolve().is_relative_to(source.parent.resolve()):
        raise ValueError('출력 폴더가 원본 설정 폴더 안에 있습니다. 원래 설치 Lua를 --source로 지정하세요.')
    if target.exists():
        backup = target.with_name(target.name+'_backup_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        shutil.copytree(target, backup)
        print('기존 시험 설정 백업:', backup)
    shutil.copytree(source.parent, target, dirs_exist_ok=True)
    baseline = target/'house_baseline.lua'
    local = target/'house_local_only.lua'
    baseline.write_text(text, encoding='utf-8')
    local.write_text(modified, encoding='utf-8')
    report = {'source': str(source), 'source_sha256': hashlib.sha256(text.encode()).hexdigest(),
              'baseline': str(baseline), 'local_only': str(local),
              'change': 'POSE_GRAPH.optimize_every_n_nodes = 0',
              'note': 'IMU·프레임·주행·스캔 정합 가중치는 원본 값을 유지합니다.'}
    (target/'comparison_source.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('기준 설정:', source)
    print('원본 보존:', baseline)
    print('변경한 값: POSE_GRAPH.optimize_every_n_nodes = 0')
    print('기존 Cartographer 종료 후 새 지도로 실행하세요:')
    print('ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true '
          'cartographer_config_dir:='+str(target)+' configuration_basename:=house_local_only.lua')
    print('비교용 설정입니다. 완료된 지도용 설정으로 확정한 것이 아닙니다.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError) as error:
        raise SystemExit('비교 설정 생성 중단: '+str(error))
