"""최신 우분투 복사용 배포 파일만 모아 ZIP 하나를 생성한다."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import argparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--warehouse', action='store_true')
    parser.add_argument('--simple-warehouse', action='store_true')
    parser.add_argument('--map-lines', action='store_true')
    parser.add_argument('--current', action='store_true', help='현재 단순 창고와 지도 표시·진단·통합 안내')
    args = parser.parse_args()
    if sum((args.warehouse, args.simple_warehouse, args.map_lines, args.current)) > 1:
        parser.error('창고 배포 옵션은 하나만 지정하세요.')
    root = Path(__file__).resolve().parents[1]
    files = [
        'house_env_cyclone.bash', 'prepare_house_nav2_frontier.py',
        'prepare_house_wheels_010.py', 'house_waffle_pi_010.launch.py',
        'wheel_geometry_audit.py', 'house_nav2_frontier.py',
        'prepare_house_revisit.py', 'prepare_house_slam_ab.py',
        'tools/diagnostics/diagnose_house_motion.py',
        'docs/guides/README_House_010_초기화실행.md',
    ]
    if args.warehouse or args.simple_warehouse or args.current:
        files = [f for f in files if f not in ('house_waffle_pi_010.launch.py',
                 'docs/guides/README_House_010_초기화실행.md')]
        if args.simple_warehouse or args.current:
            files += ['prepare_simple_warehouse.py', 'simple_warehouse_waffle_pi.launch.py',
                      'docs/guides/README_단순창고_실행.md']
        else:
            files += ['prepare_small_warehouse.py', 'small_warehouse_waffle_pi.launch.py',
                      'docs/guides/README_소형창고_실행.md']
    if args.map_lines:
        files = ['map_line_refiner.py', 'prepare_house_revisit.py', 'prepare_house_slam_ab.py',
                 'docs/guides/README_지도직선화_고정지도.md']
    if args.current:
        files += ['map_line_refiner.py', 'tools/diagnostics/inspect_slam_rotation.py',
                  'docs/guides/README_현재진행_통합실행.md',
                  'docs/guides/README_지도직선화_고정지도.md']
    for name in files:
        if not (root/name).is_file():
            raise FileNotFoundError(name)
    filename = ('current_simple_warehouse_bundle.zip' if args.current else
                'map_line_refiner_bundle.zip' if args.map_lines else
                'simple_warehouse_waffle_pi_bundle.zip' if args.simple_warehouse else
                'small_warehouse_waffle_pi_bundle.zip' if args.warehouse else 'house_waffle_pi_010_bundle.zip')
    target = root/filename
    with ZipFile(target, 'w', compression=ZIP_DEFLATED) as archive:
        for name in files:
            archive.write(root/name, Path(name).name)
    print(target)


if __name__ == '__main__':
    main()
