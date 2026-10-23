# 파일 보관 기준

정리 대상은 현재 Windows 작업 폴더입니다. Ubuntu 가상머신, Downloads와 첨부 원본, Git 내부 자료는 변경하지 않았습니다.

## 남긴 자료

| 위치 | 보관 이유 |
|---|---|
| 루트의 실행·설정 Python과 house_env_cyclone.bash | 최신 0.10m/s Nav2 탐색을 재현하는 코드와 의존 파일 |
| 루트의 test_*.py 7개 | 현재 알고리즘·설정·바퀴 기구학·주행 진단·단일 목표·막힘 재관측·창고 자산 검사 |
| testdata/ | 시작 여유 부족과 모서리 추종 문제의 실제 지도 자료 |
| docs/guides/ | 현재 실행과 위치 추정 안내 |
| docs/history/ | TF 시각 고정·바퀴 기구학·SLAM 비교의 핵심 분석 원문 |
| docs/evidence/ | TF 시각 문제의 실제 관측 보고서 |
| docs/DEVELOPMENT_JOURNEY.md | 초기 성공부터 설계 전환까지의 성장 기록 |
| docs/PORTFOLIO.md | 공개 가능한 설명과 성공 후 추가할 증거 목록 |
| tools/diagnostics/ | 증거를 수집했던 진단 도구 |
| tools/package_release.py | 최신 배포 ZIP 재생성 |
| house_waffle_pi_010_bundle.zip | 우분투 복사용 최신 배포본 하나. Git 추적 대상에서는 제외 |
| prepare_small_warehouse.py, small_warehouse_waffle_pi.launch.py | AWS 소형 창고 자산 검사와 기존 Waffle Pi 프로필 연결 |
| small_warehouse_waffle_pi_bundle.zip | 소형 창고용 배포 진입점. 외부 자산 제외, Git 추적 제외 |
| .git/ | 기존 Git 메타데이터 보존 |

## 삭제한 자료

중복된 구버전 ZIP·압축 해제본, Python 캐시, 현재 실행 경로에 쓰이지 않는 벽 추종·문/방 탐색 코드와 그 전용 테스트, 대체된 중간 설정 생성기, 이전 Fast DDS 환경, 반복되는 옛 실행 안내서를 작업 폴더에서 제거해 Windows 휴지통으로 보냈습니다. 영구 삭제하지 않았습니다. 의미 있는 시행착오는 개발 과정 문서에 요약했습니다.

옛 코드를 모두 보관하는 대신 현재 코드의 재현성과 중요한 판단의 증거를 남기는 기준입니다. 삭제한 파일 이름·크기·SHA-256과 옮긴 경로는 cleanup_manifest.json에 기록했습니다. 이 목록은 삭제한 내용의 백업 파일은 아닙니다.

현재 배포 프로필은 House 최종 성공을 아직 검증하지 않았습니다. 이전 자료의 실행 명령은 해당 시점의 분석 기록이며, 현재 실행에는 docs/guides의 최신 0.10m/s 안내서를 사용하세요.


VM용 단순 창고: `prepare_simple_warehouse.py`, `simple_warehouse_waffle_pi.launch.py`, `test_simple_warehouse.py`, `docs/guides/README_단순창고_실행.md`. 배포 파일은 `simple_warehouse_waffle_pi_bundle.zip`입니다. AWS 자산 없이 정적 상자로 생성하는 별도 맵입니다.


지도 표현: `map_line_refiner.py`, `test_map_line_refiner.py`, `docs/guides/README_지도직선화_고정지도.md`. `map_line_refiner_bundle.zip`은 실행 노드·기존 위치추정 설정 생성기·안내서 4개로 구성합니다. 원본 점유 지도는 변경하지 않습니다.


현재 기준 실행: `docs/guides/README_현재진행_통합실행.md`. 상태 기록: `docs/evidence/진행상태_2026-10-23.md`, `progress_2026-10-23.json`, `2026-10-23_slam_distortion.png`. 통합 배포: `current_simple_warehouse_bundle.zip` (15개).
