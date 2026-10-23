# 현재 진행과 실행 순서 — 2026-10-23

앞으로 현재 작업은 이 문서를 기준으로 진행합니다. 예전 House·AWS 창고·경량 AWS 창고 안내는 비교 기록입니다. 현재 환경은 **지붕 없는 8×6m 단순 창고 + Waffle Pi + Cyclone DDS + ROS_DOMAIN_ID=30**입니다.

## 현재 어디까지 진행했나

- 단순 창고 실행과 매핑 화면을 사용자 첨부 화면으로 확인했습니다.
- 파란 `/map_lines` 직선 표시도 화면에서 확인했습니다.
- 원본 점유 지도에 벽·선반이 겹치거나 어긋나는 문제가 다시 보입니다. **정상 지도 완성·전체 탐색·출발점 복귀·AMCL 고정 지도 주행 성공은 아직 확인하지 못했습니다.**
- 원점에서 멀어지는 것이 원인이라고 확정할 수 없습니다. 현재 입력 시각·TF 발행·스캔 정합·최적화 상태를 함께 확인해야 합니다.
- 직선 표시는 기존 경계를 근사해서 보여 줍니다. 지도를 보정하거나 왜곡을 복구하지 않습니다. 파란 선이 반듯해도 SLAM 지도가 정상이라는 뜻은 아닙니다.
- 이전 motion 진단에는 정지 명령 43회·회전 4회·장시간 회전 10회·이동 2회의 분류가 있습니다. 전체 /map 격자와 이번 변형 구간의 센서 기록이 없어 지도 왜곡 원인을 확정할 수 없습니다. 이 집계는 지도 변형의 증거가 아닌 주행 상태 기록입니다.

**지금은 아래 0단계에서 증거를 저장하세요. 완성 지도용 AMCL 단계로 바로 넘어가지 않습니다.** 아래 A·B·C는 매번 모두 수행하는 절차가 아니라 목적이 다른 단계입니다.

| 목적 | Gazebo | Cartographer | Nav2 | AMCL | 자동 탐색 |
|---|---|---|---|---|---|
| A. 새 지도 만들기 | 켬 | 켬 | 켬 | 끔 | 켬 |
| B. 정상 지도 저장·모드 전환 | 유지 | 저장 후 종료 | 전환 중 종료 | 아직 끔 | 먼저 종료 |
| C. 저장 지도에서 주행 | 켬 | 끔 | 켬 | 켬 | 끔 |

우분투나 VM 전체를 껐다 켤 필요는 없습니다. A→B→C 전환에서는 **Gazebo를 유지하고 ROS 노드만 교체**합니다. Gazebo를 재시작하면 시뮬레이션 시각이 초기화되므로 연결된 SLAM·Nav2·탐색·표시 노드도 종료 후 새로 시작합니다.

## 0. 지금 할 일: 변형 상태 보존

1. 터미널 4의 자동 탐색기만 Ctrl+C로 종료하여 이동을 멈춥니다. 키보드 조종 등 다른 속도 명령도 보내지 않습니다.
2. **Gazebo는 재생, Cartographer와 Nav2는 실행 상태로 잠시 유지**합니다. 아직 모두 종료하거나 지도를 초기화하지 않습니다.
3. 통합 ZIP의 파일 15개를 우분투 `~/ros2_practice_ws`에 복사합니다. 현재 실행 중인 프로그램에는 새 파일이 즉시 반영되지 않습니다.
4. 추가 터미널에서 아래 명령을 하나씩 실행합니다. 아래 지도는 **실패 사례 보존용**이며 완성 지도라고 사용하지 않습니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
mkdir -p ~/ros2_practice_ws/maps ~/ros2_practice_ws/diagnostics

ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/ros2_practice_ws/maps/simple_warehouse_problem_$(date +%Y%m%d_%H%M%S)" \
  --ros-args -p use_sim_time:=true

python3 ~/ros2_practice_ws/inspect_slam_rotation.py

python3 ~/ros2_practice_ws/diagnose_house_motion.py --seconds 60
```

`Map saved successfully`를 확인합니다. 두 진단 프로그램은 스스로 주행 명령을 보내지 않습니다. 정지 상태의 결과는 시간·TF·실행 설정 확인용이며 회전 중 센서 정합까지 증명하지 않습니다. `inspect_slam_rotation.py`의 결과는 `~/ros2_practice_ws/slam_rotation_report.json`, 주행 진단은 `~/ros2_practice_ws/diagnostics/motion_*.json`에 저장됩니다.

다음 매핑 시도에서 변형되는 순간의 원인을 보려면 별도 터미널에서 아래 기록을 **자동 탐색 시작 전** 켭니다. 완료·중단 시 Ctrl+C로 정상 종료하여 파일을 닫습니다. VM 부담을 줄이기 위해 카메라 데이터는 기록하지 않습니다. 기록도 부하를 만들 수 있습니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
mkdir -p ~/ros2_practice_ws/diagnostics
ros2 bag record \
  -o "$HOME/ros2_practice_ws/diagnostics/slam_$(date +%Y%m%d_%H%M%S)" \
  /clock /scan /odom /imu /tf /tf_static /map /cmd_vel /joint_states /rosout
```

없는 토픽은 수신될 때까지 대기할 수 있습니다. 기록을 실제 로봇 제어 노드들과 동시에 재생하지 않습니다. 파란 선만으로 왜곡 원인을 판단하거나 원본 odom을 덮어쓰지 않습니다. 지금 제공된 자료만으로 해결된 SLAM 설정을 확정하지 않았습니다.

## A. 새 지도 만들기 — 새로 매핑할 때만

현재 실패 사례를 저장한 뒤 새로운 매핑을 시작하려면 탐색기 → Nav2 → Cartographer/RViz → 직선 표시·기록기 → Gazebo를 종료합니다. 기존 지도 파일은 삭제하지 않습니다. 이번 새 세션에는 저장 지도를 로드하지 않습니다.

터미널 1 — 준비 후 Gazebo:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/prepare_simple_warehouse.py
python3 ~/ros2_practice_ws/prepare_house_wheels_010.py
python3 ~/ros2_practice_ws/prepare_house_nav2_frontier.py \
  --speed 0.10 --yaw-rate 0.20 --waffle-footprint
python3 ~/ros2_practice_ws/prepare_house_revisit.py --mode mapping \
  --source "$(ros2 pkg prefix --share turtlebot3_cartographer)/config/turtlebot3_lds_2d.lua"
ros2 launch ~/ros2_practice_ws/simple_warehouse_waffle_pi.launch.py gui:=true
```

준비 명령이 하나라도 실패하면 다음 명령을 계속 실행하지 않습니다. 준비는 설치 원본을 보존한 파생 파일을 만듭니다. 같은 설정으로 다시 실행할 때는 준비 결과를 재사용할 수 있습니다. 속도 0.10m/s, 회전 상한 0.20rad/s, 바퀴 지름 0.066m, 중심 거리 0.288m는 현재 프로필 값이며 왜곡 해결을 검증한 튜닝값이라는 뜻은 아닙니다.

터미널 2 — Cartographer와 RViz:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=true \
  cartographer_config_dir:=$HOME/ros2_practice_ws/house_revisit_config \
  configuration_basename:=house_revisit.lua
```

RViz Fixed Frame=map, Frame Rate=10으로 설정합니다. Gazebo는 재생 상태여야 합니다. A단계에서는 AMCL을 실행하지 않습니다. 지도·라이다 정렬을 먼저 확인합니다.

터미널 3 — Nav2:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_frontier.yaml
```

`Managed nodes are active`를 확인하고, 필요하다면 위 0단계의 rosbag 기록을 추가 터미널에서 시작합니다.

터미널 4 — 자동 탐색:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/house_nav2_frontier.py \
  --ros-args -p use_sim_time:=true
```

선택 사항인 터미널 5 — 직선 표시:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/map_line_refiner.py \
  --ros-args -p use_sim_time:=true
```

RViz Add → By topic → `/map_lines` → Marker를 추가합니다. 원본 Map도 표시해서 실제 왜곡을 확인합니다. 직선 파일은 `~/ros2_practice_ws/map_refined/map_lines.svg`와 `.json`에 갱신됩니다. 이 SVG를 Nav2 지도 대신 쓰지 않습니다.

진행 중 단순 일시 정지 경고가 나왔다고 매핑 완료로 판단하지 않습니다. 미탐색 공간·닫힌 통로·벽 정렬·탐색기 결과를 확인합니다. 왜곡이 다시 보이면 탐색을 중지하고 0단계처럼 문제 지도를 보존합니다. **왜곡된 상태를 고정 지도 모드로 바꿔도 왜곡은 복구되지 않습니다.**

## B. 정상 지도가 만들어진 뒤 저장·전환 — 한 번

현재는 정상 완성 확인 전이므로 이 단계는 나중에 수행합니다. 정상 지도와 완전 탐색을 확인한 뒤 터미널 4 탐색기를 종료하고, Cartographer가 아직 켜져 있을 때 저장합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
mkdir -p ~/ros2_practice_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_practice_ws/maps/simple_warehouse_fixed \
  --ros-args -p use_sim_time:=true
python3 ~/ros2_practice_ws/prepare_house_revisit.py --mode localization \
  --map ~/ros2_practice_ws/maps/simple_warehouse_fixed.yaml
```

저장·설정 생성 성공을 확인한 뒤 터미널 3 Nav2와 터미널 2 Cartographer/RViz, 직선 표시 노드를 종료합니다. 기록기도 Ctrl+C로 종료합니다. **Gazebo는 그대로 유지합니다. 우분투/VM 재부팅은 필요 없습니다.** 이어서 C단계를 실행합니다. Cartographer와 AMCL을 동시에 켜지 않습니다.

## C. 저장 지도에서 위치 추정·주행 — 지도를 다시 만들지 않음

같은 날 B단계에서 넘어왔다면 Gazebo는 유지되어 있습니다. 다음 날 모두 종료된 상태라면 터미널 1에서 같은 단순 창고 런처만 다시 실행하고, **A단계의 Cartographer·탐색기는 실행하지 않습니다.** 저장한 지도와 AMCL 설정 파일은 그대로 재사용합니다.

터미널 2 — 고정 지도와 AMCL:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true autostart:=true \
  map:=$HOME/ros2_practice_ws/maps/simple_warehouse_fixed.yaml \
  params_file:=$HOME/ros2_practice_ws/nav_house_localization.yaml
```

터미널 3 — Nav2:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_localization.yaml
```

터미널 4 — RViz:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
rviz2 --ros-args -p use_sim_time:=true
```

Fixed Frame=map, Map 토픽=/map, Durability=Transient Local로 설정하고 LaserScan=/scan을 추가합니다. `2D Pose Estimate`로 **저장 지도에서 현재 로봇의 대략적인 위치·방향**을 지정합니다. Gazebo 좌표가 SLAM map 좌표와 같다고 가정하지 않습니다. 라이다가 지도 벽과 맞는지 확인한 뒤 Nav2 Goal을 추가하여 주행합니다. C단계에는 자동 매핑 탐색기를 실행하지 않습니다.

고정 지도에서 직선을 보려면 선택 사항인 추가 터미널에서 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/map_line_refiner.py \
  --ros-args -p use_sim_time:=true \
  -p map_transient_local:=true -p observations:=1
```

C단계의 `/map`은 map_server가 저장 이미지에서 발행하므로 형태가 유지됩니다. AMCL이 라이다로 위치를 추정해 `map → odom`을 보정합니다. 구동기의 `odom → base_footprint`는 계속 유지합니다. 형태 고정은 위치 추정 오차 0을 의미하지 않습니다. 기존 지도보다 넓은 구역을 새로 매핑하려면 별도 매핑 세션이 필요합니다.

## 저장 위치와 검증 상태

Windows 작업공간의 이 통합 안내서, `docs/DEVELOPMENT_JOURNEY.md`, `docs/evidence/진행상태_2026-10-23.md`에 현재 상태를 저장했습니다. 사용자 첨부 변형 화면도 증거 폴더에 보존합니다. 우분투 지도 파일과 rosbag은 위 명령을 사용자가 실행해야 생성됩니다. Windows에서 대신 저장하거나 VM 지도를 자동으로 가져온 상태는 아닙니다.

최신 통합 배포는 `current_simple_warehouse_bundle.zip`이며 파일 15개입니다. 코드 검사 106개 통과는 Windows의 합성 데이터·설정 생성 검증입니다. 사용자 화면에서 직선 표시 작동을 확인했지만 지도 왜곡 해결·전체 매핑·복귀·AMCL 주행 성공을 의미하지 않습니다.
