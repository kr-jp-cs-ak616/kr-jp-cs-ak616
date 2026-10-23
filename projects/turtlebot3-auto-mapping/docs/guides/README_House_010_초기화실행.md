# Waffle Pi 0.10m/s: 초기화 후 처음부터 실행

이번 프로필은 표준 Gazebo Classic Waffle Pi + ROS 2 Humble + Cyclone DDS + ROS_DOMAIN_ID=30용입니다. 설치 모델과 원본 Nav2 설정은 보존하고 작업공간에 별도 수정본을 만듭니다. colcon build는 필요하지 않습니다.

좁은 출발 위치의 후보 생성도 사각 몸체 검사로 수정했습니다. 이미 이 프로필의 Nav2가 실행 중이라면 자동 탐색기만 Ctrl+C로 종료하고 최신 `house_nav2_frontier.py`를 교체한 뒤 터미널 4의 탐색 명령으로 다시 실행하면 됩니다. 시작 로그에 `후보 몸체 검사: 방향별 사각 윤곽`이 나오는지 확인하세요. 이번 수정 때문에 Gazebo·Cartographer를 다시 초기화할 필요는 없습니다.

## 적용값

| 항목 | 이번 설정 |
|---|---|
| 직진 속도 상한 | 0.10m/s (이전 0.08m/s보다 25% 증가) |
| 회전 속도 상한 | 0.20rad/s (약 11.46도/초) |
| 직진 가속·감속 크기 | 0.10m/s² |
| 각가속·각감속 크기 | 0.30rad/s² |
| 바퀴 반경·지름 | 0.033m / 0.066m |
| 바퀴 중심 거리 | 모델 형상과 일치하는 0.288m (플러그인의 기존 0.287m 수정) |
| Gazebo 바퀴 접선 가속도 설정 | 0.20m/s² |
| 몸체 검사 | 사각 윤곽 + 여유 0.01m, DWB ObstacleFootprint |
| 오도메트리 | 기존 시뮬레이션 WORLD 모드 유지 |

제조사 최대 직진 0.26m/s·회전 1.82rad/s와 구분하세요. 위 가속도·운용 속도는 매핑 시험을 위한 선택값이며 제조사 보장 최대값이 아닙니다. 바퀴 치수를 속도에 비례해 늘리는 방식은 사용하지 않습니다. 반경 0.033m, 중심 거리 0.288m는 이전 접촉 없는 측정 결과와도 일치합니다. 모터 토크·마찰·질량·관성은 원본을 유지합니다.

차동구동 변환은 `왼쪽 바퀴=(v-0.288*w/2)/0.033`, `오른쪽 바퀴=(v+0.288*w/2)/0.033` rad/s입니다. 직진 0.10m/s에서 양쪽 약 3.03rad/s(28.94rpm), 선속도·회전 상한을 함께 명령하면 바깥쪽 약 3.90rad/s(37.27rpm)입니다. 제어기의 직진·각가속도를 함께 적용한 바퀴 접선 가속도 계산값은 0.1432m/s²이므로 플러그인 설정 0.20m/s² 안에 있습니다. 모델 축간 거리 정합은 지도 왜곡의 해결을 보장하지 않으며 WORLD odom은 실물 엔코더 오도메트리와 다릅니다.

## 0. 파일 복사와 종료

`house_waffle_pi_010_bundle.zip` 안 파일 10개를 모두 우분투 `~/ros2_practice_ws`에 압축 해제합니다. 기존 파일은 같은 이름으로 교체합니다. 최신 배포본에는 별도 관측용 `diagnose_house_motion.py`도 포함됩니다.

이전 자동 탐색기, Nav2, Cartographer/RViz, 기록기, 키보드 조종기, Gazebo를 각 터미널에서 Ctrl+C로 종료하고 남아 있는 창도 닫습니다. 기존 저장 지도·bag·소스는 삭제하지 않습니다. Gazebo Reset Time만 눌러 실행 중인 노드와 시간을 섞지 마세요. 새로 실행하면 진행 중이던 SLAM 지도와 탐색 방문 기록은 새 세션으로 시작합니다.

이하 각 터미널은 모두 같은 환경을 사용합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
```

## 1. 터미널 1: 설정 생성 후 새 Gazebo

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

python3 ~/ros2_practice_ws/prepare_house_wheels_010.py

python3 ~/ros2_practice_ws/prepare_house_nav2_frontier.py \
  --speed 0.10 --yaw-rate 0.20 --waffle-footprint

python3 ~/ros2_practice_ws/prepare_house_revisit.py --mode mapping \
  --source "$(ros2 pkg prefix --share turtlebot3_cartographer)/config/turtlebot3_lds_2d.lua"

ros2 launch ~/ros2_practice_ws/house_waffle_pi_010.launch.py
```

각 생성 명령이 성공해야 다음 명령을 실행합니다. 마지막 실행은 생성한 `~/ros2_practice_ws/house_waffle_pi_010/model.sdf`를 명시적으로 스폰하므로, 수정하지 않은 설치 모델을 실수로 다시 스폰하는 일을 피합니다. 기존 `turtlebot3_house.launch.py`를 추가 실행하지 않습니다. 시작 로그의 `이번에 실제 스폰할 모델` 경로를 확인하세요.

출발 위치 기본값은 사용자가 보내준 상태 파일의 이전 출발점 WORLD 좌표 `(1.676, 0.401)`, yaw `-0.451rad`입니다. 다른 출발점이 필요하면 위 마지막 명령 대신 다음처럼 지정할 수 있습니다.

```bash
ros2 launch ~/ros2_practice_ws/house_waffle_pi_010.launch.py \
  x_pose:=1.676 y_pose:=0.401 yaw:=-0.451
```

Gazebo에서 로봇이 가구·벽과 겹치지 않는지 확인하고 재생 상태로 둡니다. 위치를 바꾸려면 Cartographer 시작 전에만 바꿉니다. 바퀴 형상이 표준과 다르면 생성기는 임의의 치수를 적용하지 않고 중단합니다.

## 2. 터미널 2: 새 Cartographer 지도

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=true \
  cartographer_config_dir:=$HOME/ros2_practice_ws/house_revisit_config \
  configuration_basename:=house_revisit.lua
```

RViz Fixed Frame은 map으로 유지합니다. 이전 비교용 `house_local_only.lua`는 사용하지 않습니다. 새 설정은 원본의 센서·가중치·전역 최적화 주기를 상속하며 전역 최적화가 꺼져 있으면 오류로 알립니다. AMCL은 매핑 중 함께 실행하지 않습니다.

## 3. 터미널 3: Nav2

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_frontier.yaml
```

`Managed nodes are active`를 확인합니다.

## 4. 터미널 4: 준비 확인 후 자동 목표 탐색

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 lifecycle get /controller_server
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_vel_theta
ros2 param get /local_costmap/local_costmap footprint

python3 ~/ros2_practice_ws/house_nav2_frontier.py --ros-args -p use_sim_time:=true
```

기대값: active [3], 0.1, 0.2, `footprint`에 사각형 좌표가 출력됩니다. `robot_radius`가 0.28로 남아 있어도 비어 있지 않은 footprint를 사용하면 몸체 검사는 그 다각형을 사용합니다. 후보 생성기도 같은 사각 윤곽과 여유로 방향별 검사를 하며 반경 0.30m 원을 필수 조건으로 사용하지 않습니다. 통로의 실제 경로·충돌 판정은 Nav2가 합니다. 시간 제한은 없고, 연결된 미탐색 경계가 안정적으로 사라지면 출발점으로 복귀하는 기존 기능을 유지합니다.

시작 위치는 고정 좌표에 의존하지 않고 TF에서 읽습니다. 표준 Waffle Pi의 `goal_footprint` 기본값은 Nav2와 같으며 `goal_padding=0.01`입니다. 몸체를 개조했다면 탐색기의 `goal_footprint`·`goal_padding`도 Nav2와 일치시켜야 합니다. 원형 검사를 명시적으로 사용하려면 `-p goal_footprint:='[]'`를 줄 수 있고 이때만 `goal_clearance`를 사용합니다. 벽과 겹친 스폰, 몸체보다 좁은 통로, 아직 관측되지 않은 몸체 영역, TF·센서·Nav2 미준비는 강제로 무시하지 않습니다. 준비 대기 이유는 로그와 상태 파일의 `wait_reason`, 후보 부족은 `report`에 기록됩니다.

RViz에 MarkerArray `/house_frontier/markers`와 Path `/house_frontier/path`를 추가합니다. Path는 하나만 남기고 Buffer Length를 1로 설정하며 다른 경로 표시는 체크 해제합니다. 관심 지점도 자동으로 선택합니다. 분홍 점은 현재 관심 위치, 초록 화살표는 이동 목표 하나입니다. 일반 탐색 후 막힌 면의 앞쪽도 순차 재관측합니다. 자동 탐색 중 키보드 조종이나 다른 벽 추종기를 함께 실행하지 않습니다.

선택적으로 다른 터미널에서 실제 스폰에 사용한 파일을 점검할 수 있습니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/wheel_geometry_audit.py \
  --sdf ~/ros2_practice_ws/house_waffle_pi_010/model.sdf
```

형상·설정 중심 거리 모두 0.288m, 지름 0.066m가 기대값입니다. 이번에는 수동 60초 측정을 다시 해야 하는 절차가 아닙니다.

## 검증과 근거

Windows에서 Nav2 설정·목표 생성·좁은 출발 위치·주행 진단·바퀴 변환·모델 수정 관련 전체 93개 검사를 통과했습니다. Python 문법 검사도 통과했습니다. Ubuntu/Gazebo 실제 주행과 모델 스폰은 이 환경에서 실행하지 않았습니다. 망설임 관측은 별도 터미널에서 `python3 ~/ros2_practice_ws/diagnose_house_motion.py --seconds 60`으로 수집할 수 있습니다.

- [ROBOTIS Waffle Pi 제원](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)
- [표준 Gazebo Waffle Pi 모델](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf)
- [Gazebo 차동구동 속도 변환과 가속 제한](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)
