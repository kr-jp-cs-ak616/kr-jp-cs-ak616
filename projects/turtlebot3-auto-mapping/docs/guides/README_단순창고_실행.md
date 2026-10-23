> **현재 진행 순서는 [통합 실행 안내](README_현재진행_통합실행.md)를 기준으로 합니다.** 이 문서는 기능별 상세 참고입니다.

# VM용 단순 창고 · Waffle Pi

8×6m 공간에 벽·선반 4개·적재실과 상자 2개를 배치합니다. 출입구 폭은 1.2m이며 지붕이 없습니다. 월드는 정적 상자 13개로만 구성하고 외부 메시·텍스처·AWS 모델을 사용하지 않습니다. 그림자는 끕니다. 로봇은 기존 Waffle Pi이며 센서 계산 부하는 남아 있습니다. 실제 VM에서의 성능·주행은 아직 확인하지 않았습니다.

`gui:=false`는 Gazebo 창 없이 서버만 실행하는 옵션입니다. 기존 명령에서 창이 안 떴다는 사실만으로 서버 실행 실패를 판단할 수 없습니다. 이번 런처의 기본은 `gui:=true`입니다.

## 파일 복사 및 종료

`simple_warehouse_waffle_pi_bundle.zip`의 파일 11개를 우분투 `~/ros2_practice_ws`에 압축 해제합니다. 같은 이름은 최신 파일로 교체합니다. AWS 저장소 다운로드나 추가 colcon build는 필요하지 않습니다.

기존 탐색기 → Nav2 → Cartographer/RViz → Gazebo 순서로 종료합니다. 다른 키보드 조종기도 종료합니다. 새 월드에서는 기존 AWS/House 지도를 불러오지 않고 새로 매핑합니다. 기존 지도와 원본 파일은 삭제할 필요가 없습니다.

## 터미널 1: 준비 및 Gazebo

아래 준비 명령은 하나씩 성공 여부를 확인하고 진행합니다.

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

Gazebo 창과 Waffle Pi가 보이는지 먼저 확인합니다. 기본 출발 위치는 x=-3m, y=-2m, yaw=0이며 상자 배치 기준 반경 0.3m의 수평 여유를 검사했습니다. 실제 스폰·물리 접촉 검증은 별도입니다. 바퀴 지름 0.066m, 중심 거리 0.288m, 직진 상한 0.10m/s 설정을 사용합니다. 생성 월드는 `~/ros2_practice_ws/warehouse_worlds/simple_warehouse.world`입니다.

## 터미널 2: SLAM/RViz

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=true \
  cartographer_config_dir:=$HOME/ros2_practice_ws/house_revisit_config \
  configuration_basename:=house_revisit.lua
```

RViz Fixed Frame은 map, Frame Rate는 10으로 설정합니다. Gazebo는 재생 상태로 둡니다. AMCL을 동시에 실행하지 않습니다.

## 터미널 3: Nav2

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_frontier.yaml
```

`Managed nodes are active`가 나올 때까지 기다립니다. 공통 설정 파일의 House 접두사는 실제 월드 선택과 무관합니다.

## 터미널 4: 자동 탐색

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 lifecycle get /controller_server
ros2 param get /controller_server FollowPath.max_vel_x

python3 ~/ros2_practice_ws/house_nav2_frontier.py \
  --ros-args -p use_sim_time:=true
```

상태 active와 속도 0.1을 확인합니다. RViz에 MarkerArray `/house_frontier/markers`, Path `/house_frontier/path`를 추가합니다. Path Buffer Length는 1로 두고 다른 경로 표시는 끕니다. 관심 지점 선택·단일 목표 이동·미탐색 경계 탐색·막힌 면 재관측·완료 판정 후 출발점 복귀는 공통 탐색기의 기존 기능입니다. 실제 전 구역 완료와 복귀 성공은 주행으로 검증해야 합니다.

## 실행 실패·멈춤 확인

Gazebo 창 또는 로봇이 뜨지 않으면 터미널 1의 `ros2 launch` 이후 오류를 확인합니다. 반복 실행하기 전에 기존 프로세스가 끝났는지 확인합니다. 추가 터미널에서 다음으로 서버와 시간 상태를 구분할 수 있습니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
ros2 node list
ros2 topic echo /clock --once
```

`/clock` 명령이 응답 없이 대기하면 Ctrl+C로 종료하고 터미널 1의 오류를 확인합니다. 서버가 정상 실행 중인데 GUI만 부담스럽다면 모든 실행을 종료한 후 터미널 1의 마지막 명령을 `gui:=false`로 바꿀 수 있습니다. 이때 Gazebo 창은 열리지 않습니다.

주행 중 멈춤 진단은 별도 터미널에서 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/diagnose_house_motion.py --seconds 60
```

결과는 `~/ros2_practice_ws/diagnostics`에 저장됩니다.

## 지도 저장

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
mkdir -p ~/ros2_practice_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_practice_ws/maps/simple_warehouse_auto \
  --ros-args -p use_sim_time:=true
```

현재 Windows 검사는 월드 XML, 외부 자산 독립성, 시각·충돌 형상 일치, 출발·통로 여유를 포함합니다. Gazebo 렌더링이나 주행 성능 검증을 대신하지 않습니다.
