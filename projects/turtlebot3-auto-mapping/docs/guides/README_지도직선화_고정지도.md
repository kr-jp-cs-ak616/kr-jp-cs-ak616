> **현재 진행 순서는 [통합 실행 안내](README_현재진행_통합실행.md)를 기준으로 합니다.** 이 문서는 기능별 상세 참고입니다.

# 지도 경계 직선화와 고정 지도 주행

## 두 기능의 차이

`map_line_refiner.py`는 격자 지도에서 반복 관측된 장애물 경계를 추출하여 RViz 직선과 SVG로 표시합니다. 원본 `/map`과 Nav2 지도는 변경하지 않습니다. 픽셀 계단을 보기 좋게 정리하는 기능이며 잘못된 SLAM 지도를 복구하는 기능은 아닙니다.

지도 형태를 고정하는 기능은 매핑 완료 후 저장 지도와 AMCL을 사용하는 별도 단계입니다. AMCL이 현재 라이다와 저장 지도를 비교하여 `map → odom`을 보정하고, 구동기가 `odom → base_footprint`를 계속 발행합니다. 지도를 고정한 상태에서 AMCL은 새 구역을 매핑하지 않습니다. Cartographer로 매핑하는 동안에는 누적 오차·재방문 제약에 맞춰 지도와 궤적이 바뀔 수 있습니다. 원점과의 거리 자체가 지도 변형의 원인은 아닙니다. odom을 강제로 덮어쓰거나 map→odom을 static TF로 만들지 않습니다.

## 1. 직선 표시 실행

배포 ZIP의 파일 4개를 우분투 `~/ros2_practice_ws`에 복사합니다. ROS에 포함된 메시지와 Python 표준 라이브러리만 사용하므로 추가 pip·apt·colcon build가 필요하지 않습니다.

Gazebo·Cartographer·Nav2는 기존대로 유지하고 추가 터미널에서 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

python3 ~/ros2_practice_ws/map_line_refiner.py \
  --ros-args -p use_sim_time:=true
```

RViz에서 다음을 설정합니다.

1. Fixed Frame은 `map`으로 둡니다.
2. Add → By topic → `/map_lines` → **Marker**를 선택합니다. MarkerArray가 아닙니다.
3. 파란색 직선이 원본 경계 위에 표시됩니다. 원본 Map의 Alpha를 0.2 정도로 낮춰 비교할 수 있습니다.
4. 직선만 보려면 Map 표시 체크를 끕니다. 이는 RViz 표시만 숨기며 SLAM·Nav2에는 영향을 주지 않습니다. 이때 빈 공간과 미탐색 영역의 면 표시도 사라집니다.

기본값은 점유도 65 이상인 장애물과 점유도 0~25의 빈 공간 사이 경계입니다. 미탐색(-1)과 맞닿은 경계는 그리지 않습니다. 처리한 서로 다른 지도 시각 3회에서 계속 관측된 장애물 셀만 사용합니다. 2초마다 최신 지도를 처리하므로 최초 표시까지 여러 지도 업데이트가 필요합니다. 반복 관측 조건은 통계적 확정이나 SLAM 품질 보장을 뜻하지 않습니다.

이어진 경계 안에서 최대 0.075m 오차로 선분을 근사하고 길이 0.35m 미만은 제외합니다. 분기점·단절을 넘어 선을 연결하거나 문을 닫는 처리는 하지 않습니다. 근사 선은 실제 벽과 일치하는 측량선이 아니며 허용 오차 범위에서 경계 위치가 바뀔 수 있습니다. 작은 물체를 더 표시하려면 길이 제한을 낮출 수 있습니다.

```bash
python3 ~/ros2_practice_ws/map_line_refiner.py \
  --ros-args -p use_sim_time:=true \
  -p tolerance:=0.05 -p min_length:=0.20 -p observations:=3
```

기존 직선화 노드를 Ctrl+C로 종료한 뒤 대체 명령 하나만 실행합니다. 여러 노드가 같은 Marker ID를 쓰지 않도록 합니다. 더 큰 오차를 적용한다고 지도 정합 문제가 해결되지는 않습니다. 전체 지도 변경 없이 별도 표시만 생성합니다.

자동 저장 위치:

- `~/ros2_practice_ws/map_refined/map_lines.svg`: 픽셀 이미지가 아닌 벡터 선분 파일.
- `~/ros2_practice_ws/map_refined/map_lines.json`: map 좌표계의 선분 끝점(m), 지도 시각, 추출 기준.

최신 결과로 같은 파일을 갱신합니다. 보관하려면 원하는 시점에 다른 이름으로 복사합니다. SVG는 선분만 있는 도면이며 Nav2의 지도 YAML/PGM을 대체하지 않습니다. 노드가 종료되거나 시간·센서 입력이 멈추면 마지막 표시를 최신 관측으로 해석하지 않습니다.

첨부한 motion 진단 파일은 전체 `/map` 데이터를 포함하지 않아 실제 지도 직선 결과를 사전 생성할 수 없었습니다. Windows에서 합성 격자의 선분 근사 오차·문 단절·미탐색 제외·좌표 변환·시각 반복·관측 초기화 검사를 수행했습니다. 실제 Ubuntu ROS와 첨부 화면의 지도에 대한 결과는 실행 후 확인해야 합니다.

## 2. 매핑 완료 후 지도 고정

매핑 중 지도를 고정하면 새 구역을 계속 추가하는 목적과 충돌합니다. 전체 공간을 매핑한 뒤 자동 탐색기를 먼저 종료하고, Cartographer가 아직 실행 중일 때 지도를 저장합니다. Gazebo는 재생 상태로 유지합니다.

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

`Map saved successfully`와 설정 생성 성공을 확인한 뒤 **기존 Nav2와 Cartographer/RViz를 종료**합니다. 직선화 노드도 종료합니다. Gazebo는 유지합니다. 저장한 지도에 이미 변형이 있으면 그 형태 그대로 고정됩니다. AMCL 전환이 기존 변형을 펴 주지는 않습니다.

새 터미널 2에서 저장 지도를 로드합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true autostart:=true \
  map:=$HOME/ros2_practice_ws/maps/simple_warehouse_fixed.yaml \
  params_file:=$HOME/ros2_practice_ws/nav_house_localization.yaml
```

새 터미널 3에서 주행을 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_localization.yaml
```

Cartographer와 함께 실행하지 않습니다. AMCL이 map→odom의 유일한 SLAM/위치추정 발행원이 되어야 합니다. 생성기는 기존 0.10m/s 주행 설정을 이어받습니다.

RViz가 종료되었다면 환경이 설정된 별도 터미널에서 열 수 있습니다.

```bash
rviz2 --ros-args -p use_sim_time:=true
```

Fixed Frame을 map으로 설정하고 Map 토픽 `/map`을 추가합니다. Map의 Durability Policy는 Transient Local로 설정합니다. LaserScan `/scan`, RobotModel, TF를 필요한 만큼 추가합니다. `2D Pose Estimate`로 저장 지도에서 **현재 로봇의 대략적인 위치와 방향**을 지정합니다. Gazebo 스폰 좌표와 SLAM map 좌표는 같다고 가정하지 않습니다. 라이다와 지도 벽의 정렬을 확인한 뒤 Nav2 Goal로 이동합니다. 자동 매핑 노드는 다시 실행하지 않습니다.

고정 지도에서도 직선을 표시하려면 추가 터미널에서 실행합니다. 저장 지도는 한 번 발행된 데이터도 받을 수 있도록 QoS와 반복 조건을 바꿉니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/map_line_refiner.py \
  --ros-args -p use_sim_time:=true \
  -p map_transient_local:=true -p observations:=1
```

지도가 고정되어도 위치 추정 오차가 완전히 사라지는 것은 아닙니다. 비슷한 선반 배치·가려진 라이다·미끄러짐·시간 불일치가 있으면 AMCL 위치가 틀릴 수 있습니다. 저장 지도 형태는 유지하면서 위치 추정을 별도로 검증하는 방식입니다.

근거: [Humble Marker 정의](https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/Marker.msg), [OccupancyGrid 정의](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/OccupancyGrid.msg), [Humble AMCL의 map→odom 계산](https://github.com/ros-navigation/navigation2/blob/humble/nav2_amcl/src/amcl_node.cpp).
