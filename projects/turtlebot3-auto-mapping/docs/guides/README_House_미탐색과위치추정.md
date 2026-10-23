# 미탐색 목표 선택과 라이다 위치 추정

현재 0.10m/s·바퀴 형상 정합 프로필의 새 세션 실행은 `README_House_010_초기화실행.md`를 우선 사용하세요. 아래 0.08m/s·0.15rad/s 언급은 이전 프로필에 대한 설명입니다.

## 이번 수정 적용

Windows의 `house_nav2_frontier_bundle.zip`을 우분투 `~/ros2_practice_ws`에 압축 해제하여 같은 이름의 파일을 교체합니다. Python 파일이므로 colcon build는 필요하지 않습니다. 실행 중인 자동 탐색 노드를 Ctrl+C로 종료한 뒤 교체하세요.

목표 선택만 바꾸어 시험하려면 Gazebo·Cartographer·Nav2는 유지하고 자동 탐색 터미널에서 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/house_nav2_frontier.py --ros-args -p use_sim_time:=true
```

RViz Fixed Frame은 `map`, MarkerArray 토픽은 `/house_frontier/markers`입니다. 파란 점은 후보, `gain=...m2`는 예상 추가 관측 면적, 노란 점은 그 후보가 바라보는 미탐색 경계, 초록 화살표는 전송한 목표입니다. 미탐색 셀 자체는 몸체 통과가 확인되지 않았으므로 목표를 그 앞의 빈 공간에 둡니다. 도착 후 스캔으로 알려진 공간이 늘어나면 다음 목표가 안쪽으로 이동합니다. 벽 뒤 공간은 문을 통과하는 Nav2 경로가 있어야 접근합니다. 라이다 무반사는 빈 공간이나 문이라는 확증이 아닙니다.

현재 수정은 후보의 가시성·예상 관측 면적·재선택 억제를 개선합니다. 지도 자체가 찌그러지거나 TF가 끊어지는 문제까지 해결했다고 판정한 것은 아닙니다. 계속 같은 곳을 선택하면 `~/ros2_practice_ws/house_nav2_frontier_state.json`을 보관해 확인해야 합니다.

## 매핑 중: Cartographer로 재방문 위치 보정

Cartographer는 이미 현재 라이다를 서브맵과 비교하여 위치를 추정합니다. 전역 최적화가 활성화되면 재방문 시 루프 폐쇄 제약으로 누적 오차도 보정합니다. 이전의 `house_local_only.lua`는 원인 비교 시험용으로 전역 최적화를 껐던 설정입니다. 재방문 보정을 쓰려면 원래 전역 SLAM 설정으로 돌아가야 합니다. 임의의 라이다 값 하나를 저장해 위치를 단정하는 방식은 비슷한 방에서 위치를 혼동할 수 있습니다.

Cartographer가 실행 중일 때 준비 명령을 실행하면 사용 중인 Lua 경로를 출력합니다. `house_local_only.lua`라면 같은 폴더의 보존된 `house_baseline.lua`를 사용합니다. 이 명령은 파일만 생성합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/prepare_house_revisit.py --mode mapping
```

실행 중인 Cartographer가 없다면 원본을 명시합니다.

```bash
python3 ~/ros2_practice_ws/prepare_house_revisit.py --mode mapping \
  --source "$(ros2 pkg prefix --share turtlebot3_cartographer)/config/turtlebot3_lds_2d.lua"
```

기존 지도를 보존하려면 종료 전에 저장합니다. 이것은 이미지 지도이며, Cartographer의 진행 중인 SLAM 상태를 재개하는 파일은 아닙니다.

```bash
mkdir -p ~/ros2_practice_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_practice_ws/maps/house_before_restart \
  --ros-args -p use_sim_time:=true
```

새 매핑 세션을 시작할 때 자동 탐색 → Nav2 → Cartographer 순으로 종료합니다. Gazebo는 유지할 수 있습니다. Cartographer를 아래 명령으로 실행한 뒤 기존 Nav2 실행 명령, 마지막으로 자동 탐색 명령을 실행합니다. 기존 SLAM 지도가 이어지는 것은 아니며 현재 위치에서 새 지도를 만듭니다. TF 축을 숨기거나 map→odom을 고정하지 마세요.

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=true \
  cartographer_config_dir:=$HOME/ros2_practice_ws/house_revisit_config \
  configuration_basename:=house_revisit.lua
```

생성기는 원본의 센서 설정·정합 가중치·최적화 주기를 유지합니다. Lua 실행 시 전역 최적화 주기가 0이면 명확한 오류로 중단하여 비활성화 상태로 계속 매핑하지 않도록 했습니다. 루프 폐쇄가 있어도 잘못된 센서·시간·정합 입력을 자동으로 모두 해결하지는 못합니다.

## 매핑 완료 후: 저장 지도에서 AMCL 위치 추정

이 절차는 다음 주행부터 같은 실내 지도를 재사용할 때 사용합니다. 매핑 도중에는 위 Cartographer 방식을 사용하세요. AMCL은 지도를 확장하지 않습니다.

완성된 House 지도가 만들어진 상태에서 저장합니다. 이전 다른 월드의 `waffle_pi_auto` 지도를 사용하지 마세요.

```bash
mkdir -p ~/ros2_practice_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_practice_ws/maps/house_lidar \
  --ros-args -p use_sim_time:=true
python3 ~/ros2_practice_ws/prepare_house_revisit.py --mode localization \
  --map ~/ros2_practice_ws/maps/house_lidar.yaml
```

생성기는 실제 지도 YAML과 이미지 존재를 확인하고 `nav_house_frontier.yaml`의 주행 설정을 보존하여 `nav_house_localization.yaml`을 만듭니다. AMCL 기본값은 설치된 Humble nav2_bringup에서 가져옵니다. 속도는 기존 0.08m/s 설정을 유지합니다.

자동 탐색·Nav2·Cartographer를 종료하고 Gazebo는 재생 상태로 유지합니다. 새 터미널마다 환경과 도메인을 설정하세요. Cartographer와 AMCL을 함께 실행하면 두 노드가 map→odom을 발행하므로 함께 실행하지 않습니다.

터미널 2:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true autostart:=true \
  map:=$HOME/ros2_practice_ws/maps/house_lidar.yaml \
  params_file:=$HOME/ros2_practice_ws/nav_house_localization.yaml
```

터미널 3:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_localization.yaml
```

RViz에서 `2D Pose Estimate`로 현재 위치와 방향의 대략적인 초기값을 지정합니다. 이후 AMCL이 저장 지도와 현재 라이다의 일치도를 바탕으로 위치를 계속 추정합니다. PoseWithCovariance 표시의 토픽 `/amcl_pose`로 추정 위치를 확인할 수 있습니다. 초기 위치를 전혀 모르는 경우 전역 초기화도 가능하지만 모양이 비슷한 방에서는 불확실하므로 이 절차에서는 대략적인 초기값을 사용합니다. 위치가 지도·라이다와 맞는지 확인한 뒤 Nav2 Goal로 주행합니다. 저장 지도 주행에서는 자동 매핑 노드를 다시 실행하지 않습니다.

## 표준 Waffle Pi의 좁은 통로 윤곽 수정

`robot_radius=0.28`, `footprint=[]`이면 로봇은 반경 0.28m의 원으로 검사됩니다. 좁은 통로에서는 표준 Waffle Pi보다 큰 원 때문에 전진·회전 궤적이 제한될 수 있습니다. 표준 Gazebo 모델에는 아래 명령으로 몸체와 바퀴를 포함하는 사각 윤곽을 사용합니다.

자동 탐색기와 Nav2를 Ctrl+C로 종료합니다. Gazebo와 Cartographer는 유지합니다. 수정된 `prepare_house_nav2_frontier.py`를 우분투에 복사한 뒤 Nav2 터미널에서 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/prepare_house_nav2_frontier.py --speed 0.08 --waffle-footprint
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_frontier.yaml
```

Nav2가 active가 되면 자동 탐색기를 원래 명령으로 다시 실행합니다. colcon build는 필요하지 않습니다. 로컬·전역 비용지도 모두 `footprint=[[-0.205,-0.16],[0.10,-0.16],[0.10,0.16],[-0.205,0.16]]`, padding 0.01m를 사용합니다. DWB는 `BaseObstacle`을 `ObstacleFootprint`으로 바꿔 모서리까지 충돌 검사합니다. 현재 기본 프로필의 선속도는 0.10m/s, 회전 상한은 0.20rad/s입니다. 후보 생성기도 사각 몸체와 방향을 검사하므로 0.30m 원이 들어갈 필요가 없습니다. 사각 윤곽 모드에서는 남아 있는 robot_radius 파라미터가 몸체 검사에 사용되지 않으므로 footprint 파라미터와 published_footprint 표시로 적용을 확인합니다.

이 치수는 표준 Gazebo Waffle Pi용입니다. 출처 모델의 몸체는 x=-0.1965~0.0685m, 바퀴 바깥쪽은 y=±0.153m입니다. 수정 윤곽은 이를 포함합니다. 실제 스폰 모델에 치수 변경이나 부착물이 있다면 그 외곽에 맞추어야 합니다. 현재 정지의 모든 원인이 해결되었다고 판정한 것은 아니며, 계속 멈추면 controller_server의 같은 시점 경고·오류로 추가 확인합니다.

근거: [표준 Waffle Pi SDF](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf), [Humble DWB 윤곽 검사](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp).

## 검증 범위

현재 Windows에서 목표 생성·벽 가시성·관심 영역·주행 진단·완료 판정·표시·윤곽·설정 생성 등 전체 자동 검사 86개를 통과했습니다. 실제 Ubuntu ROS/Gazebo 주행은 이 환경에서 실행하지 않았습니다.

근거: [Cartographer 알고리즘](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html), [Humble AMCL 기본 설정](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml), [Humble localization launch](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/localization_launch.py).
