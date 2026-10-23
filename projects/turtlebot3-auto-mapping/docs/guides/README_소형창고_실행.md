# AWS 소형 창고 · Waffle Pi · 자동 매핑

이번 실행은 지붕 없는 창고를 기본으로 사용합니다. `prepare_small_warehouse.py`가 원본의 RoofB 지붕 모델만 제외한 `~/ros2_practice_ws/warehouse_worlds/small_warehouse_no_roof.world`를 생성합니다. 원본 자산은 보존합니다.

이번 실행은 AWS RoboMaker Small Warehouse의 보존된 `ros2` 분기 월드·모델 자산을 사용합니다. 기존 Gazebo Classic·ROS 2 Humble·Cyclone DDS와 Waffle Pi 0.10m/s 프로필을 유지합니다. House는 비교 대상으로 보존합니다.

AWS 모델 자산은 별도로 내려받습니다. 예전 AWS launch를 실행하거나 패키지를 빌드하는 대신, 이번 별도 런처가 원본 world 경로와 모델·미디어 경로를 지정하여 Gazebo와 Waffle Pi를 함께 실행합니다. 추가 colcon build는 필요하지 않습니다.

## VM에서 멈출 때: 경량 모드

최신 ZIP의 `prepare_small_warehouse.py`와 `small_warehouse_waffle_pi.launch.py`를 우분투 작업공간에 덮어씁니다. 탐색기 → Nav2 → Cartographer/RViz → Gazebo 순서로 종료한 뒤 터미널 1에서 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/prepare_small_warehouse.py --vm-lite
ros2 launch ~/ros2_practice_ws/small_warehouse_waffle_pi.launch.py gui:=false
```

기존 바퀴·Nav2·SLAM 준비가 성공한 환경에서 사용하는 명령입니다. 최초 실행이라면 아래 터미널 1의 나머지 준비 명령도 먼저 수행합니다. 이후 터미널 2 → 3 → 4를 실행합니다. Gazebo 창은 열리지 않으며 RViz로 지도를 확인합니다. RViz Global Options의 Frame Rate를 10으로 낮추고 TF의 Show Names, 불필요한 디스플레이를 끄면 화면 부하를 추가로 줄일 수 있습니다.

경량 월드는 `warehouse_worlds/small_warehouse_vm_lite.world`입니다. 벽·선반·바닥 배치는 유지하고 Cluttering·Bucket·TrashCan·PalletJack·Lamp 소품 모델을 제외합니다. 따라서 원본과 장애물 구성이 다릅니다. 기존 지도는 사용하지 않고 새로 매핑합니다. 원본 자산은 보존합니다.

그림자를 끄고 물리 적분 간격 0.001초는 유지하며 업데이트 목표를 500Hz, 실시간 목표를 0.5배로 제한합니다. 로봇 설정 속도는 시뮬레이션 기준 0.10m/s이며, 실제 시간으로는 느리게 보입니다. VM에서 성능 개선이나 멈춤 해소를 보장하지 않습니다. 선반과 벽의 원본 메시, 로봇 센서 계산 부하는 남아 있습니다. 일반 모드로 복귀하려면 `prepare_small_warehouse.py`를 옵션 없이 다시 실행합니다.

Gazebo 창을 끈 상태에서도 `/clock`이 멈춘다면 우분투에서 다음을 확인합니다.

```bash
free -h
vmstat 1 5
top -b -n 1 -o %CPU | head -n 20
ros2 topic hz /clock
```

`ros2 topic hz`는 몇 초 확인한 뒤 Ctrl+C로 끝냅니다. 메모리 부족·스왑 사용·CPU 포화와 시뮬레이션 시간 정지를 구분하기 위한 진단입니다. 이 경량 모드는 Windows에서 XML 변환·기본 구조 보존을 검사했으며 실제 VM 주행은 아직 확인하지 않았습니다.

## 0. 파일 복사와 기존 실행 종료

`small_warehouse_waffle_pi_bundle.zip` 안 파일 11개를 우분투 `~/ros2_practice_ws`에 압축 해제합니다. 같은 이름의 공통 파일은 최신 버전으로 교체합니다. 로봇·탐색 코드의 House 접두사는 공통 파일 이름으로 유지했으며 실제 월드는 새 런처가 결정합니다.

필요한 기존 House 지도를 저장한 뒤 자동 탐색기 → Nav2 → Cartographer/RViz → Gazebo 순서로 종료합니다. 키보드 조종·다른 탐색기·기록기가 남아 있으면 함께 종료합니다. House와 창고 Gazebo를 동시에 실행하지 않습니다. 기존 지도와 진단 로그를 삭제하지 않습니다.

## 1. 처음 한 번: 창고 원본 자산 받기

```bash
mkdir -p ~/ros2_practice_ws/assets

git clone --depth 1 --single-branch --branch ros2 \
  https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git \
  ~/ros2_practice_ws/assets/aws-robomaker-small-warehouse-world
```

이미 같은 폴더에 내려받았다면 clone을 반복하지 않고 다음 검사로 넘어갑니다. 기본 분기는 보관 안내문만 포함할 수 있으므로 `--branch ros2`를 사용해야 합니다. 다운로드에는 인터넷이 필요하며 월드 모델은 배포 ZIP에 포함하지 않았습니다. 저장소는 보관 상태입니다. 이번에는 그 자산을 로컬 시뮬레이션에 사용합니다.

## 2. 터미널 1: 준비와 소형 창고 Gazebo

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

python3 ~/ros2_practice_ws/prepare_small_warehouse.py

python3 ~/ros2_practice_ws/prepare_house_wheels_010.py

python3 ~/ros2_practice_ws/prepare_house_nav2_frontier.py \
  --speed 0.10 --yaw-rate 0.20 --waffle-footprint

python3 ~/ros2_practice_ws/prepare_house_revisit.py --mode mapping \
  --source "$(ros2 pkg prefix --share turtlebot3_cartographer)/config/turtlebot3_lds_2d.lua"

ros2 launch ~/ros2_practice_ws/small_warehouse_waffle_pi.launch.py
```

각 준비 명령이 성공했는지 확인한 뒤 다음 명령을 실행합니다. `prepare_small_warehouse.py`는 world·모델·메시의 로컬 URI 참조와 XML을 검사하고 원본 커밋·world 해시·경로를 `warehouse_setup.json`에 기록합니다. 원본 자산을 수정하지 않습니다. 런처는 다음을 명시합니다.

- 실행 world: `warehouse_worlds/small_warehouse_no_roof.world` (지붕 제외).
- 원본 world: `assets/aws-robomaker-small-warehouse-world/worlds/small_warehouse/small_warehouse.world` (보존).
- 모델 검색: AWS models 폴더와 설치된 TurtleBot3 models 폴더.
- 미디어 검색: AWS 원본 루트. 모델의 `file://models/...` 메시 경로를 찾는 데 필요합니다.
- 로봇: 생성된 `house_waffle_pi_010/model.sdf`.

기본 출발 위치는 창고 좌표 `(0, 0)`, yaw 0rad, z 0.01m입니다. Gazebo에서 바닥에 정상 스폰됐고 선반·상자와 겹치지 않는지 먼저 확인합니다. 이 좌표의 실제 충돌 검증은 아직 수행하지 않았습니다. 위치를 바꿀 때는 Cartographer를 시작하기 전에 런처 인자로 지정합니다.

```bash
ros2 launch ~/ros2_practice_ws/small_warehouse_waffle_pi.launch.py \
  x_pose:=0.0 y_pose:=0.0 yaw:=0.0
```

창고가 무거워 Real Time Factor가 크게 낮아진다면 Gazebo GUI 없이 실행할 수 있습니다. 이때도 RViz 지도는 사용할 수 있습니다.

```bash
ros2 launch ~/ros2_practice_ws/small_warehouse_waffle_pi.launch.py gui:=false
```

위 실행 명령들은 대안입니다. 동시에 여러 개 실행하지 않습니다. 지붕 모델을 실행 월드에서 제외하므로 Gazebo 상단에서 내부를 볼 수 있습니다. 이미 실행 중인 Gazebo에는 자동 반영되지 않으므로 준비 스크립트 실행 후 Gazebo를 다시 시작합니다.

## 3. 터미널 2: 새 창고 지도 만들기

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=true \
  cartographer_config_dir:=$HOME/ros2_practice_ws/house_revisit_config \
  configuration_basename:=house_revisit.lua
```

RViz Fixed Frame은 map입니다. House 저장 지도를 로드하거나 AMCL을 동시에 실행하지 않습니다. 이 실행에서 창고 지도를 새로 생성합니다. Gazebo가 재생 상태여야 clock·라이다가 갱신됩니다.

## 4. 터미널 3: Nav2

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_frontier.yaml
```

`Managed nodes are active`를 확인합니다. 설정 파일 이름에 House가 있어도 속도·몸체·플래너 공통 설정이며, SLAM이 제공하는 현재 창고 지도를 사용합니다.

## 5. 터미널 4: 완전 자동 관심 위치·단일 목표 탐색

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 lifecycle get /controller_server
ros2 param get /controller_server FollowPath.max_vel_x

python3 ~/ros2_practice_ws/house_nav2_frontier.py --ros-args -p use_sim_time:=true
```

기대값은 active [3], 선속도 상한 0.1입니다. Publish Point를 직접 찍지 않아도 관심 미탐색 위치를 자동 선택합니다. 이동 목표와 활성 액션은 하나씩 유지하고, 일반 탐색이 끝나도 막힌 면의 안전한 앞쪽을 재관측합니다. 벽 뒤나 선반 내부를 통과하도록 만들지는 않습니다.

RViz 표시:

- MarkerArray: `/house_frontier/markers`.
- Path는 하나만 추가하고 Topic `/house_frontier/path`, Buffer Length `1`.
- 기존 `/plan`·로컬 경로 등 다른 Path 표시 해제.
- 분홍 `EXPLORE HERE`는 자동 관심 위치, `CHECK WALL`은 막힌 면 확인 위치, 초록 화살표는 실제 이동 목표.

## 6. 망설임 관측과 지도 저장

추가 터미널에서 60초 관측:

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_practice_ws/diagnose_house_motion.py --seconds 60
```

끝에 출력되는 `~/ros2_practice_ws/diagnostics/motion_<실행시각>.json`을 보관합니다. 경로·비용지도·명령 속도·실제 속도·Nav2 경고를 함께 확인할 수 있습니다.

창고 지도를 저장할 때는 House 지도와 다른 이름을 사용합니다.

```bash
mkdir -p ~/ros2_practice_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_practice_ws/maps/small_warehouse_auto \
  --ros-args -p use_sim_time:=true
```

진행 중인 SLAM 상태를 저장·재개하는 명령이 아니라 PGM/YAML 이미지 지도 저장입니다.

## 검증 범위와 출처

현재 전체 검사 96개를 통과했습니다. 새 검사는 창고 자산 경로·모델/메시 누락·XML·원본 보존 등을 다룹니다. 실제 AWS 자산 다운로드·Gazebo 렌더링·스폰·창고 완주·복귀는 현재 Windows 환경에서 실행 검증하지 않았습니다. 먼저 창고와 로봇이 정상 표시되는지 확인하고 나서 자동 탐색을 시작합니다.

- [AWS 소형 창고 ros2 원본](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/ros2)
- [원본 창고 world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/blob/ros2/worlds/small_warehouse/small_warehouse.world)
- [자산 라이선스](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/blob/ros2/LICENSE)
