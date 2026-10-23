# House 지도 왜곡: 지역 정합과 전체 최적화 분리

> 원인 분리 시험의 보존 기록입니다. 전역 최적화를 끄는 설정은 현재 기본 프로필이 아닙니다. 현재 실행에는 [최신 실행 안내](../guides/README_House_010_초기화실행.md)를 사용하세요.

## 이번 단계의 근거

접촉 없는 빈 월드 측정에서는 반경이 좌 0.033006m, 우 0.032994m, 유효 축간거리가 0.288013m로 형상값과 잘 맞았습니다. 거리 잔차 약 0.114mm, 회전 잔차 약 0.0028도였습니다. 이 결과는 정상 접촉 조건에서의 바퀴 기구학에 큰 오차가 있다는 가설을 지지하지 않습니다. House의 접촉·헛돎이나 센서 문제까지 배제하지는 않습니다.

사용자 화면에는 벽이 여러 겹으로 어긋나 있습니다. RViz 카메라 각도와 로봇 표시 문제만으로 처리하지 않고, SLAM 추정 결과를 원인 분리합니다. 원인이 확정된 상태는 아닙니다.

공식 Cartographer 튜닝 문서에서는 지역 SLAM 문제를 분리할 때 `POSE_GRAPH.optimize_every_n_nodes = 0`으로 전체 최적화를 중지하는 방법을 설명합니다.

https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html

## 1. 비교 설정 생성

Windows의 `prepare_house_slam_ab.py` 하나를 Ubuntu의 `~/ros2_practice_ws/`에 복사합니다. 빌드는 필요하지 않습니다.

탐색기는 종료하거나 정지하고, **기존 Cartographer는 아직 실행 중인 상태**에서 다음을 실행합니다. 이때 Gazebo를 재생하면 센서 상태를 관찰할 수 있지만, 설정 생성 자체는 센서 입력을 사용하지 않습니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
python3 ~/ros2_practice_ws/prepare_house_slam_ab.py
```

`/proc`에서 실행 중인 Cartographer의 실제 Lua 경로를 확인해 다음을 생성합니다.

```text
~/ros2_practice_ws/house_slam_ab_config/house_baseline.lua
~/ros2_practice_ws/house_slam_ab_config/house_local_only.lua
~/ros2_practice_ws/house_slam_ab_config/comparison_source.json
```

원본은 수정하지 않습니다. 비교 파일에서 전체 최적화만 중지합니다. IMU 사용 여부도 현재 실행 설정 그대로입니다. 기존 출력 폴더가 있으면 백업을 먼저 만듭니다. Cartographer가 0개 또는 여러 개이면 자동으로 원본을 추측하지 않습니다.

## 2. 새 지도에서 비교

1. 탐색기 터미널에서 Ctrl+C로 완전히 종료합니다. 정지 상태의 탐색기도 0 속도를 계속 보낼 수 있습니다.
2. 기존 Cartographer를 Ctrl+C로 종료하고 함께 열린 RViz도 닫습니다.
3. Gazebo는 유지합니다. 시간을 초기화하거나 로봇을 순간 이동시키지 않습니다. Nav2와 키보드 조종기는 종료합니다.
4. Cartographer 터미널에서 다음을 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true cartographer_config_dir:=$HOME/ros2_practice_ws/house_slam_ab_config configuration_basename:=house_local_only.lua
```

이전 지도를 불러오지 않고 새 지도를 만듭니다. RViz에서 Fixed Frame은 `map`, 필요하면 Map의 Draw Behind를 켜고 Views의 Target Frame을 `base_footprint`로 설정합니다. 카메라가 로봇 방향을 따라 돌면 지도 전체가 화면에서 회전할 수 있으므로, 지도 내부의 벽끼리 상대적으로 어긋나는지 봅니다.

## 3. 재현 구간 기록

나중에 실제 센서 자료를 분석할 수 있도록 주행 전에 별도 터미널에서 기록을 시작합니다. 카메라 영상은 기록하지 않습니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
mkdir -p ~/ros2_practice_ws/bags
ros2 bag record -o "$HOME/ros2_practice_ws/bags/house_local_$(date +%Y%m%d_%H%M%S)" /clock /scan /odom /imu /tf /tf_static /cmd_vel /joint_states
```

다른 터미널에서 같은 저속 벽 추종을 실행합니다.

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
python3 ~/ros2_practice_ws/wall_follow_scan.py --ros-args -p use_sim_time:=true -p max_runtime:=0.0 -p speed:=0.04 -p max_yaw_rate:=0.10
```

벽이 새로 겹치거나 로봇이 장애물에 닿으면 탐색기를 Ctrl+C로 종료한 뒤, 기록 터미널도 Ctrl+C로 정상 종료합니다. 왜곡이 발생한 시점, 접촉 여부, 화면을 함께 기록합니다. rosbag2 명령이 설치되지 않았거나 기록 오류가 나면 오류를 확인하고, 기록된 것으로 간주하지 않습니다.

기록에는 기존 SLAM의 `map -> odom`도 포함됩니다. 새 Cartographer와 바로 함께 재생하면 TF가 중복될 수 있으므로 이후 재생 분석 때 해당 연결을 제거하거나 remap/필터링해야 합니다. 현재 단계는 기록만 합니다.

## 해석과 복원

- 전체 최적화를 중지해도 같은 벽이 어긋나면 지역 스캔 정합, 센서/TF 일치, 주행 접촉 등을 더 조사해야 합니다.
- 왜곡이 줄어들면 전체 최적화나 루프 폐쇄의 영향을 의심할 근거가 됩니다. 다른 주행 경로·다른 시작점의 시험만으로 원인을 확정하지 않습니다. 같은 센서 기록으로 재현 비교하면 더 정확히 구분할 수 있습니다.
- 이 설정에서는 전체 지도 보정이 제한되므로 누적 오차가 남을 수 있습니다. 최종 매핑 설정으로 확정하지 않습니다.

원래 설정으로 복원하려면 탐색기와 Cartographer를 종료하고 새 지도에서 다음을 실행합니다.

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true cartographer_config_dir:=$HOME/ros2_practice_ws/house_slam_ab_config configuration_basename:=house_baseline.lua
```

Windows의 순수 Python 검사 53개와 문법 검사는 통과했습니다. 실제 Ubuntu launch, 지도 비교 및 원인 확인은 아직 수행하지 않았습니다.
