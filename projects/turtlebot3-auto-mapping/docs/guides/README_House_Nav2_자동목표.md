# 미탐색 좌표 자동 선택 → Nav2 Goal 전송

현재 0.10m/s·Waffle Pi 바퀴 설정을 포함한 전체 초기화 실행은 `README_House_010_초기화실행.md`를 참고하세요.

`house_nav2_frontier.py`는 지도에서 미탐색 경계를 찾아 Nav2 액션 목표를 보냅니다. 직접 `/cmd_vel`을 발행하지 않습니다. 모터 속도·경로 추종·장애물 회피·복구 동작은 Nav2가 수행합니다.

## 목표를 고르는 방식

1. `/map`에서 로봇과 연결된 알려진 빈 공간을 조사합니다. 점유값 0~64는 빈 공간, -1은 미탐색, 65 이상은 장애물로 구분합니다.
2. 미탐색에 접한 경계를 묶고, 경계 앞의 알려진 빈 공간에서 목표 좌표를 만듭니다. 기본 몸체 검사는 Nav2와 같은 Waffle Pi 사각 윤곽과 0.01m 여유를 사용합니다. 접근 방향·현재 방향·경계 방향 등을 검사해 몸체가 들어가는 자세를 선택하므로, 복도에서도 반경 0.30m 원이 들어갈 필요가 없습니다. 미탐색 셀 자체를 빈 공간이라고 가정하지 않습니다.
3. 목표에서 경계까지 알려진 빈 공간으로 시야가 이어지는지 검사합니다. 벽에 가린 경계를 향하는 목표는 제외합니다. 장애물에서 멈추는 가상 광선으로 경계 방향 120도 범위의 예상 미탐색 관측 면적을 계산하고, `(1+이동 거리)/(경계 방향 예상 면적+0.05)+1.2×경계까지 거리`가 작은 후보를 우선합니다. 관심 영역이 지정되면 그 위치에서 먼 경계에 추가 비용을 줍니다. 예상 면적은 아직 모르는 장애물을 포함할 수 있는 추정치이며 통행 판정은 Nav2가 수행합니다. 한 방의 경계가 사라지면 다른 연결된 경계가 다음 후보가 됩니다.
4. `/compute_path_to_pose`에 후보를 요청하고 경로가 실제 목표 근처까지 이어지는지 확인합니다. 장애물 때문에 경로가 없으면 다음 좌표를 확인합니다.
5. 경로가 확인된 좌표만 `/navigate_to_pose`로 전송합니다. 사용자가 Nav2 Goal을 지정했을 때와 같은 Nav2 액션을 사용합니다.
6. 도착 후 지도 갱신을 기다렸다가 다음 목표를 고릅니다. 주변 지도 정보가 그대로인 도착 목표는 반복 선택하지 않습니다. 도착 목표 주변은 지도가 조금 바뀌어도 60초간 재선택을 억제합니다. 실패한 목표 주변은 같은 지도 상태에서 120초간 제외합니다.

이전 노드의 문 판정이나 직접 벽 추종 코드는 이 노드의 주행 제어에 사용하지 않습니다. 문은 Nav2가 계산한 통과 가능한 경로의 일부가 됩니다. 문이 너무 좁거나 아직 지도가 연결되지 않았다면 강제로 통과하지 않습니다.

## 설치

`house_nav2_frontier_bundle.zip`을 우분투 `~/ros2_practice_ws`에 압축 해제합니다.

```text
house_nav2_frontier.py
prepare_house_nav2_frontier.py
README_House_Nav2_자동목표.md
prepare_house_revisit.py
prepare_house_slam_ab.py
README_House_미탐색과위치추정.md
```

기존 `house_env_cyclone.bash`를 사용합니다. Python 스크립트이므로 colcon build는 필요하지 않습니다.

기존 `house_gap_explorer.py`, `wall_follow_scan.py`, `house_wall_explorer.py`, explore_lite, 키보드 조종은 종료합니다. 자동 목표 노드와 RViz 수동 목표를 동시에 사용하면 목표를 서로 교체할 수 있으므로 수동 시험은 자동 탐색을 일시정지한 뒤 수행합니다.

## 터미널 역할

| 터미널 | 실행 내용 |
|---|---|
| 1 | 기존 Gazebo 유지, 재생 상태 확인 |
| 2 | 기존 Cartographer 유지 |
| 3 | 이전 직접 탐색기 종료 후 Nav2 실행 |
| 4 | 기존 bag 기록 유지 가능 |
| 5 | 새 Nav2 자동 목표 탐색기 |

### 터미널 3: Nav2 설정 생성 및 실행

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

python3 ~/ros2_practice_ws/prepare_house_nav2_frontier.py --speed 0.10 --yaw-rate 0.20 --waffle-footprint

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  autostart:=true \
  params_file:=$HOME/ros2_practice_ws/nav_house_frontier.yaml
```

설정 생성기는 기존 `nav_house_base_footprint.yaml`, `nav_explore_waffle_pi.yaml`, 설치된 Waffle Pi 기본 설정 순서로 기준 파일을 찾습니다. 원본은 보존하고 `nav_house_frontier.yaml`을 새로 만듭니다. 이미 생성된 출력이 있으면 백업합니다. 출력된 원본 경로와 생성 성공을 확인한 뒤 Nav2를 실행합니다. 원본 파일을 명시하려면 `--source /절대경로/기존설정.yaml`을 사용합니다.

생성기는 현재 시험 환경의 DWBLocalPlanner + NavfnPlanner 구성용입니다. 다른 플러그인이면 설정을 임의로 변환하지 않고 오류로 알립니다. 현재 선속도 상한은 0.10m/s, 회전 상한은 0.20rad/s입니다. 가감속을 완화하고, 저속 주행에 맞춰 정지 속도 판정값을 0.02m/s로 설정합니다. 경로 계획은 알려진 공간을 사용하고, 주기적으로 발행되는 Cartographer 지도에 맞춰 static layer의 transient-local 구독을 끕니다. AMCL·map_server를 추가 실행하지 않습니다. 바퀴 모델을 포함한 새 세션 실행은 README_House_010_초기화실행.md를 참고하세요.

### 터미널 5: Nav2 준비 확인 후 자동 목표 시작

```bash
source ~/ros2_practice_ws/house_env_cyclone.bash
export ROS_DOMAIN_ID=30

ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

세 노드가 모두 `active [3]`인 것을 확인합니다. 이어서:

```bash
python3 ~/ros2_practice_ws/house_nav2_frontier.py \
  --ros-args -p use_sim_time:=true
```

정상적인 로그 흐름:

```text
경계=...개, Nav2 확인 후보=...개, 상태=ready
미탐색 목표 경로 확인: x=..., y=...
Nav2 Goal 전송: 경로 주행과 장애물 회피는 Nav2가 수행합니다.
Nav2 이동 중: 남은 거리=...m, 복구 횟수=...
미탐색 목표 도착: 지도 갱신 후 다음 목표 선택
```

## RViz에서 좌표 보기

Fixed Frame은 `map`으로 둡니다. RViz 왼쪽 아래 Add → By topic → `/house_frontier/markers` → MarkerArray → OK로 추가합니다. 표시를 나중에 켜도 보이도록 최근 표시를 보존하며 1초 간격으로 갱신합니다. 좌표·후보가 바뀌면 오래된 마커를 지웁니다.

| 표시 | 의미 |
|---|---|
| 분홍색 EXPLORE HERE | 자동 선택한 관심 미탐색 위치 |
| 분홍색 CHECK WALL | 재관측할 막힌 면의 위치. 실제 이동 목표와 구분 |
| 주황색 CHECK 화살표 | Nav2에 도달 경로 확인 중인 목표 |
| 초록색 GOAL 화살표와 x/y 좌표 | Nav2에 전송한 현재 이동 목표 |
| 보라색 HOME 화살표 | 출발점 복귀 목표 |

마커는 지도 위로 약간 띄워 그리므로 지도에 가려지는 문제를 줄입니다. 화살표는 목표 위치·방향을 나타내며 직선 주행 경로를 뜻하지 않습니다. 실제 경로는 Nav2의 경로 표시로 확인합니다. RViz 텍스트의 글꼴 호환을 위해 짧은 영어 상태 이름을 사용합니다.

기존 `/house_frontier/goal` Pose도 사용할 수 있습니다. `/house_frontier/candidates` PoseArray에는 현재 선택 목표 하나만 넣고 비활성일 때 비웁니다. 이동 경로는 Path `/house_frontier/path` 하나만 표시하고 Buffer Length를 1로 설정하세요. 취소·도착·실패 때 경로를 비우며, 현재 목표의 상태는 MarkerArray로 확인합니다.

## 시간 갱신 중단과 정지

이전 `시뮬레이션 시간 갱신 중단`은 직접 벽 추종 방식만의 문제가 아닙니다. Gazebo 일시정지, 시간 초기화, VM 처리 지연, DDS 수신 중단을 구분해야 합니다. 새 노드는 `/clock`이 2초 동안 진행하지 않거나 센서·TF 시각이 맞지 않으면 진행 중 Nav2 목표 취소를 요청하고 자동 탐색을 일시정지합니다. 직접 0 속도를 덮어 발행하지 않습니다.

새 확인 터미널에서 같은 환경을 source한 뒤:

```bash
ros2 topic echo /clock --field clock
```

초 값이 계속 증가하는지 확인하고 Ctrl+C로 echo만 종료합니다. Gazebo 일시정지 상태에서는 재생 버튼을 누릅니다. 시각·수신 문제가 해결된 뒤 자동 탐색을 재개합니다.

```bash
ros2 topic pub --once /house_frontier/resume std_msgs/msg/Bool '{data: true}'
```

사용자 일시정지:

```bash
ros2 topic pub --once /house_frontier/resume std_msgs/msg/Bool '{data: false}'
```

Ctrl+C 종료 시 진행 중인 목표 취소를 요청하고 결과를 최대 2초간 처리합니다. 서버가 응답하지 않거나 시뮬레이션이 멈춰 취소가 끝나지 않으면 Nav2 로그와 상태를 확인합니다. Nav2 프로세스는 탐색기와 별개이므로 취소 확인 없이 수동 주행이나 다른 탐색기를 동시에 시작하지 않습니다.

## 완료와 출발점 복귀

전체 실행 시간 제한은 없습니다. 개별 Nav2 목표에는 기본 600초의 실제 시간 응답 제한이 있습니다. Nav2 자체의 진행 감시도 유지됩니다.

로봇과 연결된 알려진 공간에 미탐색 경계가 없고, 지도 면적이 안정된 상태가 시뮬레이션 시간 60초 이상·서로 다른 지도 갱신 3회 이상 유지되면 막힌 면 재관측 단계로 전환합니다. 문 이외의 알려진 장애물 면도 구간별로 골라 안전한 앞쪽 목표를 하나씩 방문한 뒤 복귀합니다. 도달 실패는 미확인으로 기록하고 로그로 알립니다. 미탐색 경계가 여전히 남아 있거나 안전한 관측 후보조차 없으면 완료로 판정하지 않습니다. 출발점은 odom 좌표로 보관하고 최신 map 변환으로 재투영합니다. 벽 너머 모든 공간의 완전 탐색을 증명하는 판정은 아닙니다.

미탐색 경계가 남지만 안전한 후보나 Nav2 경로가 없는 경우는 완료가 아닙니다. 새 지도를 기다리고 다음 후보를 확인합니다. 복귀 경로나 복귀 주행이 실패하면 일시정지하고 원인을 알립니다.

사용자 복귀 요청:

```bash
ros2 topic pub --once /house_frontier/return_home std_msgs/msg/Bool '{data: true}'
```

진단 파일:

```text
/home/jjcs/ros2_practice_ws/house_nav2_frontier_state.json
```

## 검증

이 모듈의 검사 46개와 전체 유지 검사 93개를 통과했습니다. 좁은 복도 출발, 관심 영역·경계 방향 정보량·도착 방향 선택, 통과 불가능한 통로 거부, 몸체 방향·지도 회전, 실제 제공 지도에서의 좌표 생성, 완료 안정성, Nav2 설정과 표시 구분을 검사했습니다. 실제 ROS/Nav2 액션 통신·Gazebo 주행과 RViz 렌더링은 이 Windows 환경에서 실행 검증하지 못했습니다.

사용한 Humble 인터페이스:

- [ComputePathToPose 액션](https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/ComputePathToPose.action)
- [NavigateToPose 액션](https://github.com/ros-navigation/navigation2/blob/humble/nav2_msgs/action/NavigateToPose.action)
- [Nav2 Humble 기본 파라미터](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/params/nav2_params.yaml)
