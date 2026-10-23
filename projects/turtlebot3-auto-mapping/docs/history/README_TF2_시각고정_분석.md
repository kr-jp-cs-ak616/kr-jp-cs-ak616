# House 윤곽 시각 고정 보고서 분석

> 보존용 분석 기록입니다. 현재 실행에는 [최신 실행 안내](../guides/README_House_010_초기화실행.md)를 사용하세요. 관측 원본은 [진단 자료](../evidence/house_state_report_2026-10-21.json), 수집 도구는 [collect_house_state.py](../../tools/diagnostics/collect_house_state.py)에 보존했습니다.

분석일: 2026-10-21. 자료: 사용자가 제공한 house_state_report.txt.

## 보고서에서 확인한 사실

20초 관측 동안 다음 데이터는 계속 갱신됐다.

| 항목 | 처음 시각 | 마지막 시각 | 표본 수 |
|---|---:|---:|---:|
| /clock | 1293.8 | 1313.4 | 197 |
| /scan | 1293.801 | 1313.401 | 99 |
| /odom | 1293.807 | 1313.459 | 579 |
| TF odom → base_footprint | 1293.807 | 1313.425 | 578 |
| TF map → odom | 1293.8 | 1313.401 | 292 |
| /local_costmap/costmap | 1293.8 | 1313.3 | 33 |
| /local_costmap/published_footprint | 126.043 | 126.043 | 100 |

로봇 윤곽은 미발행이 아니라 5Hz 정도로 계속 발행되며, 시각만 과거에 고정돼 있다. 모든 확인된 스트림에서 시각 역행은 0회다. 신규 관측 노드에서는 odom→base_footprint, odom→base_link, map→odom을 현재 시각 부근으로 조회했다. 외부 TF 발행 전체가 멈춘 상황과 다르다.

/clock 발행자는 Gazebo 1개, 윤곽 발행자는 local_costmap 1개다. /tf 발행자는 robot_state_publisher, turtlebot3_diff_drive, cartographer_node로 구성돼 있다. 수집된 TF 부모 관계에서 명확한 중복 부모는 확인되지 않았다. 단, 이 수집 방식은 동적 TF 메시지를 발행자별로 분리하지 않으므로 동일 부모·자식 쌍의 복수 발행을 완전히 배제하는 자료는 아니다.

실제 실행 환경은 Cyclone DDS, ROS_DOMAIN_ID=30이다. Nav2·비용 지도·Cartographer는 use_sim_time=true이다. 로컬 비용 지도는 odom/base_footprint, 반경0.28m, 5Hz 갱신, obstacle_layer+voxel_layer+inflation_layer다. 보고서에 포함된 apt 설치 버전은 다음과 같다.

- Nav2 1.1.20
- tf2 / tf2_ros 0.25.23
- rclcpp 16.0.21
- rmw_cyclonedds_cpp 1.3.5

이는 설치 패키지 정보이며 프로세스가 로드한 모든 공유 라이브러리 경로까지 검증한 것은 아니다.

## 유력한 원인: tf2 0.25.23 스레드 교착 결함

ROS geometry2 공식 저장소 이슈 #992는 Ubuntu22.04/Humble에서 tf2 0.25.23의 waitForTransform과 testTransformableRequests 사이 잠금 순서 문제를 보고한다. Nav2 LaserScan MessageFilter와 TF listener의 서로 다른 스레드가 관련되고, TF 수신·cleanup 등이 정체될 수 있다. 같은 이슈의 댓글에는 사용자 보고서와 같은 rclcpp16.0.21, Cyclone DDS1.3.5 조합에서 RViz 응답 중단도 보고됐다.

본 보고서의 '외부 TF 정상, 특정 Nav2 버퍼를 사용하는 윤곽 시각 고정, RMW 변경 후에도 재발, RViz 응답 중단'은 이 결함과 강하게 부합한다. 다만 Nav2 프로세스의 스레드 역추적을 수집하지 않았으므로 교착 자체를 확정했다고 표현하지 않는다. 반경 축소, 미탐색 허용, TF 허용 시간 증가로 해결할 문제라고 볼 근거는 없다.

공식 tf2와 tf2_ros의 0.25.24 변경 이력(2026-09-15)에는 해당 ABBA 교착 수정이 명시돼 있다. 소스 태그가 존재한다는 사실과 사용자의 apt 저장소에 바이너리가 제공되는지는 다르므로, 먼저 apt 후보 버전을 확인해야 한다.

## 다음 조치

자동 탐색 및 회전은 정지 상태를 유지한다. 새 코드나 추가 설정 변경은 하지 않는다. Ubuntu에서:

```bash
sudo apt update
apt-cache policy ros-humble-tf2 ros-humble-tf2-ros
```

Installed와 Candidate를 확인한다. 두 패키지 모두 수정판 0.25.24 이상이 정상 Humble 저장소에 있다면, 필요한 미저장 지도를 보존하고 모든 ROS 프로세스 종료 후 두 패키지를 함께 갱신한다.

```bash
sudo apt install --only-upgrade ros-humble-tf2 ros-humble-tf2-ros
dpkg-query -W ros-humble-tf2 ros-humble-tf2-ros
```

새 라이브러리를 사용하려면 기존 Gazebo·Nav2·SLAM·RViz 프로세스를 모두 다시 시작해야 한다. 기존 colcon 워크스페이스를 이번 apt 업데이트 때문에 무조건 재빌드할 필요는 없다. 직접 빌드한 tf2/Nav2 오버레이가 있다면 apt 설치 버전과 실제 로드 경로가 다를 수 있으므로 별도로 확인한다.

Candidate가 0.25.23이면 같은 패키지 재설치만 반복하지 않는다. 출력 결과를 보관한다. testing 저장소 전환이나 소스 오버레이 빌드는 후보 확인 후 별도로 계획하며 자동으로 권장 설정에 섞지 않는다. 소스 빌드를 선택하면 그 단계에서는 colcon build가 필요하다.

업데이트 후 검증: 동일 설정 유지 → 1분 이상 윤곽 시각 관측 → 최소 수분 동안 시각 고정 재발 여부 확인 → 30도 Spin의 TF 오류 확인 → 수동 단거리 목표 → 자동 탐색. 시각 문제 해결이 실제 장애물 기하 문제까지 자동 해결한다는 뜻은 아니다. 55도 회전 근사 검사에서 검출된 장애물 값은 시간 동기 복구 뒤 별도로 확인해야 한다.

## 근거

- 결함 보고: https://github.com/ros2/geometry2/issues/992
- tf2 수정 이력: https://github.com/ros2/geometry2/blob/0.25.24/tf2/CHANGELOG.rst
- tf2_ros 수정 이력: https://github.com/ros2/geometry2/blob/0.25.24/tf2_ros/CHANGELOG.rst
- Nav2 1.1.20 윤곽 생성: https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp
