# 기존 house_env.bash는 그대로 보존한다. 이 파일은 별도 통신 분리 시험용이다.
source /opt/ros/humble/setup.bash || return 1
source "$HOME/ros2_practice_ws/install/setup.bash" || return 1
if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
  echo '먼저 sudo apt install ros-humble-rmw-cyclonedds-cpp 를 실행하세요.'
  return 1
fi
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS CYCLONEDDS_URI
export TURTLEBOT3_MODEL=waffle_pi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo 'House 통신 분리 시험: Waffle Pi / Cyclone DDS'
