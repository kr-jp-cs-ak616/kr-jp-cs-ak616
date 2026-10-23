"""House 환경에 검증 후 생성한 Waffle Pi SDF를 명시적으로 스폰한다."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    model = Path.home()/'ros2_practice_ws/house_waffle_pi_010/model.sdf'
    if not model.is_file():
        raise RuntimeError('먼저 prepare_house_wheels_010.py를 실행하세요: '+str(model))
    print('이번에 실제 스폰할 모델:', model)
    return LaunchDescription([
        # 사용자가 보낸 상태 파일의 origin_odom(WORLD 기준)을 재사용한다.
        DeclareLaunchArgument('x_pose', default_value='1.676'),
        DeclareLaunchArgument('y_pose', default_value='0.401'),
        DeclareLaunchArgument('yaw', default_value='-0.451'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzserver.launch.py')),
                                 launch_arguments={'world': str(tb3/'worlds/turtlebot3_house.world')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzclient.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(tb3/'launch/robot_state_publisher.launch.py')),
                                 launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'waffle_pi', '-file', str(model),
                        '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
                        '-z', '0.01', '-Y', LaunchConfiguration('yaw')])
    ])
