"""단순 상자 창고와 Waffle Pi를 실행한다. 기본으로 Gazebo 창을 연다."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ws = Path.home()/'ros2_practice_ws'
    world = ws/'warehouse_worlds/simple_warehouse.world'
    model = ws/'house_waffle_pi_010/model.sdf'
    if not world.is_file() or not model.is_file():
        raise RuntimeError('먼저 prepare_simple_warehouse.py와 prepare_house_wheels_010.py를 실행하세요.')
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    print('단순 창고 월드:', world)
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(
            [str(tb3/'models'), os.environ.get('GAZEBO_MODEL_PATH', '')])),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-3.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzserver.launch.py')),
                                 launch_arguments={'world': str(world)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzclient.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('gui'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(tb3/'launch/robot_state_publisher.launch.py')),
                                 launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'waffle_pi', '-file', str(model),
                        '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
                        '-z', '0.01', '-Y', LaunchConfiguration('yaw')]),
    ])
