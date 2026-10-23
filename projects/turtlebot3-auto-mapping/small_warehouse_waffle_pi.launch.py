"""AWS 소형 창고 원본 월드에 기존 0.10m/s 프로필의 Waffle Pi를 스폰한다."""
import json
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
    settings = ws/'warehouse_setup.json'
    model = ws/'house_waffle_pi_010/model.sdf'
    if not settings.is_file() or not model.is_file():
        raise RuntimeError('먼저 prepare_small_warehouse.py와 prepare_house_wheels_010.py를 실행하세요.')
    info = json.loads(settings.read_text(encoding='utf-8'))
    if not info.get('roof_removed'):
        raise RuntimeError('지붕 없는 월드를 생성하려면 최신 prepare_small_warehouse.py를 다시 실행하세요.')
    world, models, source = Path(info['world']), Path(info['models']), Path(info['source'])
    if not world.is_file() or not models.is_dir():
        raise RuntimeError('창고 자산 경로가 없어졌습니다. prepare_small_warehouse.py를 다시 실행하세요.')
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    model_path = os.pathsep.join([str(models), str(tb3/'models'), os.environ.get('GAZEBO_MODEL_PATH', '')])
    resource_path = os.pathsep.join([str(source), str(tb3), os.environ.get('GAZEBO_RESOURCE_PATH', '')])
    print('소형 창고 월드:', world)
    print('실제 Waffle Pi 스폰 모델:', model)
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resource_path),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='false' if info.get('vm_lite') else 'true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzserver.launch.py')),
                                 launch_arguments={'world': str(world)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gzclient.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('gui'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(tb3/'launch/robot_state_publisher.launch.py')),
                                 launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', 'waffle_pi', '-file', str(model),
                        '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
                        '-z', '0.01', '-Y', LaunchConfiguration('yaw')])
    ])
