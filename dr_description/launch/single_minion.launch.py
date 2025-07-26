import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def load_initial_pose_from_yaml():
    config_pkg_path = get_package_share_directory('dr_description')
    yaml_path = os.path.join(config_pkg_path, 'config', 'minions_params.yaml')
    
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
        
    robots = data.get('/**', {}).get('ros__parameters', {}).get('robots', [])
    if robots:
        first_robot = robots[0]
        return str(first_robot.get('x', 1.0)), str(first_robot.get('y', 1.0)), str(first_robot.get('yaw', 0.0))
    else:
        return "1.0", "1.0", "0.0"


def generate_launch_description():
    package_name = 'dr_description'

    
    x_pose, y_pose, yaw = load_initial_pose_from_yaml()


    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'single_rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'minion',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    ])
