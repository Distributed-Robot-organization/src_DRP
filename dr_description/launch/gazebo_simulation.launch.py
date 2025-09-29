import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load_initial_pose_from_yaml():
    config_pkg_path = get_package_share_directory('dr_configurations')
    yaml_path = os.path.join(config_pkg_path, 'robottino_behavior.yaml')
    
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
    
    pose = data.get('initial_pose', {})
    return str(pose.get('x', 0.0)), str(pose.get('y', 0.0)), str(pose.get('yaw', 0.0))


def generate_launch_description():
    package_name = 'dr_description'
    robot_namespace = LaunchConfiguration('robot_namespace')
    
    x_pose, y_pose, yaw = load_initial_pose_from_yaml()

    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'plane_2.world'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true',
            'robot_namespace': robot_namespace
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
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gazebo_params.yaml'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
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

    # Delay spawning controllers to ensure Gazebo is fully loaded
    delayed_diff_drive_spawner = TimerAction(period=2.0, actions=[diff_drive_spawner])
    delayed_joint_broad_spawner = TimerAction(period=2.0, actions=[joint_broad_spawner])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='minion',
            description='Robot namespace'
        ),
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
    ])
