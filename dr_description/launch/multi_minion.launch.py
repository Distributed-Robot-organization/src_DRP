import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PythonExpression


def start_minions():
    nodes = []

    config_pkg_path = get_package_share_directory('dr_description')
    yaml_path = os.path.join(config_pkg_path,'config' , 'minions_params_file.yaml')

    with open(yaml_path, 'r') as file:
        raw_yaml = yaml.safe_load(file)
        minions_params_file = raw_yaml["/**"]["ros__parameters"]

        for i in range(len(minions_params_file['init_names'])):
            minion_name = minions_params_file['init_names'][i]
            minion_pose_x = minions_params_file['init_x'][i]
            minion_pose_y = minions_params_file['init_y'][i]
            minion_pose_yaw = minions_params_file['init_yaw'][i]

            print(f"Spawning {minion_name} at ({minion_pose_x}, {minion_pose_y}, {minion_pose_yaw})")

            # Include rsp launch filez
            rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        config_pkg_path,
                        'launch',
                        'multi_rsp.launch.py'
                    )
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'minion_name': minion_name,
                    'use_ros2_control': 'true'
                }.items()
            )

            # Spawning in Gazebo
            spawn_entity = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', PythonExpression(["'/", minion_name, "/robot_description", "'"]),
                    '-entity', minion_name,
                    '-robot_namespace', minion_name,
                    '-x', PythonExpression(["'", str(minion_pose_x), "'"]),
                    '-y', PythonExpression(["'", str(minion_pose_y), "'"]),
                    '-z', '0.0',
                    '-Y', PythonExpression(["'", str(minion_pose_yaw), "'"]),
                ],
                output='screen'
            )

            # Controller spawner (opzionale)
            diff_drive_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont", "--robot_namespace", minion_name],
                namespace=minion_name
            )

            joint_broad_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad", "--robot_namespace", minion_name],
                namespace=minion_name
            )

            # Twist mux (global)
            twist_mux_params = os.path.join(
                config_pkg_path,
                'config',
                'twist_mux.yaml'
            )

            twist_mux = Node(
                package="twist_mux",
                executable="twist_mux",
                parameters=[twist_mux_params, {'use_sim_time': True}],
                remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
                namespace=minion_name
            )

            nodes += [
                rsp, 
                spawn_entity, 
                diff_drive_spawner, 
                joint_broad_spawner,
                twist_mux
            ]

    return nodes


def generate_launch_description():   

    config_pkg_path = get_package_share_directory('dr_description')

    # World path
    world = os.path.join(
        config_pkg_path,
        'worlds',
        'empty.world'
    )

    # Gazebo params
    gazebo_params_file = os.path.join(
        config_pkg_path,
        'config',
        'gazebo_params.yaml'
    )

    # Launch Gazebo
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

    minions_nodes = start_minions()

    return LaunchDescription([
        gazebo,
        *minions_nodes
    ])