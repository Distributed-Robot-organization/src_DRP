import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def load_robots_from_yaml():
    config_pkg_path = get_package_share_directory('dr_description')
    yaml_path = os.path.join(config_pkg_path, 'config', 'minions_params.yaml')
    
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)
        
    robots = data.get('/**', {}).get('ros__parameters', {}).get('robots', [])
    return robots if robots else [{'name': 'robot1', 'x': 1.0, 'y': 1.0, 'yaw': 0.0}]


def generate_launch_description():
    package_name = 'dr_description'
    robots = load_robots_from_yaml()
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )
    
    launch_nodes = []
    

    # Crea un robot state publisher per ogni robot con namespace
    for robot in robots:
        robot_name = robot.get('name', 'robot1')
        x    = str(robot.get('x',   1.0))
        y    = str(robot.get('y',   1.0))
        yaw  = str(robot.get('yaw', 0.0))
        
        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory(package_name),
                    'launch',
                    'multi_rsp.launch.py'
                )
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'use_ros2_control': 'false',  # Use gazebo_control instead for multi-robot
                'namespace': robot_name
            }.items()
        )
        
        # Spawn entity per ogni robot
        print(f"Spawning {robot_name} at x={robot.get('x')}, y={robot.get('y')}, yaw={robot.get('yaw')}")
        twist_mux = Node(
                    package    = 'twist_mux',
                    executable = 'twist_mux',
                    name=f'{robot_name}_twist_mux',
                    namespace  = robot_name,
                    parameters = [twist_mux_params, {'use_sim_time': True}],
                    remappings = [('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
                )
        
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'{robot_name}_spawn_entity',
            arguments=[
                '-topic', f'{robot_name}/robot_description',
                '-entity', robot_name,
                '-x', str(robot.get('x', 1.0)),
                '-y', str(robot.get('y', 1.0)),
                '-z', '0.01',
                '-Y', str(robot.get('yaw', 0.0))
            ],
            output='screen'
        )
        control_params = os.path.join(
            get_package_share_directory(package_name),
            'config',
            'my_controllers.yaml'  # <-- assicurati che esista
        )

        controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            name=f'{robot_name}_controller_manager',
            namespace=robot_name,
            parameters=[
                {'use_sim_time': True},
                control_params
            ],
            output='screen'
        )
        
        diff_drive_spawner = Node(
            package    = 'controller_manager',
            executable = 'spawner',
            name=f'{robot_name}_diff_drive_spawner',
            namespace  = robot_name,
            arguments  = [
                'diff_cont',
                '-c', f'/{robot_name}/controller_manager'   # controller_manager namespaced
            ]
        )

        joint_broad_spawner = Node(
            package    = 'controller_manager',
            executable = 'spawner',
            name=f'{robot_name}_joint_broad_spawner',
            namespace  = robot_name,
            arguments  = [
                'joint_broad',
                '-c', f'/{robot_name}/controller_manager'
            ]
        )
        launch_nodes.extend([
            rsp,
            twist_mux,
            spawn_entity,
            controller_manager_node, 
            joint_broad_spawner,
            diff_drive_spawner
            ])

    return LaunchDescription(launch_nodes)

