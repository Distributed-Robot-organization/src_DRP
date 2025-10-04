import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return


def check_exists(context):
    if not os.path.exists(context.launch_configurations['map_file']):
        raise Exception("[{}] Map file `{}` does not exist".format(__file__, context.launch_configurations['map_file']))
    
    if not os.path.exists(context.launch_configurations['nav2_params_file']):
        raise Exception("[{}] Nav2 parameters file `{}` does not exist".format(__file__, context.launch_configurations['nav2_params_file']))
    
    if context.launch_configurations['use_rviz'] == "true" and \
        not os.path.exists(context.launch_configurations['rviz_config_file']):
        raise Exception("[{}] Rviz configuration `{}` does not exist".format(__file__, context.launch_configurations['rviz_config_file']))

    return


def generate_launch_description():
    # Get package directories
    dr_nav2_dir = get_package_share_directory('dr_nav2')
    
    # Declare launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='pollo')
    map_file = LaunchConfiguration('map_file', default=os.path.join(dr_nav2_dir, 'maps', 'plane_2.yaml'))
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=os.path.join(dr_nav2_dir, 'config', 'nav2_params.yaml'))
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(dr_nav2_dir, 'rviz', 'nav2_default_view.rviz'))
    nav2_autostart = LaunchConfiguration('nav2_autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Initial pose parameters
    set_initial_pose = LaunchConfiguration('set_initial_pose', default='false')
    initial_x = LaunchConfiguration('initial_x', default='1.0')
    initial_y = LaunchConfiguration('initial_y', default='1.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='0.0')
    
    # Remap lifecycle nodes to robot namespace
    map_node = PythonExpression(["'/", robot_name, '/map_server', "'"])
    amcl_node = PythonExpression(["'/", robot_name, '/amcl', "'"])
    bt_navigator_node = PythonExpression(["'/", robot_name, '/bt_navigator', "'"])
    controller_node = PythonExpression(["'/", robot_name, '/controller_server', "'"])
    planner_node = PythonExpression(["'/", robot_name, '/planner_server', "'"])
    behavior_node = PythonExpression(["'/", robot_name, '/behavior_server', "'"])
    velocity_smoother_node = PythonExpression(["'/", robot_name, '/velocity_smoother', "'"])

    lifecycle_nodes_loc = [
        [map_node],
        [amcl_node]
    ]

    lifecycle_nodes_nav = [
        [bt_navigator_node],
        [controller_node], 
        [planner_node], 
        [behavior_node], 
        [velocity_smoother_node]
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = { 
        'use_sim_time': use_sim_time,
        'base_frame_id': PythonExpression(["'", robot_name, '/base_link', "'"]),
        'odom_frame_id': PythonExpression(["'", robot_name, '/odom', "'"]),
        'robot_base_frame': PythonExpression(["'", robot_name, '/base_link', "'"]),
        'global_frame': PythonExpression(["'",'map', "'"]),
        'topic': PythonExpression(["'/", robot_name, '/scan', "'"]),
        'x': initial_x,
        'y': initial_y,
        'yaw': initial_yaw,
        'set_initial_pose': set_initial_pose,
        'yaml_filename': map_file
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # Nodes
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        namespace=robot_name,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'topic_name': "/map"},
            {'frame_id': "map"},
            {'yaml_filename': map_file}
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        namespace=robot_name,
        respawn=True,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[configured_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        namespace=robot_name,
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        namespace=robot_name,
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        namespace=robot_name,
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        namespace=robot_name,
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        namespace=robot_name,
        respawn=True,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'), 
            ('cmd_vel_smoothed', 'cmd_vel')
        ],
    )
    
    # Lifecycle manager for map_server and amcl
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        namespace=robot_name,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': nav2_autostart},
            {'bond_timeout': 0.0},
            {'node_names': lifecycle_nodes_loc}
        ],
    )
    
    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        namespace=robot_name,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': nav2_autostart},
            {'bond_timeout': 0.0},
            {'node_names': lifecycle_nodes_nav}
        ],
    )

    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name,
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        OpaqueFunction(function=print_env),
        
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value=use_sim_time,
            choices=['true', 'false'],
            description='Flag to toggle between real robot and simulation'
        ),
        DeclareLaunchArgument(
            name='robot_name', 
            default_value=robot_name,
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            name='map_file', 
            default_value=map_file,
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            name='nav2_params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RVIZ config file to use'
        ),
        DeclareLaunchArgument(
            name='nav2_autostart',
            default_value=nav2_autostart,
            choices=['true', 'false'],
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            name='use_rviz', 
            default_value=use_rviz,
            choices=['true', 'false'],
            description='Whether to start RViz'
        ),
        DeclareLaunchArgument(
            name='set_initial_pose',
            default_value=set_initial_pose,
            choices=['true', 'false'],
            description='Flag to enable sending initial pose to AMCL'
        ),
        DeclareLaunchArgument(
            name='initial_x',
            default_value=initial_x,
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_y',
            default_value=initial_y,
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_yaw',
            default_value=initial_yaw,
            description='Initial yaw position of the robot'
        ),
        
        OpaqueFunction(function=print_env),
        OpaqueFunction(function=check_exists),
        
        # Add the nodes
        map_server,
        amcl,
        bt_navigator,
        controller_server,
        planner_server,
        behavior_server,
        velocity_smoother,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        rviz_node,
    ])
