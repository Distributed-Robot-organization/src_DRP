import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    
    pkg_dir = get_package_share_directory('dr_navigation')
    default_map_path = os.path.join(pkg_dir, 'maps', 'map_slam_2', 'plane_2.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    map_sub_arg = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='true',
        description='Enable transient local QoS for map subscription'
    )

    # Include the localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                pkg_dir,
                'launch',
                'localization_launch.py'
            )
        ]),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    nav2_params_path = os.path.join(pkg_dir, 'nav2_params.yaml')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                pkg_dir,
                'launch',
                'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_subscribe_transient_local': LaunchConfiguration('map_subscribe_transient_local'),
            'params_file': nav2_params_path
        }.items()
    )

    # Launch rviz2 with the specified configuration
    rviz_cmd = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            os.path.join(
                pkg_dir,
                'rviz',
                'nav2_rviz2.rviz'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        map_sub_arg,
        localization_launch,
        navigation_launch,
        rviz_cmd
    ])
