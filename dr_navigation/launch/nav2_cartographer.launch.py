from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    slam_toolbox_launch_dir = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch')
    
    pkg_dir = get_package_share_directory('dr_navigation')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # RViz config file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'cartographer_rviz2.rviz')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml'),
            description='Full path to the slam_toolbox parameters file.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true.'
        ),

        # Include slam_toolbox's online_async_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
