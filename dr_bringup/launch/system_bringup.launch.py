"""
@file dr_system_bringup.launch.py
@brief Launch file to bring up the full CR robotic system in stages.

This script sequentially launches various subsystems of the robot of each pkg, with 2 second delay for each other.
Launch file are:
1- Moveit
2- Scene management
3- Vision
4- GUI
5- Alexa integration
6- Behavior Tree orchestrator
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def include(pkg, launch_file, delay=0.0, **kwargs):
    """Utility: include <pkg>/launch/<launch_file> con eventuale ritardo."""
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', launch_file])
        ),
        launch_arguments=kwargs.items()
    )
    return GroupAction([TimerAction(period=delay, actions=[action])]) if delay else action


def generate_launch_description():
    return LaunchDescription([
        include('dr_description', 'gazebo_simulation.launch.py', delay=0.0),
                
        # include('dr_vision', 'dr_vision.launch.py', delay=2.0),

        include('dr_nav2', 'bringup_nav2.launch.py', delay=4.0),
        

    ])