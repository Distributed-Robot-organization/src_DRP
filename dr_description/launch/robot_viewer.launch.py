from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    
    urdf_file = PathJoinSubstitution([
        FindPackageShare('dr_description'),
        'description',
        'robot.urdf.xacro'
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('dr_description'),
        'rviz',
        'viewer_robot.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='minion',
            description='Robot namespace'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file, 
                                            ' is_ignition:=false',
                                            ' robot_namespace:=', robot_namespace])
            }],
            namespace=robot_namespace
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=robot_namespace
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])