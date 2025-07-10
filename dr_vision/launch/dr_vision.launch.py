from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    #launch node build_dataset.py
    build_test = Node(
        package='dr_vision',
        executable='build_dataset.py',
        name='image_capture_node',
        output='screen'
    )
    
    return LaunchDescription([
        # build_test,
        
    ])