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
    #detection.py node
    detection_node = Node(
        package='dr_vision',
        executable='detection.py',  
        name='object_detection_node',
        output='screen',
        parameters=[{
            'rgb_image_topic': 'camera/image_raw',
            'depth_image_topic': 'camera/depth/image_raw'
        }]
    )
    
    return LaunchDescription([
        # build_test, #not possible to save in right folder using ros2
        detection_node
    ])
    
    #ros2 run teleop_twist_keyboard teleop_twist_keyboard