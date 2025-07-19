from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0'
            }]
        ),
        
        Node(
            package='joy_manager',
            executable='joystick_radiomaster_pocket',
            name='radiomaster_joystick_controller',
            output='screen',
            parameters=[{
                'rpyt_topic': 'joystick_radiomaster/rpyt',
                'gear_S1_topic': 'joystick/gear_S1',
                'switch_SC_topic': 'joystick/switch_SC',
                'switch_SB_topic': 'joystick/switch_SB',
                'button_SD_topic': 'joystick/SD',
                'button_SA_topic': 'joystick/SA',
                'button_SE_topic': 'capture_trigger',
                'button_SAB_topic': 'joystick/SAB'
            }]
        )
    ])