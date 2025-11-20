from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05
            }]                  
        ),
        Node(
            package='ds4',
            executable='ds4_print', 
            name='ds4_print',
            parameters=[{
            }]                  
        ),
        
    ]) 