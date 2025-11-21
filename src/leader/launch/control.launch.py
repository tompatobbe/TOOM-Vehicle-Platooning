import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- NODE 1 CONFIGURATION ---
    node1 = Node(
        package='joy',       # The name of the package where the node is defined
        executable='joy_node', # The name of the executable (defined in setup.py or CMakeLists.txt)
        name='joy_name',      # (Optional) Overwrite the node name
        output='screen',                # Logs output to the console
        parameters=[
            {'param_name': 'value'}     # (Optional) Dictionary for parameters
        ]
    )

    # --- NODE 2 CONFIGURATION ---
    node2 = Node(
        package='leader',       # Can be the same package as Node 1
        executable='stick_driver',
        name='stick_driver',
        output='screen',
        # remappings=[                  # (Optional) Remap topics if necessary
        #     ('/old_topic', '/new_topic')
        # ]
    )

    # --- RETURN LAUNCH DESCRIPTION ---
    return LaunchDescription([
        node1,
        node2
    ])