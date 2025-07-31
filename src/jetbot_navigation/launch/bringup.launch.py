from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetbot_navigation',
            node_executable='keyboard_control',
            node_name='keyboard_control',
	    output='screen'
        ),
        Node(
            package='jetbot_navigation',
            node_executable='navigator',
            node_name='navigator',
	    output='screen'
        )
    ])
