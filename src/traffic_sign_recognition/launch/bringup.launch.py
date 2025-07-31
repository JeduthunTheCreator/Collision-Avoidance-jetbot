from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traffic_sign_recognition',
            node_executable='traffic_sign_node',
            node_name='traffic_sign_node',
	    output='screen'
        ),
        Node(
            package='traffic_sign_recognition',
            node_executable='decision_maker_node',
            node_name='decision_maker_node',
	    output='screen'
        )
    ])
