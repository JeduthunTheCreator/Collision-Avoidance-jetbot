import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='my_first_pkg').find('my_first_pkg')
    default_model_path_second_robot = os.path.join(pkg_share, 'urdf/second_robot.urdf')
    world_path=os.path.join(pkg_share, 'world/my_world.world')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_sec',
        parameters=[{'robot_description': Command(['xacro', LaunchConfiguration('robot_model')])}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_sec'
    )

    spawn_second_robot_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'second_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '-1.0'],
        output='screen'
    )

    # Integrate OctoMap for SLAM
    octomap_mapping_node = launch_ros.actions.Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='robot_model', default_value=default_model_path_second_robot,
                                            description='Absolute path to robot urdf file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_second_robot_entity,
        octomap_mapping_node
