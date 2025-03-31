from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_patrol')
    rviz_config = os.path.join(pkg_share, 'config', 'robot_patrol_config.rviz')

    patrol_node = Node(
        package='robot_patrol',
        executable='go_to_pose_action_node',
        name='go_to_pose_action_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        patrol_node,
        rviz_node
    ])