from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'ros_lidar_sdk'
    rviz_config = 'rviz2.rviz'
    pkg_share = get_package_share_directory(package_name)
    
    rviz_config_path = os.path.join(pkg_share,'rviz',rviz_config)
    return LaunchDescription([
        Node(
            package='ros_lidar_sdk',
            executable='lidar_sdk_node',
            name='lidar_sdk_node',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rviz_config_path],
            output='screen'
        )
    ])
