from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config=get_package_share_directory('ros_lidar_sdk')+'/rviz/rviz2.rviz'

    return LaunchDescription([
        Node(namespace='ros_lidar_sdk', package='ros_lidar_sdk', executable='lidar_sdk_node', output='screen'),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
