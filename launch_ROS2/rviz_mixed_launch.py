import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

frame_id = 'cyber_sensor_mixed'
local_ip = '192.168.1.5'

package_name = 'cyber_sensor_mixed'
cur_path = get_package_share_directory(package_name)
rviz_config_path = os.path.join(cur_path, 'config', 'cyber_sensor_mixed_ROS2.rviz')

cyber_sensor_ros2_params = [
    {"frame_id": frame_id},
    {"local_ip": local_ip}
]

def generate_launch_description():
    cyber_sensor_driver = Node(
        package=package_name,
        executable='cyber_sensor_mixed_node',
        output='screen',
        parameters=cyber_sensor_ros2_params
    )

    cyber_sensor_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    return LaunchDescription([
        cyber_sensor_driver,
        cyber_sensor_rviz,
    ])
