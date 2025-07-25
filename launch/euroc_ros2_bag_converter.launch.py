from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='euroc_ros2_bag_converter',
            executable='euroc_ros2_bag_converter_node',
            name='euroc_ros2_bag_converter_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': True},
                        {'euroc_root': "~/dataset/euroc/MH_01_easy"},
                        {'bag_path': "~/dataset/euroc/MH_01_easy/euroc.db3"}],
        )
    ])

