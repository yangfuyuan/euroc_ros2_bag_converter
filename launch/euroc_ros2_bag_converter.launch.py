from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'euroc_root',
            default_value='~/dataset/euroc/MH_05_difficult',
            description='Path to the root of the EuRoC dataset (mav0 directory)'
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value='~/dataset/euroc/MH_05_difficult/euroc.db3',
            description='Output path for the generated ROS 2 bag (.db3)'
        ),
        Node(
            package='euroc_ros2_bag_converter',
            executable='euroc_ros2_bag_converter_node',
            name='euroc_ros2_bag_converter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': True},
                {'euroc_root': LaunchConfiguration('euroc_root')},
                {'bag_path': LaunchConfiguration('bag_path')}
            ],
        )
    ])
