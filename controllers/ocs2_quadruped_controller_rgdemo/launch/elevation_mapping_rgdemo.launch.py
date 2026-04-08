from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller_pkg = get_package_share_directory('ocs2_quadruped_controller_rgdemo')

    return LaunchDescription([
        DeclareLaunchArgument(
            'node_parameter_file',
            default_value=controller_pkg + '/config/convex_plane_decomposition_node.yaml',
            description='ROS node parameters for convex plane decomposition',
        ),
        Node(
            package='convex_plane_decomposition_ros',
            executable='convex_plane_decomposition_ros_node',
            name='convex_plane_decomposition_ros_node',
            namespace='convex_plane_decomposition_ros',
            output='screen',
            parameters=[
                LaunchConfiguration('node_parameter_file'),
            ],
        ),
    ])
