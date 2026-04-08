import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_topic = DeclareLaunchArgument(
        "elevation_topic",
        default_value="/elevation_mapping/elevation_map_raw",
    )

    return LaunchDescription(
        [
            map_topic,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ocs2_quadruped_controller_uncertainty_v1"),
                        "launch",
                        "elevation_mapping_test1.launch.py",
                    )
                ),
                launch_arguments={
                    "elevation_topic": LaunchConfiguration("elevation_topic"),
                }.items(),
            ),
        ]
    )
