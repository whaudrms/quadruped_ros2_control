from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    terrain_topic = LaunchConfiguration("terrain_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    num_vertices = LaunchConfiguration("num_vertices")
    growth_factor = LaunchConfiguration("growth_factor")
    boundary_margin = LaunchConfiguration("boundary_margin")
    nominal_x = LaunchConfiguration("nominal_x")
    nominal_y = LaunchConfiguration("nominal_y")
    nominal_z = LaunchConfiguration("nominal_z")

    return LaunchDescription([
        DeclareLaunchArgument(
            "terrain_topic",
            default_value="/planar_terrain",
            description="Input convex planar terrain topic",
        ),
        DeclareLaunchArgument(
            "marker_topic",
            default_value="/foot_placement_constraint_demo",
            description="Output MarkerArray topic",
        ),
        DeclareLaunchArgument("num_vertices", default_value="16"),
        DeclareLaunchArgument("growth_factor", default_value="1.05"),
        DeclareLaunchArgument("boundary_margin", default_value="0.05"),
        DeclareLaunchArgument("nominal_x", default_value="0.0"),
        DeclareLaunchArgument("nominal_y", default_value="0.0"),
        DeclareLaunchArgument("nominal_z", default_value="0.4"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open a focused RViz view (no swing trajectory displays)",
        ),
        Node(
            package="ocs2_quadruped_controller",
            executable="foot_placement_constraint_visualizer",
            name="foot_placement_constraint_visualizer",
            output="screen",
            parameters=[{
                "terrain_topic": terrain_topic,
                "marker_topic": marker_topic,
                "num_vertices": ParameterValue(num_vertices, value_type=int),
                "growth_factor": ParameterValue(growth_factor, value_type=float),
                "boundary_margin": ParameterValue(boundary_margin, value_type=float),
                "nominal_x": ParameterValue(nominal_x, value_type=float),
                "nominal_y": ParameterValue(nominal_y, value_type=float),
                "nominal_z": ParameterValue(nominal_z, value_type=float),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="foot_placement_constraint_rviz",
            arguments=["-d", str(Path(get_package_share_directory(
                "ocs2_quadruped_controller")) / "config" / "foot_placement_constraint_demo.rviz")],
            remappings=[("/foot_placement_constraint_demo", marker_topic)],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
