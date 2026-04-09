import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_controller = "ocs2_quadruped_controller_height_only_v1"


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations["pkg_description"]
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_controllers = os.path.join(
        get_package_share_directory(package_description),
        "config",
        context.launch_configurations["controller_config"],
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_controller), "config", "visualize_ocs2.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_ocs2_height_only_v1",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "publish_frequency": 20.0,
                "use_tf_static": True,
                "robot_description": robot_description,
                "ignore_timestamp": True,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ocs2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_quadruped_controller_height_only_v1", "--controller-manager", "/controller_manager"],
    )

    terrain_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_controller),
                "launch",
                "elevation_mapping_height_only_v1.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_plane_decomposition")),
    )

    fake_elevation_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fake_elevation_map_publisher"),
                "launch",
                "fake_elevation_map.launch.py",
            )
        ),
        launch_arguments={
            "map_mode": LaunchConfiguration("fake_map_mode"),
            "image_path": LaunchConfiguration("terrain_image"),
            "height_scale": LaunchConfiguration("terrain_height_scale"),
            "resolution": LaunchConfiguration("terrain_resolution"),
            "map_length_x": LaunchConfiguration("terrain_map_length_x"),
            "map_length_y": LaunchConfiguration("terrain_map_length_y"),
            "box_center_x": LaunchConfiguration("terrain_box_center_x"),
            "box_center_y": LaunchConfiguration("terrain_box_center_y"),
            "box_size_x": LaunchConfiguration("terrain_box_size_x"),
            "box_size_y": LaunchConfiguration("terrain_box_size_y"),
            "box_height": LaunchConfiguration("terrain_box_height"),
            "uncertainty_base": LaunchConfiguration("terrain_uncertainty_base"),
            "uncertainty_gradient_weight": LaunchConfiguration("terrain_uncertainty_gradient_weight"),
            "uncertainty_blur_kernel": LaunchConfiguration("terrain_uncertainty_blur_kernel"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_fake_elevation_map")),
    )

    return [
        rviz,
        robot_state_publisher,
        controller_manager,
        fake_elevation_map,
        terrain_pipeline,
        joint_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[ocs2_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        "pkg_description",
        default_value="go2_description_height_only_v1",
        description="package for robot description",
    )

    launch_plane_decomposition = DeclareLaunchArgument(
        "launch_plane_decomposition",
        default_value="false",
        description="Launch convex plane decomposition node. Requires /elevation_mapping/elevation_map_raw input.",
    )

    controller_config = DeclareLaunchArgument(
        "controller_config",
        default_value="robot_control_perceptive_height_only_v1.yaml",
        description="Controller yaml file inside the description package config directory.",
    )

    launch_fake_elevation_map = DeclareLaunchArgument(
        "launch_fake_elevation_map",
        default_value="false",
        description="Launch a static elevation map publisher from a terrain image.",
    )

    terrain_image = DeclareLaunchArgument(
        "terrain_image",
        default_value=os.path.join(get_package_share_directory("ocs2_anymal_loopshaping_mpc"), "data", "step.png"),
        description="Terrain image used by the fake elevation map publisher.",
    )

    terrain_height_scale = DeclareLaunchArgument(
        "terrain_height_scale",
        default_value="0.12",
        description="Height scale for the fake elevation map terrain image.",
    )
    fake_map_mode = DeclareLaunchArgument(
        "fake_map_mode",
        default_value="image",
        description="Fake elevation map source mode: image or box.",
    )

    terrain_resolution = DeclareLaunchArgument(
        "terrain_resolution",
        default_value="0.03",
        description="Grid-map resolution used by the fake elevation map publisher.",
    )
    terrain_map_length_x = DeclareLaunchArgument(
        "terrain_map_length_x",
        default_value="2.5",
        description="Grid-map length in x used by the fake elevation map publisher.",
    )
    terrain_map_length_y = DeclareLaunchArgument(
        "terrain_map_length_y",
        default_value="1.6",
        description="Grid-map length in y used by the fake elevation map publisher.",
    )
    terrain_box_center_x = DeclareLaunchArgument(
        "terrain_box_center_x",
        default_value="0.55",
        description="Box center x used in direct box map mode.",
    )
    terrain_box_center_y = DeclareLaunchArgument(
        "terrain_box_center_y",
        default_value="0.0",
        description="Box center y used in direct box map mode.",
    )
    terrain_box_size_x = DeclareLaunchArgument(
        "terrain_box_size_x",
        default_value="0.36",
        description="Box size x used in direct box map mode.",
    )
    terrain_box_size_y = DeclareLaunchArgument(
        "terrain_box_size_y",
        default_value="1.10",
        description="Box size y used in direct box map mode.",
    )
    terrain_box_height = DeclareLaunchArgument(
        "terrain_box_height",
        default_value="0.08",
        description="Box height used in direct box map mode.",
    )

    terrain_uncertainty_base = DeclareLaunchArgument(
        "terrain_uncertainty_base",
        default_value="0.05",
        description="Base uncertainty layer value for height-only perceptive experiments.",
    )

    terrain_uncertainty_gradient_weight = DeclareLaunchArgument(
        "terrain_uncertainty_gradient_weight",
        default_value="0.35",
        description="Gradient-driven uncertainty weight for height-only perceptive experiments.",
    )

    terrain_uncertainty_blur_kernel = DeclareLaunchArgument(
        "terrain_uncertainty_blur_kernel",
        default_value="5",
        description="Blur kernel used before computing uncertainty gradients.",
    )

    return LaunchDescription(
        [
        pkg_description,
        controller_config,
        launch_plane_decomposition,
        launch_fake_elevation_map,
            fake_map_mode,
            terrain_image,
            terrain_height_scale,
            terrain_resolution,
            terrain_map_length_x,
            terrain_map_length_y,
            terrain_box_center_x,
            terrain_box_center_y,
            terrain_box_size_x,
            terrain_box_size_y,
            terrain_box_height,
            terrain_uncertainty_base,
            terrain_uncertainty_gradient_weight,
            terrain_uncertainty_blur_kernel,
            OpaqueFunction(function=launch_setup),
        ]
    )
