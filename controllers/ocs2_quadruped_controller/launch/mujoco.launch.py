import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

package_controller = "ocs2_quadruped_controller"


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    enable_perceptive = context.launch_configurations['enable_perceptive'].lower() in ("true", "1", "yes", "on")
    enable_perceptive_reference_modification = context.launch_configurations['enable_perceptive_reference_modification'].lower() in ("true", "1", "yes", "on")
    enable_perceptive_foot_placement_constraint = context.launch_configurations['enable_perceptive_foot_placement_constraint'].lower() in ("true", "1", "yes", "on")
    enable_perceptive_foot_collision_constraint = context.launch_configurations['enable_perceptive_foot_collision_constraint'].lower() in ("true", "1", "yes", "on")
    publish_static_terrain = context.launch_configurations['publish_static_terrain'].lower() in ("true", "1", "yes", "on")
    terrain_scene_file = context.launch_configurations['terrain_scene_file']
    pkg_path = os.path.join(get_package_share_directory(package_description))

    with tempfile.NamedTemporaryFile(
            mode="w", suffix="_ocs2_quadruped_controller.yaml", delete=False) as controller_param_file:
        controller_param_file.write(
            "ocs2_quadruped_controller:\n"
            "  ros__parameters:\n"
            f"    enable_perceptive: {'true' if enable_perceptive else 'false'}\n"
            f"    enable_perceptive_reference_modification: {'true' if enable_perceptive_reference_modification else 'false'}\n"
            f"    enable_perceptive_foot_placement_constraint: {'true' if enable_perceptive_foot_placement_constraint else 'false'}\n"
            f"    enable_perceptive_foot_collision_constraint: {'true' if enable_perceptive_foot_collision_constraint else 'false'}\n"
        )
        controller_override_file = controller_param_file.name

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control.yaml",
        ]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_controller),
        "config",
        "visualize_ocs2.rviz"
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
        output='screen',
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    ocs2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ocs2_quadruped_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "120",
            "--switch-timeout", "120",
            "-p", controller_override_file,
        ],
        output="screen",
    )

    static_terrain_publisher = Node(
        package=package_controller,
        executable="planar_terrain_publisher",
        name="planar_terrain_publisher",
        output="screen",
        parameters=[
            {
                "scene_xml": terrain_scene_file,
                "terrain_topic": "/convex_plane_decomposition_ros/planar_terrain",
                "frame_id": "map",
                "resolution": 0.03,
                "publish_rate": 2.0,
            }
        ],
    )

    launch_nodes = [
        rviz,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[imu_sensor_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=imu_sensor_broadcaster_spawner,
                on_exit=[ocs2_controller_spawner],
            )
        ),
    ]

    if enable_perceptive and publish_static_terrain:
        launch_nodes.append(static_terrain_publisher)

    return launch_nodes


def generate_launch_description():  
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )

    enable_perceptive = DeclareLaunchArgument(
        'enable_perceptive',
        default_value='false',
        description='Enable perceptive terrain-aware OCS2 controller path'
    )

    publish_static_terrain = DeclareLaunchArgument(
        'publish_static_terrain',
        default_value='true',
        description='Publish a static PlanarTerrain message for perceptive mode'
    )

    enable_perceptive_reference_modification = DeclareLaunchArgument(
        'enable_perceptive_reference_modification',
        default_value='true',
        description='Enable terrain-aware target trajectory modification inside perceptive reference manager'
    )

    enable_perceptive_foot_placement_constraint = DeclareLaunchArgument(
        'enable_perceptive_foot_placement_constraint',
        default_value='true',
        description='Enable perceptive foot placement soft constraints'
    )

    enable_perceptive_foot_collision_constraint = DeclareLaunchArgument(
        'enable_perceptive_foot_collision_constraint',
        default_value='true',
        description='Enable perceptive foot collision soft constraints'
    )

    terrain_scene_file = DeclareLaunchArgument(
        'terrain_scene_file',
        default_value='/home/tony/unitree_mujoco/unitree_robots/go2/basic_step.xml',
        description='MuJoCo scene XML used to generate static planar terrain'
    )

    return LaunchDescription([
        pkg_description,
        enable_perceptive,
        enable_perceptive_reference_modification,
        enable_perceptive_foot_placement_constraint,
        enable_perceptive_foot_collision_constraint,
        publish_static_terrain,
        terrain_scene_file,
        OpaqueFunction(function=launch_setup),
    ])
