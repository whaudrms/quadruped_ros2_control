import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_controller = "ocs2_quadruped_controller_test1"


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations["pkg_description"]
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_description),
            "config",
            "robot_control_test1.yaml",
        ]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_controller), "config", "visualize_ocs2.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_ocs2_test1",
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
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
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
        arguments=["ocs2_quadruped_controller_test1", "--controller-manager", "/controller_manager"],
    )

    return [
        rviz,
        robot_state_publisher,
        controller_manager,
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
        default_value="go2_description_test1",
        description="package for robot description",
    )

    return LaunchDescription(
        [
            pkg_description,
            OpaqueFunction(function=launch_setup),
        ]
    )
