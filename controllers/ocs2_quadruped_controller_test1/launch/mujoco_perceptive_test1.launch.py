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

package_controller = "ocs2_quadruped_controller_test1"


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    controller_config = context.launch_configurations['controller_config']
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_controllers = os.path.join(
        get_package_share_directory(package_description),
        'config',
        controller_config,
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_controller), "config", "visualize_ocs2.rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2_test1',
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
        arguments=["ocs2_quadruped_controller_test1", "--controller-manager", "/controller_manager"]
    )

    terrain_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_controller),
                'launch',
                'elevation_mapping_test1.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('launch_plane_decomposition')),
    )

    fake_elevation_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fake_elevation_map_publisher'),
                'launch',
                'fake_elevation_map.launch.py',
            )
        ),
        launch_arguments={
            'image_path': LaunchConfiguration('terrain_image'),
            'height_scale': LaunchConfiguration('terrain_height_scale'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_fake_elevation_map')),
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
        'pkg_description',
        default_value='go2_description_test1',
        description='package for robot description'
    )

    controller_config = DeclareLaunchArgument(
        'controller_config',
        default_value='robot_control_perceptive_test1.yaml',
        description='Controller config file inside the description package config directory.'
    )

    launch_plane_decomposition = DeclareLaunchArgument(
        'launch_plane_decomposition',
        default_value='false',
        description='Launch convex plane decomposition node. Requires /elevation_mapping/elevation_map_raw input.'
    )

    launch_fake_elevation_map = DeclareLaunchArgument(
        'launch_fake_elevation_map',
        default_value='false',
        description='Launch a static elevation map publisher from a terrain image.'
    )

    terrain_image = DeclareLaunchArgument(
        'terrain_image',
        default_value=os.path.join(
            get_package_share_directory('ocs2_anymal_loopshaping_mpc'),
            'data',
            'step.png',
        ),
        description='Terrain image used by the fake elevation map publisher.'
    )

    terrain_height_scale = DeclareLaunchArgument(
        'terrain_height_scale',
        default_value='0.12',
        description='Height scale for the fake elevation map terrain image.'
    )

    return LaunchDescription([
        pkg_description,
        controller_config,
        launch_plane_decomposition,
        launch_fake_elevation_map,
        terrain_image,
        terrain_height_scale,
        OpaqueFunction(function=launch_setup),
    ])
