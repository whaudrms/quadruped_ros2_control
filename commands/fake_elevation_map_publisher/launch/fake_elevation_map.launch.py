from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_image = get_package_share_directory('ocs2_anymal_loopshaping_mpc') + '/data/step.png'

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_path',
            default_value=default_image,
            description='Path to a grayscale terrain image used to synthesize an elevation map.',
        ),
        DeclareLaunchArgument(
            'height_scale',
            default_value='0.12',
            description='Height scale applied to the grayscale terrain image.',
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.03',
            description='Grid map resolution.',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='2.0',
            description='Re-publish rate for the static elevation map.',
        ),
        DeclareLaunchArgument(
            'uncertainty_base',
            default_value='0.0',
            description='Base uncertainty value published in the uncertainty layer.',
        ),
        DeclareLaunchArgument(
            'uncertainty_gradient_weight',
            default_value='0.0',
            description='Additional uncertainty induced by terrain-image gradients.',
        ),
        DeclareLaunchArgument(
            'uncertainty_blur_kernel',
            default_value='5',
            description='Gaussian blur kernel used before computing uncertainty gradients.',
        ),
        Node(
            package='fake_elevation_map_publisher',
            executable='fake_elevation_map_node',
            name='fake_elevation_map_node',
            output='screen',
            parameters=[{
                'image_path': LaunchConfiguration('image_path'),
                'height_scale': LaunchConfiguration('height_scale'),
                'resolution': LaunchConfiguration('resolution'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'uncertainty_base': LaunchConfiguration('uncertainty_base'),
                'uncertainty_gradient_weight': LaunchConfiguration('uncertainty_gradient_weight'),
                'uncertainty_blur_kernel': LaunchConfiguration('uncertainty_blur_kernel'),
                'topic_name': '/elevation_mapping/elevation_map_raw',
                'frame_id': 'odom',
                'elevation_layer': 'elevation',
            }],
        ),
    ])
