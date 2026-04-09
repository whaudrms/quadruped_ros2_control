from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_image = get_package_share_directory('ocs2_anymal_loopshaping_mpc') + '/data/step.png'

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_mode',
            default_value='image',
            description='Map source mode: image or box.',
        ),
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
            'map_length_x',
            default_value='2.5',
            description='Map length in x for generated box mode.',
        ),
        DeclareLaunchArgument(
            'map_length_y',
            default_value='1.6',
            description='Map length in y for generated box mode.',
        ),
        DeclareLaunchArgument(
            'box_center_x',
            default_value='0.55',
            description='Box center x for box mode.',
        ),
        DeclareLaunchArgument(
            'box_center_y',
            default_value='0.0',
            description='Box center y for box mode.',
        ),
        DeclareLaunchArgument(
            'box_size_x',
            default_value='0.36',
            description='Box size x for box mode.',
        ),
        DeclareLaunchArgument(
            'box_size_y',
            default_value='1.10',
            description='Box size y for box mode.',
        ),
        DeclareLaunchArgument(
            'box_height',
            default_value='0.08',
            description='Box height for box mode.',
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
                'map_mode': LaunchConfiguration('map_mode'),
                'image_path': LaunchConfiguration('image_path'),
                'height_scale': LaunchConfiguration('height_scale'),
                'resolution': LaunchConfiguration('resolution'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'map_length_x': LaunchConfiguration('map_length_x'),
                'map_length_y': LaunchConfiguration('map_length_y'),
                'box_center_x': LaunchConfiguration('box_center_x'),
                'box_center_y': LaunchConfiguration('box_center_y'),
                'box_size_x': LaunchConfiguration('box_size_x'),
                'box_size_y': LaunchConfiguration('box_size_y'),
                'box_height': LaunchConfiguration('box_height'),
                'uncertainty_base': LaunchConfiguration('uncertainty_base'),
                'uncertainty_gradient_weight': LaunchConfiguration('uncertainty_gradient_weight'),
                'uncertainty_blur_kernel': LaunchConfiguration('uncertainty_blur_kernel'),
                'topic_name': '/elevation_mapping/elevation_map_raw',
                'frame_id': 'odom',
                'elevation_layer': 'elevation',
            }],
        ),
    ])
