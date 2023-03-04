import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    """Launch file which brings up visual slam node configured for Isaac Sim."""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        namespace=NAMESPACE,
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[(f'/{NAMESPACE}/stereo_camera/left/camera_info', f'/{NAMESPACE}/Left/camera_info'),
                    (f'/{NAMESPACE}/stereo_camera/right/camera_info', f'/{NAMESPACE}/Right/camera_info'),
                    (f'/{NAMESPACE}/stereo_camera/left/image', f'/{NAMESPACE}/Left/rgb'),
                    (f'/{NAMESPACE}/stereo_camera/right/image', f'/{NAMESPACE}/Right/rgb'),
                    (f'/{NAMESPACE}/visual_slam/imu', f'/{NAMESPACE}/imu')],
        parameters=[{
                    'use_sim_time': True,
                    'enable_imu': True,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'input_base_frame': f'{NAMESPACE}/base_link',
                    'input_imu_frame': f'{NAMESPACE}/zed_camera_center',
                    'input_left_camera_frame': f"{NAMESPACE}/zed_left_camera_frame",
                    'input_right_camera_frame': f"{NAMESPACE}/zed_right_camera_frame",
                    'map_frame': f'{NAMESPACE}/map',
                    'odom_frame': f'{NAMESPACE}/odom',
                    'base_frame': f'{NAMESPACE}/base_link'
                    }]

    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace=NAMESPACE,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container])