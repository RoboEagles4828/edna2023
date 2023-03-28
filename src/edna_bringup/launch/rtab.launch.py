import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    rtabmap_ros_path = get_package_share_directory("rtabmap_ros")

    rtabmap_args = {
        'rtabmap_args': '--delete_db_on_start',
        'use_sim_time': use_sim_time,
        'namespace': f'{NAMESPACE}_rtab',
        # Frames
        'frame_id': f'{NAMESPACE}/base_link',
        'odom_frame_id': f'{NAMESPACE}/odom',
        'map_frame_id': f'{NAMESPACE}/map',
        # Topics
        'rgb_topic': f'/{NAMESPACE}/left/rgb',
        'camera_info_topic': f'/{NAMESPACE}/left/camera_info',
        'depth_topic': f'/{NAMESPACE}/left/depth',
        'imu_topic': f'/{NAMESPACE}/imu',
        'odom_topic': f'/{NAMESPACE}/zed/odom',

        'approx_sync': 'false',
        'wait_imu_to_init': 'true',
        'visual_odometry': 'false',
        'publish_tf_odom': 'false',
        'qos': '1',
        'rviz': 'true',
    }

    rtab_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    rtabmap_ros_path,'launch','rtabmap.launch.py'
                )]), launch_arguments=rtabmap_args.items())
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        rtab_layer,
    ])