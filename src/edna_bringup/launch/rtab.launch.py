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
        'namespace': NAMESPACE,
        'rgb_topic': '/Left/rgb',
        'camera_info_topic': '/Left/camera_info',
        'depth_topic': '/Left/depth_pcl',
        'depth_camera_info_topic': '/Left/camera_info',
        'frame_id': f'{NAMESPACE}/base_link',
        'approx_sync': 'true',
        'wait_imu_to_init': 'true',
        'imu_topic': f'/imu',
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