import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():

    # Camera model (force value)
    camera_model = 'zed2i'  

    config_common_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        'common.yaml'
    )
    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model + '.yaml'
    )

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        namespace=str(NAMESPACE),
        executable='zed_wrapper',
        name='zed',
        output='screen',
        #prefix=['xterm -e valgrind --tools=callgrind'],
        #prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_camera_path,  # Camera related parameters
            # Overriding
            {
                'general.camera_name': f'{NAMESPACE}/{camera_model}',
                'general.camera_model': camera_model,
                'general.svo_file': 'live',
                'pos_tracking.base_frame': f'{NAMESPACE}/base_link',
                'pos_tracking.map_frame': f'{NAMESPACE}/map',
                'pos_tracking.odometry_frame': f'{NAMESPACE}/odom',
                'general.zed_id': 0,
                'general.serial_number': 0,
                'pos_tracking.publish_tf': True,
                'pos_tracking.publish_map_tf': True,
                'pos_tracking.publish_imu_tf': True
            }
        ]
    )


    delay_zed_wrapper = TimerAction(period=5.0, actions=[zed_wrapper_node])
    
    # Define LaunchDescription variable
    ld = LaunchDescription()
    
    # Add nodes to LaunchDescription
    ld.add_action(delay_zed_wrapper)

    return ld
