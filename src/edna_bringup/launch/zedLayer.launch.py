import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_zed = get_package_share_directory("zed_wrapper")
    edna_bringup = get_package_share_directory("edna_bringup")

    camera_model = 'zed2i'
    camera_name = camera_model
    
    publish_urdf = 'true'  # Publish static frames from camera URDF
    
    # Robot base frame. Note: overrides the parameter `pos_tracking.base_frame` in `common.yaml`.
    base_frame = 'base_link'
    # Position X of the camera with respect to the base frame [m].
    cam_pos_x = '0.0'
    # Position Y of the camera with respect to the base frame [m].
    cam_pos_y = '0.0'
    # Position Z of the camera with respect to the base frame [m].
    cam_pos_z = '0.0'
    # Roll orientation of the camera with respect to the base frame [rad].
    cam_roll = '0.0'
    # Pitch orientation of the camera with respect to the base frame [rad].
    cam_pitch = '0.0'
    # Yaw orientation of the camera with respect to the base frame [rad].
    cam_yaw = '0.0'

    # ZED Configurations from local config
    config_common_path = os.path.join(edna_bringup, 'config', 'zed-config-common.yaml')
    config_camera_path = ''

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(pkg_zed, 'urdf', 'zed_descr.urdf.xacro')

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': camera_model,
            'camera_name': camera_name,
            'node_name': 'zed_node',
            'config_common_path': config_common_path,
            'config_camera_path': config_camera_path,
            'publish_urdf': publish_urdf,
            'xacro_path': xacro_path,
            'svo_path': '',
            'base_frame': base_frame,
            'cam_pos_x': cam_pos_x,
            'cam_pos_y': cam_pos_y,
            'cam_pos_z': cam_pos_z,
            'cam_roll': cam_roll,
            'cam_pitch': cam_pitch,
            'cam_yaw': cam_yaw
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        zed_wrapper_launch,
    ])
