import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")
    rviz_file = os.path.join(bringup_path, 'config', 'description.rviz')
    
    common = { 'use_sim_time': 'false', 'namespace': NAMESPACE }
    
    control_launch_args = common | {
        'use_ros2_control': 'false',
        'load_controllers': 'false'
    }
    
    debug_launch_args = common | {
        'enable_rviz': 'true',
        'enable_foxglove': 'false',
        'enable_joint_state_publisher': 'true',
        'rviz_file': rviz_file
    }
    
    control_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','controlLayer.launch.py'
                )]), launch_arguments=control_launch_args.items())

    debug_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','debugLayer.launch.py'
                )]), launch_arguments=debug_launch_args.items())

    # Launch!
    return LaunchDescription([
        control_layer,
        debug_layer,
    ])