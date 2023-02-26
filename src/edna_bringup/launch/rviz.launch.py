import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")

    use_sim_time = 'false'
    
    control_launch_args = {
        'use_sim_time': use_sim_time,
        'namespace': NAMESPACE,
        'use_ros2_control': 'false'
    }
    control_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','controlLayer.launch.py'
                )]), launch_arguments=control_launch_args.items())
    

    rviz_file = os.path.join(bringup_path, 'config', 'description.rviz')
    debug_launch_args = {
        'use_sim_time': use_sim_time,
        'namespace': NAMESPACE,
        'enable_rviz': 'true',
        'enable_foxglove': 'false',
        'rviz_file': rviz_file
    }
    debug_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','debugLayer.launch.py'
                )]), launch_arguments=debug_launch_args.items())

    # Launch!
    return LaunchDescription([
        control_layer,
        debug_layer
    ])