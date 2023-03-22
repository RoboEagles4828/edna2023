import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")
    rviz_file = os.path.join(bringup_path, 'config', 'riodebug.rviz')
    
    common = { 'use_sim_time': 'false', 'namespace': 'real' }

    control_launch_args = common | {
        'use_ros2_control': 'true',
        'load_controllers': 'false',
        'forward_command_controller': 'true', # Change to false in order to disable publishing
        'hardware_plugin': 'swerve_hardware/RealDriveHardware', # Use IsaacDriveHardware for isaac or RealDriveHardware for real.
    }
    
    debug_launch_args = common | {
        'enable_rviz': 'true',
        'enable_foxglove': 'false',
        'enable_debugger_gui': 'true',
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
    delay_debug_layer =  TimerAction(period=3.0, actions=[debug_layer])

    # Launch!
    return LaunchDescription([
        control_layer,
        delay_debug_layer
    ])
