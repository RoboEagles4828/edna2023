import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")
    joystick_file = os.path.join(bringup_path, 'config', 'xbox-real.yaml')
    
    common = { 'use_sim_time': 'false', 'namespace': 'real' }
    
    control_launch_args = common | {
        'use_ros2_control': 'true',
        'hardware_plugin': 'swerve_hardware/RealDriveHardware',
    }
    
    teleoplaunch_args = common | {
        'joystick_file': joystick_file,
        'enable_joy': 'false'
    }
    
    control_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','controlLayer.launch.py'
                )]), launch_arguments=control_launch_args.items())
    
    teleop_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','teleopLayer.launch.py'
                )]), launch_arguments=teleoplaunch_args.items())

    # Launch!
    return LaunchDescription([
        control_layer,
        teleop_layer,
    ])
