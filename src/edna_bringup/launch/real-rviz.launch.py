import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")
    rviz_file = os.path.join(bringup_path, 'config', 'real.rviz')
    
    common = { 
        'use_sim_time': 'false', 
        'namespace': 'real',
        'forward_command_controller': 'false',
    }
    
    debug_launch_args = common | {
        'enable_rviz': 'true',
        'enable_foxglove': 'false',
        'enable_debugger_gui': 'true',
        'rviz_file': rviz_file
    }

    debug_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','debugLayer.launch.py'
                )]), launch_arguments=debug_launch_args.items())

    # Launch!
    return LaunchDescription([
        debug_layer,
    ])