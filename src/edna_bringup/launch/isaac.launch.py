import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    bringup_path = get_package_share_directory("edna_bringup")
    use_sim_time = 'true'


    # Control
    control_launch_args = {
        'use_sim_time': use_sim_time,
        'namespace': NAMESPACE,
        'use_ros2_control': 'true',
        'hardware_plugin': 'swerve_hardware/IsaacDriveHardware',
    }
    control_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','controlLayer.launch.py'
                )]), launch_arguments=control_launch_args.items())
    

    # Teleop
    joystick_file = os.path.join(bringup_path, 'config', 'xbox-sim.yaml')
    teleoplaunch_args = {
        'use_sim_time': use_sim_time,
        'namespace': NAMESPACE,
        'joystick_file': joystick_file,
    }
    teleop_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','teleopLayer.launch.py'
                )]), launch_arguments=teleoplaunch_args.items())


    # Debug tools
    rviz_file = os.path.join(bringup_path, 'config', 'view.rviz')
    debug_launch_args = {
        'use_sim_time': use_sim_time,
        'namespace': NAMESPACE,
        'enable_rviz': 'true',
        'enable_foxglove': 'true',
        'rviz_file': rviz_file
    }
    debug_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','debugLayer.launch.py'
                )]), launch_arguments=debug_launch_args.items())
    delay_debug_layer =  TimerAction(period=3.0, actions=[debug_layer])

    # Launch!
    return LaunchDescription([
        control_layer,
        teleop_layer,
        delay_debug_layer,
    ])
