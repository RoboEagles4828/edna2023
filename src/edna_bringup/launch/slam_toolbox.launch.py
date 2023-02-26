import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import sys
sys.path.append(f"{get_package_share_directory('edna_bringup')}/launch")

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_pkg_path = os.path.join(get_package_share_directory('edna_bringup'))
    params_file = os.path.join(bringup_pkg_path, 'config', 'mapper_params.yaml')

    run_slam_toolbox = Node(
        package='slam_toolbox',
        namespace=NAMESPACE,
        executable='async_slam_toolbox_node',
        parameters=[{ 
            'use_sim_time': use_sim_time, 
            'scan_topic': f'/{NAMESPACE}/scan',
            'base_frame': f'{NAMESPACE}_footprint',
            'map_frame': f'{NAMESPACE}_map',
            'odom_frame': f'{NAMESPACE}_odom'
        }, params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'
        ),
        run_slam_toolbox
    ])