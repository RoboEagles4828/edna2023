import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

# Easy use of namespace since args are not strings
NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    bringup_pkg_path = os.path.join(get_package_share_directory('edna_bringup'))
    params_file = os.path.join(bringup_pkg_path, 'config', 'mapper.yaml')

    run_slam_toolbox = Node(
        package='slam_toolbox',
        namespace=namespace,
        executable='async_slam_toolbox_node',
        parameters=[{ 
            'use_sim_time': use_sim_time, 
            'scan_topic': f'/{NAMESPACE}/scan',
            'base_frame': f'{NAMESPACE}/base_footprint',
            'map_frame': f'{NAMESPACE}/map',
            'odom_frame': f'{NAMESPACE}/odom'
        }, params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='default',
            description='The namespace of nodes and links'),
        run_slam_toolbox
    ])