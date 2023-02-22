import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import xacro
import sys
sys.path.append(f"{get_package_share_directory('edna_bringup')}/launch")
import namespaceutil

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('edna_description'))
    xacro_file = os.path.join(pkg_path,'urdf', 'robots','edna.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={ 'namespace': NAMESPACE })

    bringup_pkg_path = os.path.join(get_package_share_directory('edna_bringup'))
    rviz_file = os.path.join(bringup_pkg_path, 'config', 'description.rviz')
    tmp_rviz_file = os.path.join(bringup_pkg_path, 'config', 'tmp_description.rviz')
    namespaceutil.processRvizFileForNamespace(rviz_file, tmp_rviz_file, NAMESPACE)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace=NAMESPACE,
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create the publiser gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        namespace=NAMESPACE,
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Start Rviz2 with basic view
    run_rviz2_node = Node(
        package='rviz2',
        namespace=NAMESPACE,
        executable='rviz2',
        parameters=[{ 'use_sim_time': use_sim_time }],
        name='isaac_rviz2',
        output='screen',
        arguments=[["-d"], [tmp_rviz_file]],
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        joint_state_publisher_gui,
        run_rviz2_node
    ])