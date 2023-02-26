import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    joystick_file = LaunchConfiguration('joystick_file')
    
    joy = Node(
            package='joy',
            namespace=namespace,
            executable='joy_node', 
            name='joy_node',
            parameters=[{'use_sim_time': use_sim_time }, joystick_file])

    controller_prefix = 'swerve_controller'
    joy_teleop_twist = Node(
        package='teleop_twist_joy',
        namespace=namespace,
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{'use_sim_time': use_sim_time }, joystick_file],
        remappings={(f'/{namespace}/cmd_vel', f'/{namespace}/{controller_prefix}/cmd_vel_unstamped')},
        )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='default',
            description='The namespace of nodes and links'),
        DeclareLaunchArgument(
            'joystick_file',
            default_value='',
            description='The file with joystick parameters'),
        joy,
        joy_teleop_twist
    ])