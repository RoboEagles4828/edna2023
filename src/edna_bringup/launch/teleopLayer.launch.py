import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node

# Easy use of namespace since args are not strings
NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    joystick_file = LaunchConfiguration('joystick_file')
    
    joy = Node(
            package='joy',
            namespace=namespace,
            executable='joy_node', 
            name='joy_node',
            parameters=[])

    controller_prefix = 'swerve_controller'
    joy_teleop_twist = Node(
        package='teleop_twist_joy',
        namespace=namespace,
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joystick_file],
        remappings={(f'/{NAMESPACE}/cmd_vel', f'/{NAMESPACE}/{controller_prefix}/cmd_vel_unstamped')},
    )
    # joint_trajectory_teleop = Node(
    #     package='joint_trajectory_teleop',
    #     namespace=namespace,
    #     executable='publish_trajectory_message',
    #     parameters=[]
    # )
    
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
        joy_teleop_twist,
        # joint_trajectory_teleop
    ])