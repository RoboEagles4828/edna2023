import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

# Easy use of namespace since args are not strings
# NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    joystick_file = LaunchConfiguration('joystick_file')
    enable_joy = LaunchConfiguration('enable_joy')
    
    frc_auton_reader = Node(
        package = "frc_auton",
        namespace=namespace,
        executable= "reader",
        name = "frc_auton_node",
        parameters=[{
            "auton_name": "24",   
        }]
    )
    frc_teleop_writer = Node(
        package = "frc_auton",
        namespace=namespace,
        executable= "writer",
        name = "frc_auton_node",
        parameters=[{
            "record_auton": False,
            "record_without_fms": False,    
        }]
    )
    joy = Node(
            package='joy',
            namespace=namespace,
            executable='joy_node', 
            name='joy_node',
            condition=IfCondition(enable_joy),
            parameters=[{
                'use_sim_time': use_sim_time,
                'deadzone': 0.15,
            }])

    controller_prefix = 'swerve_controller'
    joy_teleop_twist = Node(
        package='teleop_twist_joy',
        namespace=namespace,
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joystick_file, {'use_sim_time': use_sim_time}],
        remappings={
            ("cmd_vel", f"{controller_prefix}/cmd_vel_unstamped"),
            ("odom", "zed/odom")          
        },
    )
    joint_trajectory_teleop = Node(
        package='joint_trajectory_teleop',
        namespace=namespace,
        executable='joint_trajectory_teleop',
        name='joint_trajectory_teleop',
        parameters=[{'use_sim_time': use_sim_time}]
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
        DeclareLaunchArgument(
            'enable_joy',
            default_value='true',
            description='Enables joystick teleop'),
        joy,
        joy_teleop_twist,
        joint_trajectory_teleop,
        frc_auton_reader,
        # frc_teleop_writer
    ])