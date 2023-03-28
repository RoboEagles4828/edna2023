import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, TextSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.events import Shutdown

# Easy use of namespace since args are not strings
# NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    load_controllers = LaunchConfiguration('load_controllers')
    forward_command_controllers = LaunchConfiguration('forward_command_controller')
    namespace = LaunchConfiguration('namespace')
    hardware_plugin = LaunchConfiguration('hardware_plugin')

    # Process the URDF file
    description_pkg_path = os.path.join(get_package_share_directory('edna_description'))
    xacro_file = os.path.join(description_pkg_path,'urdf', 'robots','edna.urdf.xacro')
    edna_description_xml = Command(['xacro ', xacro_file, ' hw_interface_plugin:=', hardware_plugin])
    
    # Get paths to other config files
    bringup_pkg_path = os.path.join(get_package_share_directory('edna_bringup'))
    controllers_file = os.path.join(bringup_pkg_path, 'config', 'controllers.yaml')
    
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace=namespace,
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': edna_description_xml,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
            'frame_prefix': [namespace, '/']
        }],
    )

    # Starts ROS2 Control
    control_node = Node(
        package="controller_manager",
        namespace=namespace,
        executable="ros2_control_node",
        condition=IfCondition(use_ros2_control),
        parameters=[{
            "robot_description": edna_description_xml,
            "use_sim_time": use_sim_time,
            }, controllers_file],
        output="both",
    )
    control_node_require = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[
                LogInfo(msg="Listener exited; tearing down entire system."),
                EmitEvent(event=Shutdown())
            ],
        )
    )

    # Starts ROS2 Control Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", ['/', namespace, "/controller_manager"]],
        condition=IfCondition(use_ros2_control),
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", ['/', namespace, "/controller_manager"]],
        parameters=[{
            "robot_description": edna_description_xml,
            "use_sim_time": use_sim_time,
            }, controllers_file],
        condition=IfCondition(use_ros2_control and load_controllers),
    )

    #Starts ROS2 Control Swerve Drive Controller
    swerve_drive_controller_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["swerve_controller", "-c", ['/', namespace, "/controller_manager"]],
        condition=IfCondition(use_ros2_control and load_controllers),
    )
    swerve_drive_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_drive_controller_spawner],
        )
    )

    # Starts ROS2 Control Forward Controller
    forward_position_controller_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["forward_position_controller", "-c", ['/', namespace, "/controller_manager"]],
        condition=IfCondition(use_ros2_control and forward_command_controllers),
    )
    forward_position_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )
    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["forward_velocity_controller", "-c", ['/', namespace, "/controller_manager"]],
        condition=IfCondition(use_ros2_control and forward_command_controllers),
    )
    forward_velocity_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_velocity_controller_spawner],
        )
    )


    # Launch!
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='default',
            description='The namespace of nodes and links'),
        DeclareLaunchArgument(
            'hardware_plugin',
            default_value='swerve_hardware/IsaacDriveHardware',
            description='Which ros2 control hardware plugin to use'),
        DeclareLaunchArgument(
            'load_controllers',
            default_value='true',
            description='Enable or disable ros2 controllers but leave hardware interfaces'),
        DeclareLaunchArgument(
            'forward_command_controller',
            default_value='false',
            description='Forward commands for ros2 control'),
        node_robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        swerve_drive_controller_delay,
        forward_position_controller_delay,
        forward_velocity_controller_delay,
        control_node_require
    ])