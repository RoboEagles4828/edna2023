import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

NAMESPACE = os.environ.get('ROS_NAMESPACE')

def generate_launch_description():
    use_sim_time = True

    # Process the URDF file
    description_pkg_path = os.path.join(get_package_share_directory('edna_description'))
    xacro_file = os.path.join(description_pkg_path,'urdf', 'robots','edna.urdf.xacro')
    edna_description_config = xacro.process_file(xacro_file)
    edna_description_xml = edna_description_config.toxml()

    # Get paths to other config files
    bringup_pkg_path = os.path.join(get_package_share_directory('edna_bringup'))
    controllers_file = os.path.join(bringup_pkg_path, 'config', 'controllers.yaml')
    joystick_file = os.path.join(bringup_pkg_path, 'config', 'xbox-holonomic-sim.config.yaml')
    rviz_file = os.path.join(bringup_pkg_path, 'config', 'view.rviz')

    # Save Built URDF file to Description Directory
    description_source_code_path = os.path.abspath(os.path.join(description_pkg_path, "../../../../src/edna_description/urdf"))
    urdf_save_path = os.path.join(description_source_code_path, "edna.urdf")
    with open(urdf_save_path, 'w') as f:
        f.write(edna_description_xml)

    # Create a robot_state_publisher node
    params = {'robot_description': edna_description_xml, 'use_sim_time': use_sim_time, 'publish_frequency': 50.0}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        namespace=NAMESPACE
    )

    # Starts ROS2 Control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': edna_description_xml, 'use_sim_time': True }, controllers_file],
        output="screen",
        namespace=NAMESPACE
    )

    # Starts ROS2 Control Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        namespace=NAMESPACE
    )

    #Starts ROS2 Control Swerve Drive Controller
    swerve_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_controller", "-c", "/controller_manager"],
        namespace=NAMESPACE
    )
    swerve_drive_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_drive_controller_spawner],
        )
    )

    # Start Rviz2 with basic view
    run_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{ 'use_sim_time': True }],
        name='isaac_rviz2',
        output='screen',
        arguments=[["-d"], [rviz_file]],
        namespace=NAMESPACE
    )
    rviz2_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[run_rviz2_node],
        )
    )

    # Start Joystick Node
    joy = Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            namespace=NAMESPACE,
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])

    # Start Teleop Node to translate joystick commands to robot commands
    joy_teleop = Node(
        package='teleop_twist_joy', 
        executable='teleop_node',
        name='teleop_twist_joy_node', 
        parameters=[joystick_file],
        remappings={('/cmd_vel', '/swerve_controller/cmd_vel_unstamped')},
        namespace=NAMESPACE
        )

    # Launch!
    return LaunchDescription([
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        swerve_drive_controller_delay,
        rviz2_delay,
        joy,
        joy_teleop
    ])