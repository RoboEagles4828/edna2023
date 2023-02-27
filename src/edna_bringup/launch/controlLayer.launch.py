import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition

# Easy use of namespace since args are not strings
NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    namespace = LaunchConfiguration('namespace')
    hardware_plugin = LaunchConfiguration('hardware_plugin')

    # Process the URDF file
    description_pkg_path = os.path.join(get_package_share_directory('edna_description'))
    xacro_file = os.path.join(description_pkg_path,'urdf', 'robots','edna.urdf.xacro')
    edna_description_xml = Command(['xacro ', xacro_file, ' namespace:=', namespace, ' hw_interface_plugin:=', hardware_plugin])
    
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
        }],
    )

    # Starts ROS2 Control
    control_node = Node(
        package="controller_manager",
        namespace=namespace,
        executable="ros2_control_node",
        condition=IfCondition(PythonExpression([ "'", use_ros2_control, "' == 'true'" ])),
        parameters=[{
            "robot_description": edna_description_xml,
            "use_sim_time": use_sim_time,
            "front_left_wheel_joint": f"{NAMESPACE}_front_left_wheel_joint",
            "front_right_wheel_joint": f"{NAMESPACE}_front_right_wheel_joint",
            "rear_left_wheel_joint": f"{NAMESPACE}_rear_left_wheel_joint",
            "rear_right_wheel_joint": f"{NAMESPACE}_rear_right_wheel_joint",
            "front_left_axle_joint": f"{NAMESPACE}_front_left_axle_joint",
            "front_right_axle_joint": f"{NAMESPACE}_front_right_axle_joint",
            "rear_left_axle_joint": f"{NAMESPACE}_rear_left_axle_joint",
            "rear_right_axle_joint": f"{NAMESPACE}_rear_right_axle_joint"
            }, controllers_file],
        output="both",
    )

    # Starts ROS2 Control Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", f"/{NAMESPACE}/controller_manager"],
        condition=IfCondition(PythonExpression([ "'", use_ros2_control, "' == 'true'" ])),
    )

    #Starts ROS2 Control Swerve Drive Controller
    swerve_drive_controller_spawner = Node(
        package="controller_manager",
        namespace=namespace,
        executable="spawner",
        arguments=["swerve_controller", "-c", f"/{NAMESPACE}/controller_manager"],
        condition=IfCondition(PythonExpression([ "'", use_ros2_control, "' == 'true'" ])),
    )
    swerve_drive_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_drive_controller_spawner],
        )
    )

    # Starts ROS2 Control Joint State Broadcaster
    joint_state_publisher_gui = Node (
        package='joint_state_publisher_gui',
        namespace=namespace,
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition( PythonExpression([ "'", use_ros2_control, "' == 'false'" ]) ),
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
        node_robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        swerve_drive_controller_delay,
        joint_state_publisher_gui
    ])