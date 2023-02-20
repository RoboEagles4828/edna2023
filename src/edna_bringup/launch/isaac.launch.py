import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import yaml

import xacro

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'default'

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
    tmp_rviz_file = os.path.join(bringup_pkg_path, 'config', 'tmp_view.rviz')
    
    # Save Built URDF file to Description Directory
    description_source_code_path = os.path.abspath(os.path.join(description_pkg_path, "../../../../src/edna_description/urdf"))
    urdf_save_path = os.path.join(description_source_code_path, "edna.urdf")
    with open(urdf_save_path, 'w') as f:
        f.write(edna_description_xml)

    # Modify the Rviz file for the correct namespace
    rviz_data = None
    with open(rviz_file, 'r') as stream:
        rviz_data = yaml.safe_load(stream)

    for display in rviz_data['Visualization Manager']['Displays']:
        for k, v in display.items():
            if 'Topic' in k and 'Value' in v:
                print(f"mapping {v['Value']} -> /{NAMESPACE}{v['Value']}")
                v['Value'] = f"/{NAMESPACE}{v['Value']}"

    print("Writing tmp rviz file")
    with open(tmp_rviz_file, 'w') as stream:
        yaml.dump(rviz_data, stream)
    

    # Create a robot_state_publisher node
    params = {'robot_description': edna_description_xml, 'use_sim_time': use_sim_time, 'publish_frequency': 50.0}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace=NAMESPACE,
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # Starts ROS2 Control
    control_node = Node(
        package="controller_manager",
        namespace=NAMESPACE,
        executable="ros2_control_node",
        parameters=[{'robot_description': edna_description_xml, 'use_sim_time': use_sim_time }, controllers_file],
        output="both",
    )

    # Starts ROS2 Control Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=NAMESPACE,
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", f"/{NAMESPACE}/controller_manager"],
    )

    #Starts ROS2 Control Swerve Drive Controller
    swerve_drive_controller_spawner = Node(
        package="controller_manager",
        namespace=NAMESPACE,
        executable="spawner",
        arguments=["swerve_controller", "--controller-manager", f"/{NAMESPACE}/controller_manager"],
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
        namespace=NAMESPACE,
        executable='rviz2',
        parameters=[{ 'use_sim_time': True }],
        name='isaac_rviz2',
        output='screen',
        arguments=[["-d"], [tmp_rviz_file]],
    )
    rviz2_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[run_rviz2_node],
        )
    )

    # Start Foxglove Bridge
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'port': 8765,
        }],
    )

    # Start Joystick Node
    joy = Node(
            package='joy',
            namespace=NAMESPACE,
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])

    # Start Teleop Node to translate joystick commands to robot commands
    joy_teleop = Node(
        package='teleop_twist_joy',
        namespace=NAMESPACE,
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joystick_file],
        remappings={(f'/{NAMESPACE}/cmd_vel', f'/{NAMESPACE}/swerve_controller/cmd_vel_unstamped')},
        )

    # Launch!
    return LaunchDescription([
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        swerve_drive_controller_delay,
        rviz2_delay,
        joy,
        joy_teleop,
        # foxglove
    ])