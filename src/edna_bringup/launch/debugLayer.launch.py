from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import yaml

def processRvizFileForNamespace(rviz_file, NAMESPACE):
    rviz_data = None
    with open(rviz_file, 'r') as stream:
        rviz_data = yaml.safe_load(stream)

    for display in rviz_data['Visualization Manager']['Displays']:
        for k, v in display.items():
            if 'Topic' in k and 'Value' in v:
                print(f"mapping {v['Value']} -> /{NAMESPACE}{v['Value']}")
                v['Value'] = f"/{NAMESPACE}{v['Value']}"

    print("Writing tmp rviz file")
    with open(rviz_file, 'w') as stream:
        yaml.dump(rviz_data, stream)

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_foxglove = LaunchConfiguration('enable_foxglove')
    rviz_file = LaunchConfiguration('rviz_file')


    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        namespace=namespace,
        parameters=[{
            'port': 8765,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_foxglove)
    )

    rviz2 = Node(
        package='rviz2',
        namespace=namespace,
        executable='rviz2',
        parameters=[{ 'use_sim_time': use_sim_time }],
        output='screen',
        arguments=[["-d"], [rviz_file]],
        condition=IfCondition(enable_rviz)
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
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='enables rviz'),
        DeclareLaunchArgument(
            'rviz_file',
            default_value='',
            description='The config file for rviz'),
        DeclareLaunchArgument(
            'enable_foxglove',
            default_value='true',
            description='enables foxglove bridge'),
            foxglove,
            rviz2
    ])