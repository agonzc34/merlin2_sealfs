from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    merlin2_dir = get_package_share_directory('merlin2_demo')
    sealfs_dir = get_package_share_directory('rossealfs')
    rb1_dir = get_package_share_directory('rb1_gazebo')
    self_dir = get_package_share_directory('merlin2_sealfs')
    
    sealfs_config_yaml_path = os.path.join(self_dir, 'config', 'params.yaml')
    
    # Params
    sealfs_config = LaunchConfiguration('sealfs_config')
    sealfs_config_param = DeclareLaunchArgument(
        'sealfs_config',
        default_value=sealfs_config_yaml_path,
        description='Path to sealfs config file')

    
    # Nodes
    sealfs_action_cmd = Node(
        package='rossealfs',
        executable='rossealfs',
        name='rossealfs',
        # ros_arguments=[{'params-file': sealfs_config}],
        arguments=['--ros-args', '--params-file', sealfs_config],
    )
    
    remapper_action_cmd = Node(
        package='merlin2_sealfs',
        executable='remapper',
        name='remapper_pub',
        parameters=[{'out_topic_name': '/sealfs/all'}],
    )
    
    
    # Launch
    merlin2_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(merlin2_dir, 'launch', 'merlin2_demo2.launch.py')),
    )
    
    nav2_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rb1_dir, 'launch', 'granny.launch.py')),
    )
    
    ld = LaunchDescription()
    
    ld.add_action(sealfs_config_param)
    
    # ld.add_action(nav2_action_cmd)
    ld.add_action(sealfs_action_cmd)
    ld.add_action(remapper_action_cmd)
    ld.add_action(merlin2_action_cmd)
    
    return ld        