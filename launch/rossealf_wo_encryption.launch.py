from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import subprocess
import yaml
import os

def shutdown_callback(**kwargs):
    subprocess.run(['bash', os.path.join(get_package_share_directory('merlin2_sealfs'), 'scripts', 'shutdown.sh')], env=kwargs)

def generate_launch_description():
    self_dir = get_package_share_directory('merlin2_sealfs')
    
    file = open(os.path.join(self_dir, 'config', 'params2.yaml'), 'r')
    params = yaml.safe_load(file)
    rossealfs_params = params['sealfs_params']['rossealfs']
        
    # Nodes
    sealfs_action_cmd = Node(
        package='rossealfs',
        executable='rossealfs',
        name='rossealfs',
        parameters=[rossealfs_params]
    )
    
    remapper_action_cmd = Node(
        package='merlin2_sealfs',
        executable='remapper',
        name='remapper_pub',
        parameters=[{'out_topic_name': '/sealfs/all'}],
    )
    

    
    ld = LaunchDescription()
    
    ld.add_action(remapper_action_cmd)
    ld.add_action(sealfs_action_cmd)
            
    return ld        