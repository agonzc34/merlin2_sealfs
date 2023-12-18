from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown, OnProcessExit, OnExecutionComplete, OnProcessStart
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.substitutions import FindExecutable
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
    sealfs_params = params['sealfs_params']['sealfs']
        
    # Processes
    sealfs_startup = ExecuteProcess(
        cmd=[[FindExecutable(name='sh'), ' ', os.path.join(self_dir, 'scripts', 'sealfs.sh')]],
        env={'ROSSEALFS_MOUNTPOINT': rossealfs_params['mountpoint'],
             'SEALFS_BACK': sealfs_params['backpoint'],
             'SEALFS_PATH': sealfs_params['sealfs_path'],
             'SEALFS_SIZE': str(sealfs_params['size']),
             'SEALFS_KEY_PATH': sealfs_params['keys_dir'],
             'SEALFS_NRATCHET': str(sealfs_params['nratchet']),
             },
        # output='screen',
        shell=True,
    )
    
    shutdown_env = {'ROSSEALFS_MOUNTPOINT': rossealfs_params['mountpoint']}
    
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
    
    module_loaded_event = RegisterEventHandler(OnProcessExit(
        target_action=sealfs_startup,
        on_exit=[LogInfo(msg='Sealfs module loaded'), sealfs_action_cmd],
    ))
    
    shutdown_event = RegisterEventHandler(OnShutdown(
        on_shutdown=[
            LogInfo(msg='Sealfs module unloaded'),
            OpaqueFunction(function=shutdown_callback, kwargs=shutdown_env)
        ],
    ))
    
    ld = LaunchDescription()
    
    ld.add_action(remapper_action_cmd)
    ld.add_action(sealfs_startup)
    
    ld.add_action(shutdown_event)
    ld.add_action(module_loaded_event)
            
    return ld        