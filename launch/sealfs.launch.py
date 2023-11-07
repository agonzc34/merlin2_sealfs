from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, OpaqueFunction
import yaml
import os


def generate_launch_description():
    self_dir = get_package_share_directory('merlin2_sealfs')
    rossealfs_dir = get_package_share_directory('rossealfs')    
    
    file = open(os.path.join(self_dir, 'config', 'params2.yaml'), 'r')
    params = yaml.safe_load(file)
    rossealfs_params = params['sealfs_params']['rossealfs']
    sealfs_params = params['sealfs_params']['sealfs']
        
    # Processes
    sealfs_startup = ExecuteProcess(
        cmd=[os.path.join(self_dir, 'scripts', 'sealfs.sh')],
        env={'ROSSEALFS_MOUNTPOINT': rossealfs_params['mountpoint'],
             'SEALFS_BACK': sealfs_params['backpoint'],
             'SEALFS_PATH': sealfs_params['sealfs_path'],
             'SEALFS_SIZE': str(sealfs_params['size']),
             'SEALFS_KEY_PATH': sealfs_params['keys_dir'],
             'SEALFS_NRATCHET': str(sealfs_params['nratchet']),
             },
        output='screen'
    )
    
    # sealfs_shutdown = ExecuteProcess(
    #     cmd=[os.path.join(self_dir, 'scripts', 'shutdown.sh')],
    #     env={'ROSSEALFS_MOUNTPOINT': rossealfs_params['mountpoint'],
    #         'SEALFS_PATH': sealfs_params['sealfs_path'],
    #         'SEALFS_KEY_PATH': sealfs_params['keys_dir'],
    #         },
    #     output='screen',
    # )
    
    # Nodes
    sealfs_action_cmd = Node(
        package='rossealfs',
        executable='rossealfs',
        name='rossealfs',
        # ros_arguments=[{'params-file': sealfs_config}],
        parameters=[rossealfs_params]
    )
    
    remapper_action_cmd = Node(
        package='merlin2_sealfs',
        executable='remapper',
        name='remapper_pub',
        parameters=[{'out_topic_name': '/sealfs/all'}],
    )
    
    
    
    shutdown_event = RegisterEventHandler(OnShutdown(
        on_shutdown=[
            LogInfo(msg='Shutting down sealfs...'),
        ],
    ))
    
    ld = LaunchDescription()
    
    ld.add_action(sealfs_startup)
    ld.add_action(sealfs_action_cmd)
    ld.add_action(remapper_action_cmd)
    
    ld.add_action(shutdown_event)
            
    return ld        