#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

import os


def generate_launch_description():

    ecat_master_proc = ExecuteProcess(
        cmd=['runuser', '-u', 'bluerobin', '--', 'env', 'LD_LIBRARY_PATH=/usr/xenomai/lib', 'ecat_master'],
        output='screen',
    )
    imu_proc = ExecuteProcess(
        cmd=['runuser', '-u', 'bluerobin', '--', 'env', 'LD_LIBRARY_PATH=/usr/xenomai/lib', 'tocabi_sensor'],
        output='screen',
    )

    # Get package directories
    p73_description_share = get_package_share_directory('p73_walker_description')
 
    urdf_path = os.path.join(p73_description_share, 'urdf', 'p73_walker.urdf')

    print(f"URDF path: {urdf_path}")
    #check if the model file exists
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    
    # P73 Controller node
    setting_realrobot_PDgain_path = os.path.join(p73_description_share, 'setting', 'setting_realrobot_PDgain.yaml')
    p73_controller_node = Node(
        package='p73_controller',
        executable='p73_controller',
        name='p73_controller',
        output='screen',
        parameters=[
            {
                'urdf_path': urdf_path,
                'sim_mode': False
            },
            setting_realrobot_PDgain_path
        ]
    )

    return LaunchDescription([
        p73_controller_node,
        # ecat_master_proc,
        # imu_proc,
    ])

