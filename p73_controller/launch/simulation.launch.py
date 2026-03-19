#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

import os


def generate_launch_description():
    # Get package directories
    p73_description_share = get_package_share_directory('p73_walker_description')
 
    # Model file path
    model_file = os.path.join(p73_description_share, 'mujoco', 'p73_walker.xml')
    urdf_path = os.path.join(p73_description_share, 'urdf', 'p73_walker.urdf')

    setting_sim_PDgain_path = os.path.join(p73_description_share, 'setting', 'setting_sim_PDgain.yaml')
    
    #check if the model file exists
    if not os.path.exists(model_file):
        raise FileNotFoundError(f"Model file not found: {model_file}")
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # MuJoCo ROS2 node
    mujoco_node = Node(
        package='mjc_ros2',
        executable='mujoco_ros2',
        name='mujoco_ros2',
        output='screen',
        parameters=[{
            'model_file': model_file,
            'joint_names': ['L_HipRoll_Joint', 'L_HipPitch_Joint', 'L_HipYaw_Joint', 'L_Knee_Joint', 'L_AnklePitch_Joint', 'L_AnkleRoll_Joint',
                            'R_HipRoll_Joint', 'R_HipPitch_Joint', 'R_HipYaw_Joint', 'R_Knee_Joint', 'R_AnklePitch_Joint', 'R_AnkleRoll_Joint',
                            'WaistYaw_Joint']
        }]
    )
    
    # P73 Controller node
    p73_controller_node = Node(
        package='p73_controller',
        executable='p73_controller',
        name='p73_controller',
        output='screen',
        parameters=[{
                'urdf_path': urdf_path,
                'xml_path': model_file,
                'sim_mode': True
            },
            setting_sim_PDgain_path
        ]
    )

    # P73 GUI node (RQt plugin)
    # RQt will save/restore window position and size automatically
    p73_gui_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='p73_gui',
        output='screen',
        arguments=['--force-discover', '--standalone', 'p73_gui/P73Gui']
    )

    delayed_p73_controller_node = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco_node,
            on_start=[TimerAction(period=0.2, actions=[p73_controller_node])]
        )
    )

    return LaunchDescription([
        mujoco_node,
        delayed_p73_controller_node,
        p73_gui_node,
    ])

