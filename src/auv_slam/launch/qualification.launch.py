#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
import launch_ros.descriptions

def generate_launch_description():
    auv_slam_share = get_package_share_directory('auv_slam')
    
    urdf_file = os.path.join(auv_slam_share, 'urdf', 'orca4_description.urdf')
    rviz_config = os.path.join(auv_slam_share, 'rviz', 'urdf_config.rviz')
    bridge_config = os.path.join(auv_slam_share, 'config', 'ign_bridge.yaml')
    
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    qual_params = os.path.join(auv_slam_share, 'config', 'qualification_params.yaml')
    
    world_file = os.path.join(auv_slam_share, 'worlds', 'qualification_world.sdf')
    
    gz_models_path = os.path.join(auv_slam_share, "models")
    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", default="")
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')]),
        'GZ_SIM_RESOURCE_PATH':
                      ':'.join([gz_resource_path, gz_models_path])
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str
            ),
            'use_sim_time': True
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    gazebo_process = ExecuteProcess(
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', '3', world_file],
        output='screen',
        additional_env=gz_env,
        shell=False
    )

    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name", "orca4_ign",
                    "-topic", "robot_description",
                    "-z", "0.2",
                    "-x", "-12.0",
                    "-y", "0.0",
                    "-Y", "0.0",
                    "--ros-args", "--log-level", "warn"
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    '--ros-args',
                    '-p', f'config_file:={bridge_config}'
                ],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    thruster_mapper = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='auv_slam',
                executable='simple_thruster_mapper.py',
                name='thruster_mapper',
                output='screen',
                parameters=[thruster_params, {'use_sim_time': True}]
            )
        ]
    )
    
    stereo_gate_detector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_slam',
                executable='stereo_qualification_detector_node.py',
                name='stereo_qualification_detector',
                output='screen',
                parameters=[qual_params, {'use_sim_time': True}]
            )
        ]
    )
    
    navigator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_slam',
                executable='qualification_navigator_node.py',
                name='qualification_navigator',
                output='screen',
                parameters=[qual_params, {'use_sim_time': True}]
            )
        ]
    )
    
    safety_monitor = Node(
        package='auv_slam',
        executable='safety_monitor_node.py',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'max_depth': -1.55,
            'min_depth': 0.2,
            'max_roll': 0.785,
            'max_pitch': 0.785,
            'watchdog_timeout': 5.0,
            'max_mission_time': 36000.0,
            'pool_bounds_x': [-12.5, 12.5],
            'pool_bounds_y': [-8.0, 8.0],
            'use_sim_time': True
        }]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
    
    rqt_debug_view = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_detection_view',
                arguments=['/qualification/debug_image'],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    imu_monitor = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='auv_slam',
                executable='imu_monitor_node.py',
                name='imu_monitor',
                output='screen',
                prefix='xterm -e',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        spawn_entity,
        bridge,
        thruster_mapper,
        stereo_gate_detector,
        navigator,
        safety_monitor,
        rqt_debug_view,
        imu_monitor,
        rviz_node
    ])

if __name__ == '__main__':
    generate_launch_description()