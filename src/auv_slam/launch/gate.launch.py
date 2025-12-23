#!/usr/bin/env python3
"""
IMPROVED Autonomous White Gate Navigation Launch File
With better camera bridge and diagnostics
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import launch_ros.descriptions


def generate_launch_description():
    auv_slam_share = get_package_share_directory('auv_slam')
    
    # Paths
    urdf_file = os.path.join(auv_slam_share, 'urdf', 'orca4_description.urdf')
    rviz_config = os.path.join(auv_slam_share, 'rviz', 'urdf_config.rviz')
    bridge_config = os.path.join(auv_slam_share, 'config', 'ign_bridge.yaml')
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    world_file = os.path.join(auv_slam_share, 'worlds', 'white_gate_autonomous.sdf')
    
    print("="*70)
    print("🤖 AUTONOMOUS WHITE GATE NAVIGATION - IMPROVED LAUNCH")
    print("="*70)
    print(f"World: {world_file}")
    print(f"URDF: {urdf_file}")
    print(f"Bridge: {bridge_config}")
    print("="*70)
    
    # Verify files exist
    for file_path, name in [(world_file, "World"), (urdf_file, "URDF")]:
        if not os.path.exists(file_path):
            print(f"❌ ERROR: {name} file not found: {file_path}")
            return LaunchDescription([])
    
    print("✅ All files found")
    print("="*70)
    
    # Gazebo environment
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
    
    # Launch arguments
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    declare_enable_debug = DeclareLaunchArgument(
        'enable_debug_view',
        default_value='true',
        description='Launch debug image viewer'
    )
    
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Verbose output'
    )
    
    # 1. Robot State Publisher
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
    
    # 2. Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 3. Gazebo Simulator
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen',
        additional_env=gz_env,
        shell=False
    )
    
    # 4. Spawn Robot (3s delay)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-name", "orca4_ign",
                    "-topic", "robot_description",
                    "-z", "0.3",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-Y", "0.0",
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
    
    # 5. ROS-Gazebo Bridge (4s delay)
    bridge = TimerAction(
        period=4.0,
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
    
    # 6. Additional image bridge for front_left camera (5s delay)
    # This ensures the camera topic is properly bridged
    camera_bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                name='front_left_camera_bridge',
                arguments=['/front_left/image_raw'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 7. Thruster Mapper (5s delay)
    thruster_mapper = TimerAction(
        period=5.0,
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
    
    # 8. Autonomous Navigation (7s delay)
    autonomous_nav = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='auv_slam',
                executable='white_gate_navigator.py',
                name='autonomous_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 9. Camera diagnostics (6s delay)
    camera_diagnostics = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'hz', '/front_left/image_raw'],
                name='camera_hz_check',
                output='screen',
                shell=False
            )
        ]
    )
    
    # 10. RViz (Optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # 11. Debug Image Viewer (8s delay)
    debug_viewer = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='debug_image_viewer',
                arguments=['/autonomous/debug_image'],
                parameters=[{'use_sim_time': True}],
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_debug_view'))
            )
        ]
    )
    
    # Launch sequence
    return LaunchDescription([
        # Arguments
        declare_enable_rviz,
        declare_enable_debug,
        declare_verbose,
        
        # Core (immediate)
        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        
        # Timed launches
        spawn_entity,        # 3s
        bridge,              # 4s
        camera_bridge,       # 5s - CRITICAL for camera
        thruster_mapper,     # 5s
        autonomous_nav,      # 7s
        debug_viewer,        # 8s
        
        # Optional
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()