#!/usr/bin/env python3
"""
FIXED Autonomous White Gate Navigation Launch File
Includes debugging output and proper spawning
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
    
    # ========================================================================
    # PATHS
    # ========================================================================
    urdf_file = os.path.join(auv_slam_share, 'urdf', 'orca4_description.urdf')
    rviz_config = os.path.join(auv_slam_share, 'rviz', 'urdf_config.rviz')
    bridge_config = os.path.join(auv_slam_share, 'config', 'ign_bridge.yaml')
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    world_file = os.path.join(auv_slam_share, 'worlds', 'white_gate_autonomous.sdf')
    
    print("="*70)
    print("AUTONOMOUS WHITE GATE NAVIGATION - LAUNCH")
    print("="*70)
    print(f"World file: {world_file}")
    print(f"URDF file: {urdf_file}")
    print(f"Bridge config: {bridge_config}")
    print("="*70)
    
    # Check if files exist
    if not os.path.exists(world_file):
        print(f"ERROR: World file not found at {world_file}")
        print("Please ensure white_gate_autonomous.sdf is in the worlds directory")
        return LaunchDescription([])
    
    if not os.path.exists(urdf_file):
        print(f"ERROR: URDF file not found at {urdf_file}")
        return LaunchDescription([])
    
    # ========================================================================
    # GAZEBO ENVIRONMENT SETUP
    # ========================================================================
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
    
    print(f"Gazebo models path: {gz_models_path}")
    print("="*70)
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    declare_enable_debug_view = DeclareLaunchArgument(
        'enable_debug_view',
        default_value='true',
        description='Launch debug image viewer'
    )
    
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Enable verbose output'
    )
    
    # ========================================================================
    # CORE SIMULATION NODES
    # ========================================================================
    
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
    
    # 3. Gazebo Simulator (with verbose output)
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen',
        additional_env=gz_env,
        shell=False
    )
    
    # 4. Spawn Robot Entity (Delayed 3 seconds for Gazebo to fully start)
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
    
    # 5. ROS-Gazebo Bridge (Delayed 4 seconds)
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
    
    # ========================================================================
    # CONTROL NODES
    # ========================================================================
    
    # 6. Thruster Mapper (Delayed 5 seconds)
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
    
    # 7. Autonomous Navigation Node (Delayed 7 seconds)
    autonomous_nav = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='auv_slam',
                executable='improved_white_gate_nav.py',
                name='autonomous_navigator',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # ========================================================================
    # VISUALIZATION NODES
    # ========================================================================
    
    # 8. RViz (Optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # 9. Debug Image Viewer (Delayed 8 seconds)
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
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Launch arguments
        declare_enable_rviz,
        declare_enable_debug_view,
        declare_verbose,
        
        # Core simulation (immediate start)
        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        
        # Timed launches with longer delays
        spawn_entity,       # 3s delay
        bridge,             # 4s delay
        thruster_mapper,    # 5s delay
        autonomous_nav,     # 7s delay
        debug_viewer,       # 8s delay
        
        # Optional visualization
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()