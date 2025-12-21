#!/usr/bin/env python3
"""
Complete SAUVC Mission Launch File
Location: src/auv_slam/launch/complete_mission.launch.py

Mission Flow:
1. Flare Bumping Task (red/yellow/blue in specified order)
2. Navigation Gate Task
3. Additional tasks (if implemented)

Optimized for Radxa X4 Single Board Computer
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
    flare_params = os.path.join(auv_slam_share, 'config', 'flare_params.yaml')
    gate_params = os.path.join(auv_slam_share, 'config', 'gate_params.yaml')
    safety_params = os.path.join(auv_slam_share, 'config', 'safety_params.yaml')
    world_file = os.path.join(auv_slam_share, 'worlds', 'underwater_world_ign.sdf')
    
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
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='-20.0',
        description='X position for robot spawn (starting zone)'
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y position for robot spawn'
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='-1.3',
        description='Z position for robot spawn (underwater)'
    )
    
    declare_enable_gate_task = DeclareLaunchArgument(
        'enable_gate_task',
        default_value='true',
        description='Enable gate navigation task after flare task'
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
    
    # 3. Gazebo Simulator
    gazebo_process = ExecuteProcess(
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', '3', world_file],
        output='screen',
        additional_env=gz_env,
        shell=False
    )
    
    # 4. Spawn Robot Entity (Delayed 2 seconds)
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
                    "-z", LaunchConfiguration('spawn_z'),
                    "-x", LaunchConfiguration('spawn_x'),
                    "-y", LaunchConfiguration('spawn_y'),
                    "-Y", "0.0",
                    "--ros-args", "--log-level", "warn"
                ],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
    
    # 5. ROS-Gazebo Bridge (Delayed 3 seconds)
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
    
    # ========================================================================
    # CONTROL & NAVIGATION NODES
    # ========================================================================
    
    # 6. Thruster Mapper (Delayed 3 seconds)
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
    
    # ========================================================================
    # FLARE TASK NODES (TASK 1)
    # ========================================================================
    
    # 7. Flare Detector (Delayed 5 seconds)
    flare_detector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_slam',
                executable='flare_detector_node.py',
                name='flare_detector',
                output='screen',
                parameters=[flare_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # 8. Flare Navigator (Delayed 5 seconds)
    flare_navigator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_slam',
                executable='flare_navigator_node.py',
                name='flare_navigator',
                output='screen',
                parameters=[flare_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # ========================================================================
    # GATE TASK NODES (TASK 2 - DISABLED DURING FLARE TASK)
    # ========================================================================
    
    # 9. Gate Detector (Delayed 10 seconds, starts disabled)
    # Will be enabled after flare mission completes
    gate_detector = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='auv_slam',
                executable='gate_detector_node.py',
                name='gate_detector',
                output='screen',
                parameters=[gate_params, {'use_sim_time': True}],
                # Start paused - will be activated after flare task
                condition=IfCondition(LaunchConfiguration('enable_gate_task'))
            )
        ]
    )
    
    # 10. Gate Navigator (Delayed 10 seconds, starts disabled)
    gate_navigator = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='auv_slam',
                executable='gate_navigator_node.py',
                name='gate_navigator',
                output='screen',
                parameters=[gate_params, {'use_sim_time': True}],
                # Start paused - will be activated after flare task
                condition=IfCondition(LaunchConfiguration('enable_gate_task'))
            )
        ]
    )
    
    # ========================================================================
    # SAFETY & MONITORING NODES
    # ========================================================================
    
    # 11. Safety Monitor (Delayed 3 seconds)
    safety_monitor = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='auv_slam',
                executable='safety_monitor_node.py',
                name='safety_monitor',
                output='screen',
                parameters=[safety_params, {'use_sim_time': True}]
            )
        ]
    )
    
    # ========================================================================
    # VISUALIZATION NODES
    # ========================================================================
    
    # 12. RViz (Optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # 13. Debug Image Viewer for Flares
    flare_debug_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='flare_debug_viewer',
        arguments=['/flare/debug_image'],
        parameters=[{'use_sim_time': True}]
    )
    
    # 14. Debug Image Viewer for Gate (disabled initially)
    gate_debug_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='gate_debug_viewer',
        arguments=['/gate/debug_image'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('enable_gate_task'))
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Launch arguments
        declare_enable_rviz,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_enable_gate_task,
        
        # Core simulation (immediate start)
        robot_state_publisher,
        joint_state_publisher,
        gazebo_process,
        
        # Timed launches - Simulation setup
        spawn_entity,           # 2s delay
        bridge,                 # 3s delay
        thruster_mapper,        # 3s delay
        safety_monitor,         # 3s delay
        
        # Timed launches - Flare Task (TASK 1)
        flare_detector,         # 5s delay
        flare_navigator,        # 5s delay
        
        # Timed launches - Gate Task (TASK 2) - Optional
        gate_detector,          # 10s delay (conditional)
        gate_navigator,         # 10s delay (conditional)
        
        # Visualization
        rviz_node,              # Optional
        flare_debug_viewer,     # Always on
        gate_debug_viewer,      # Conditional
    ])


if __name__ == '__main__':
    generate_launch_description()