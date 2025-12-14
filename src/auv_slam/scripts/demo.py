#!/usr/bin/env python3
"""
AUV Motion Demonstration Script
1. Moves forward 2m
2. Performs 4-DOF demonstrations at -0.8m depth
3. 5-second gaps between operations
4. Clean logging with start/end markers

Usage:
    Launched automatically by demo.launch.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion


class MotionDemo(Node):
    def __init__(self):
        super().__init__('motion_demo_node')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Subscriber for odometry feedback
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Target depth (negative is below water surface)
        self.TARGET_DEPTH = -0.8  # 0.8m below water surface
        
        # Current state
        self.current_position = None
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Demo parameters
        self.motion_duration = 4.0  # seconds per motion
        self.gap_duration = 5.0     # 5 seconds between operations
        
        # Depth control parameters
        self.depth_control_gain = 2.0
        self.depth_deadband = 0.1
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 AUV MOTION DEMONSTRATION')
        self.get_logger().info('='*70)
        self.get_logger().info(f'   Target depth: {self.TARGET_DEPTH}m')
        self.get_logger().info('   Sequence:')
        self.get_logger().info('   1. Move forward 2m')
        self.get_logger().info('   2. HEAVE demonstration')
        self.get_logger().info('   3. SURGE demonstration')
        self.get_logger().info('   4. YAW demonstration')
        self.get_logger().info('   5. ROLL demonstration')
        self.get_logger().info('   Gap between operations: 5 seconds')
        self.get_logger().info('='*70)
        
        # Wait for odometry
        self.get_logger().info('⏳ Waiting for odometry...')
        while self.current_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('✅ Odometry ready!')
        self.get_logger().info(f'   Starting position: X={self.current_position[0]:.2f}m, Y={self.current_position[1]:.2f}m, Z={self.current_position[2]:.2f}m')
        self.get_logger().info('')
        self.get_logger().info('Starting in 3 seconds...')
        time.sleep(3)
    
    def odom_callback(self, msg: Odometry):
        """Update current state from odometry"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.current_depth = msg.pose.pose.position.z
        
        # Convert quaternion to euler angles
        q = msg.pose.pose.orientation
        self.current_roll, self.current_pitch, self.current_yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
    
    def compute_depth_control(self, target_depth: float) -> float:
        """Compute depth control command"""
        depth_error = target_depth - self.current_depth
        
        if abs(depth_error) < self.depth_deadband:
            return 0.0
        
        z_cmd = depth_error * self.depth_control_gain
        return max(-0.8, min(z_cmd, 0.8))
    
    def publish_cmd(self, linear_x=0.0, linear_y=0.0, target_depth=None,
                    angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Publish velocity command with automatic depth control"""
        if target_depth is None:
            target_depth = self.TARGET_DEPTH
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.linear.z = self.compute_depth_control(target_depth)
        cmd.angular.x = angular_x
        cmd.angular.y = angular_y
        cmd.angular.z = angular_z
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop(self):
        """Stop all motion but maintain depth"""
        self.publish_cmd(0.0, 0.0)
    
    def hold_depth(self, duration: float, target_depth: float = None):
        """Hold at target depth for specified duration"""
        if target_depth is None:
            target_depth = self.TARGET_DEPTH
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publish_cmd(0.0, 0.0, target_depth)
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def log_operation_start(self, operation_name: str, description: str):
        """Log start of operation"""
        self.get_logger().info('')
        self.get_logger().info('╔' + '='*68 + '╗')
        self.get_logger().info(f'║ ▶️  STARTING: {operation_name:<54} ║')
        self.get_logger().info(f'║     {description:<62} ║')
        self.get_logger().info('╚' + '='*68 + '╝')
    
    def log_operation_end(self, operation_name: str, start_pos, final_stats: dict):
        """Log end of operation"""
        dx = self.current_position[0] - start_pos[0]
        dy = self.current_position[1] - start_pos[1]
        dz = self.current_depth - start_pos[2]
        
        self.get_logger().info('')
        self.get_logger().info('╔' + '='*68 + '╗')
        self.get_logger().info(f'║ ✅ COMPLETED: {operation_name:<53} ║')
        self.get_logger().info(f'║     Movement: ΔX={dx:+.3f}m, ΔY={dy:+.3f}m, ΔZ={dz:+.3f}m{" "*20}║')
        self.get_logger().info(f'║     Final Depth: {self.current_depth:.3f}m{" "*44}║')
        if 'yaw_change' in final_stats:
            self.get_logger().info(f'║     Yaw Change: {final_stats["yaw_change"]:+.1f}°{" "*44}║')
        if 'roll_change' in final_stats:
            self.get_logger().info(f'║     Roll Change: {final_stats["roll_change"]:+.1f}°{" "*43}║')
        self.get_logger().info('╚' + '='*68 + '╝')
    
    def execute_operation(self, name: str, description: str,
                         linear_x=0.0, linear_y=0.0, target_depth=None,
                         angular_x=0.0, angular_y=0.0, angular_z=0.0,
                         duration=None):
        """Execute a single operation with logging"""
        
        if duration is None:
            duration = self.motion_duration
        
        if target_depth is None:
            target_depth = self.TARGET_DEPTH
        
        # Log start
        self.log_operation_start(name, description)
        
        # Record starting values
        start_pos = self.current_position
        start_yaw = self.current_yaw
        start_roll = self.current_roll
        
        # Execute motion
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publish_cmd(linear_x, linear_y, target_depth,
                           angular_x, angular_y, angular_z)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop
        self.stop()
        time.sleep(0.5)
        
        # Calculate final stats
        final_stats = {
            'yaw_change': math.degrees(self.current_yaw - start_yaw),
            'roll_change': math.degrees(self.current_roll - start_roll)
        }
        
        # Log end
        self.log_operation_end(name, start_pos, final_stats)
        
        # Gap between operations
        if self.gap_duration > 0:
            self.get_logger().info('')
            self.get_logger().info(f'⏸️  Waiting {self.gap_duration:.0f} seconds before next operation...')
            self.hold_depth(self.gap_duration, target_depth)
    
    def move_forward_2m(self):
        """Move forward approximately 2 meters"""
        self.log_operation_start("INITIAL FORWARD MOVEMENT", "Moving 2m forward to starting position")
        
        start_pos = self.current_position
        target_distance = 2.0
        
        # Move forward until 2m traveled
        while True:
            dx = self.current_position[0] - start_pos[0]
            dy = self.current_position[1] - start_pos[1]
            distance_traveled = math.sqrt(dx*dx + dy*dy)
            
            if distance_traveled >= target_distance:
                break
            
            self.publish_cmd(linear_x=0.5)
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop
        self.stop()
        time.sleep(1.0)
        
        # Log completion
        final_stats = {}
        self.log_operation_end("INITIAL FORWARD MOVEMENT", start_pos, final_stats)
        
        self.get_logger().info('')
        self.get_logger().info(f'⏸️  Waiting {self.gap_duration:.0f} seconds before demonstrations...')
        self.hold_depth(self.gap_duration)
    
    def run_demonstration(self):
        """Run complete demonstration sequence"""
        
        try:
            # ============================================
            # STEP 0: SUBMERGE TO TARGET DEPTH
            # ============================================
            self.get_logger().info('')
            self.get_logger().info('╔' + '='*68 + '╗')
            self.get_logger().info('║ 📉 SUBMERGING TO TARGET DEPTH                                   ║')
            self.get_logger().info('╚' + '='*68 + '╝')
            self.hold_depth(duration=5.0)
            self.get_logger().info(f'✅ At target depth: {self.current_depth:.3f}m')
            time.sleep(2.0)
            
            # ============================================
            # STEP 1: MOVE FORWARD 2M
            # ============================================
            self.move_forward_2m()
            
            # ============================================
            # STEP 2: HEAVE DEMONSTRATION
            # ============================================
            self.execute_operation(
                name="HEAVE UP",
                description="Ascending to -0.5m (0.3m shallower)",
                target_depth=-0.5
            )
            
            self.execute_operation(
                name="HEAVE DOWN",
                description="Descending to -1.1m (0.3m deeper)",
                target_depth=-1.1
            )
            
            # Return to target depth
            self.get_logger().info('')
            self.get_logger().info('🎯 Returning to target depth (-0.8m)...')
            self.hold_depth(duration=3.0)
            self.get_logger().info(f'✅ At target depth: {self.current_depth:.3f}m')
            time.sleep(self.gap_duration)
            
            # ============================================
            # STEP 3: SURGE DEMONSTRATION
            # ============================================
            self.execute_operation(
                name="SURGE FORWARD",
                description="Moving forward at 0.6 m/s (depth locked at -0.8m)",
                linear_x=0.6
            )
            
            self.execute_operation(
                name="SURGE BACKWARD",
                description="Moving backward at 0.6 m/s (depth locked at -0.8m)",
                linear_x=-0.6
            )
            
            # ============================================
            # STEP 4: YAW DEMONSTRATION
            # ============================================
            self.execute_operation(
                name="YAW RIGHT",
                description="Rotating clockwise at 0.5 rad/s (depth locked)",
                angular_z=-0.5
            )
            
            self.execute_operation(
                name="YAW LEFT",
                description="Rotating counter-clockwise at 0.5 rad/s (depth locked)",
                angular_z=0.5
            )
            
            # ============================================
            # STEP 5: ROLL DEMONSTRATION
            # ============================================
            self.execute_operation(
                name="ROLL RIGHT",
                description="Attempting roll (limited capability due to symmetric design)",
                angular_x=0.3
            )
            
            self.execute_operation(
                name="ROLL LEFT",
                description="Attempting roll (limited capability due to symmetric design)",
                angular_x=-0.3
            )
            
            # ============================================
            # FINAL: MISSION COMPLETE
            # ============================================
            self.stop()
            
            self.get_logger().info('')
            self.get_logger().info('╔' + '='*68 + '╗')
            self.get_logger().info('║ 🎉 MISSION COMPLETE!                                            ║')
            self.get_logger().info('╚' + '='*68 + '╝')
            self.get_logger().info('')
            self.get_logger().info('Summary:')
            self.get_logger().info('  ✅ Initial forward movement: 2m')
            self.get_logger().info('  ✅ HEAVE demonstrations: 2 operations')
            self.get_logger().info('  ✅ SURGE demonstrations: 2 operations')
            self.get_logger().info('  ✅ YAW demonstrations: 2 operations')
            self.get_logger().info('  ✅ ROLL demonstrations: 2 operations')
            self.get_logger().info('')
            self.get_logger().info(f'Final Position: X={self.current_position[0]:.2f}m, Y={self.current_position[1]:.2f}m')
            self.get_logger().info(f'Final Depth: {self.current_depth:.3f}m')
            self.get_logger().info(f'Final Yaw: {math.degrees(self.current_yaw):.1f}°')
            self.get_logger().info('')
            
        except KeyboardInterrupt:
            self.get_logger().info('⚠️  Interrupted by user')
            self.stop()
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    
    demo_node = MotionDemo()
    
    try:
        demo_node.run_demonstration()
    except KeyboardInterrupt:
        pass
    finally:
        demo_node.stop()
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()