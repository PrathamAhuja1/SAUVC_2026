#!/usr/bin/env python3
"""
AUV Motion Demonstration Script
Sequence:
1. Heave to -0.8m depth
2. Roll left/right while at depth
3. Surface to -0.2m
4. Surge forward
5. Surge backward
6. Yaw left
7. Yaw right
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
        
        # Current state
        self.current_position = None
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Demo parameters
        self.motion_duration = 5.0  # seconds per motion
        self.gap_duration = 3.0     # gap between motions
        
        # Depth control
        self.depth_control_gain = 2.5
        self.depth_deadband = 0.1
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 AUV MOTION DEMONSTRATION')
        self.get_logger().info('='*70)
        self.get_logger().info('   Sequence:')
        self.get_logger().info('   1. HEAVE DOWN to -0.8m')
        self.get_logger().info('   2. ROLL (left/right) at depth')
        self.get_logger().info('   3. SURFACE to -0.2m')
        self.get_logger().info('   4. SURGE FORWARD')
        self.get_logger().info('   5. SURGE BACKWARD')
        self.get_logger().info('   6. YAW LEFT')
        self.get_logger().info('   7. YAW RIGHT')
        self.get_logger().info('='*70)
        
        # Wait for odometry
        self.get_logger().info('⏳ Waiting for odometry...')
        while self.current_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('✅ Odometry ready!')
        self.get_logger().info(f'   Starting depth: {self.current_depth:.3f}m')
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
        return max(-1.0, min(z_cmd, 1.0))
    
    def publish_cmd(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                    angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.linear.z = linear_z
        cmd.angular.x = angular_x
        cmd.angular.y = angular_y
        cmd.angular.z = angular_z
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop(self):
        """Stop all motion"""
        self.publish_cmd(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    def hold_depth(self, duration: float, target_depth: float):
        """Hold at target depth for specified duration"""
        start_time = time.time()
        while (time.time() - start_time) < duration:
            z_cmd = self.compute_depth_control(target_depth)
            self.publish_cmd(0.0, 0.0, z_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
    
    def log_start(self, name: str):
        """Log operation start"""
        self.get_logger().info('')
        self.get_logger().info('╔' + '='*68 + '╗')
        self.get_logger().info(f'║ ▶️  {name:<63} ║')
        self.get_logger().info('╚' + '='*68 + '╝')
    
    def log_end(self, name: str):
        """Log operation end"""
        self.get_logger().info('╔' + '='*68 + '╗')
        self.get_logger().info(f'║ ✅ COMPLETED: {name:<53} ║')
        self.get_logger().info(f'║    Depth: {self.current_depth:.3f}m | Roll: {math.degrees(self.current_roll):+.1f}° | Yaw: {math.degrees(self.current_yaw):+.1f}°{" "*10}║')
        self.get_logger().info('╚' + '='*68 + '╝')
        self.get_logger().info('')
    
    def heave_to_depth(self, target_depth: float, operation_name: str):
        """Heave to target depth"""
        self.log_start(operation_name)
        
        timeout = 20.0
        start_time = time.time()
        
        while abs(self.current_depth - target_depth) > 0.15:
            if (time.time() - start_time) > timeout:
                self.get_logger().warn('⏰ Timeout reaching target depth')
                break
            
            z_cmd = self.compute_depth_control(target_depth)
            self.publish_cmd(0.0, 0.0, z_cmd)
            
            # Progress update
            elapsed = time.time() - start_time
            if int(elapsed * 2) % 2 == 0:
                self.get_logger().info(
                    f'   Depth: {self.current_depth:.3f}m → Target: {target_depth:.3f}m',
                    throttle_duration_sec=0.4
                )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Hold at depth for 2 seconds
        self.hold_depth(2.0, target_depth)
        
        self.log_end(operation_name)
        self.get_logger().info(f'⏸️  Holding depth for {self.gap_duration:.0f} seconds...')
        self.hold_depth(self.gap_duration, target_depth)
    
    def execute_roll(self):
        """Execute roll motion at depth"""
        operation_name = "ROLL (Left/Right)"
        self.log_start(operation_name)
        
        target_depth = -0.8
        roll_speed = 0.4  # rad/s
        duration = self.motion_duration
        
        # Roll RIGHT
        self.get_logger().info('   Rolling RIGHT...')
        roll_start = time.time()
        while (time.time() - roll_start) < duration / 2:
            z_cmd = self.compute_depth_control(target_depth)
            self.publish_cmd(0.0, 0.0, z_cmd, roll_speed, 0.0, 0.0)
            
            elapsed = time.time() - roll_start
            if int(elapsed * 2) % 2 == 0:
                self.get_logger().info(
                    f'   Depth: {self.current_depth:.3f}m | Roll: {math.degrees(self.current_roll):+.1f}°',
                    throttle_duration_sec=0.4
                )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stabilize
        self.get_logger().info('   Stabilizing...')
        self.hold_depth(1.0, target_depth)
        
        # Roll LEFT
        self.get_logger().info('   Rolling LEFT...')
        roll_start = time.time()
        while (time.time() - roll_start) < duration / 2:
            z_cmd = self.compute_depth_control(target_depth)
            self.publish_cmd(0.0, 0.0, z_cmd, -roll_speed, 0.0, 0.0)
            
            elapsed = time.time() - roll_start
            if int(elapsed * 2) % 2 == 0:
                self.get_logger().info(
                    f'   Depth: {self.current_depth:.3f}m | Roll: {math.degrees(self.current_roll):+.1f}°',
                    throttle_duration_sec=0.4
                )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop
        self.stop()
        time.sleep(0.5)
        
        self.log_end(operation_name)
        self.get_logger().info(f'⏸️  Holding depth for {self.gap_duration:.0f} seconds...')
        self.hold_depth(self.gap_duration, target_depth)
    
    def execute_motion(self, operation_name: str, linear_x=0.0, linear_y=0.0,
                      angular_z=0.0, maintain_depth=None, duration=None):
        """Execute a motion with depth control"""
        if duration is None:
            duration = self.motion_duration
        
        self.log_start(operation_name)
        
        start_pos = self.current_position
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            if maintain_depth is not None:
                z_cmd = self.compute_depth_control(maintain_depth)
            else:
                z_cmd = 0.0
            
            self.publish_cmd(linear_x, linear_y, z_cmd, 0.0, 0.0, angular_z)
            
            # Progress update
            elapsed = time.time() - start_time
            if int(elapsed * 2) % 2 == 0:
                if maintain_depth is not None:
                    self.get_logger().info(
                        f'   Depth: {self.current_depth:.3f}m | Yaw: {math.degrees(self.current_yaw):+.1f}°',
                        throttle_duration_sec=0.4
                    )
                else:
                    dx = self.current_position[0] - start_pos[0]
                    dy = self.current_position[1] - start_pos[1]
                    self.get_logger().info(
                        f'   Moved: ΔX={dx:+.3f}m, ΔY={dy:+.3f}m, Depth: {self.current_depth:.3f}m',
                        throttle_duration_sec=0.4
                    )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop
        self.stop()
        time.sleep(0.5)
        
        self.log_end(operation_name)
        
        if maintain_depth is not None:
            self.get_logger().info(f'⏸️  Holding depth for {self.gap_duration:.0f} seconds...')
            self.hold_depth(self.gap_duration, maintain_depth)
    
    def run_demonstration(self):
        """Run complete demonstration sequence"""
        
        try:
            # 1. HEAVE DOWN to -0.8m
            self.heave_to_depth(-0.8, "HEAVE DOWN to -0.8m")
            
            # 2. ROLL at depth
            self.execute_roll()
            
            # 3. SURFACE to -0.2m
            self.heave_to_depth(-0.2, "SURFACE to -0.2m")
            
            # 4. SURGE FORWARD
            self.execute_motion(
                "SURGE FORWARD",
                linear_x=0.6,
                maintain_depth=-0.2
            )
            
            # 5. SURGE BACKWARD
            self.execute_motion(
                "SURGE BACKWARD",
                linear_x=-0.6,
                maintain_depth=-0.2
            )
            
            # 6. YAW LEFT
            self.execute_motion(
                "YAW LEFT (Counter-Clockwise)",
                angular_z=0.5,
                maintain_depth=-0.2
            )
            
            # 7. YAW RIGHT
            self.execute_motion(
                "YAW RIGHT (Clockwise)",
                angular_z=-0.5,
                maintain_depth=-0.2
            )
            
            # MISSION COMPLETE
            self.stop()
            
            self.get_logger().info('')
            self.get_logger().info('╔' + '='*68 + '╗')
            self.get_logger().info('║ 🎉 MISSION COMPLETE!                                            ║')
            self.get_logger().info('╚' + '='*68 + '╝')
            self.get_logger().info('')
            self.get_logger().info('✅ All operations completed successfully!')
            self.get_logger().info('')
            self.get_logger().info('Summary:')
            self.get_logger().info('  ✅ HEAVE DOWN to -0.8m')
            self.get_logger().info('  ✅ ROLL left/right')
            self.get_logger().info('  ✅ SURFACE to -0.2m')
            self.get_logger().info('  ✅ SURGE forward')
            self.get_logger().info('  ✅ SURGE backward')
            self.get_logger().info('  ✅ YAW left')
            self.get_logger().info('  ✅ YAW right')
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