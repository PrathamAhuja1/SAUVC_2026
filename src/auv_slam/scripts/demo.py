#!/usr/bin/env python3
"""
Simple Demo - Works with FIXED Thruster Mapper
Now uses CORRECT signs (no inversions needed)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class SimpleDemo(Node):
    def __init__(self):
        super().__init__('simple_demo')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # State
        self.current_depth = 0.0
        self.current_position = None
        self.current_yaw = 0.0
        self.odom_received = False
        self.movement_start_position = None
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Demo state
        self.demo_active = False
        self.current_movement = 0
        self.movement_start_time = 0.0
        
        # MOVEMENTS - Standard ROS Conventions
        self.movements = [
            # HEAVE: Standard ROS (negative Z = down, positive Z = up)
            {
                'name': 'HEAVE DOWN', 
                'vx': 0.0, 
                'vz': -0.4,  # Standard: Negative Z = descend
                'yaw': 0.0, 
                'duration': 10.0,
                'expected': 'Descend (negative ΔZ)'
            },
            {
                'name': 'HEAVE UP', 
                'vx': 0.0, 
                'vz': 0.4,   # Standard: Positive Z = ascend
                'yaw': 0.0, 
                'duration': 10.0,
                'expected': 'Ascend (positive ΔZ)'
            },
            
            # SURGE: Standard ROS convention
            {
                'name': 'SURGE FORWARD', 
                'vx': 0.5,   # CORRECT: Positive X = forward
                'vz': 0.0, 
                'yaw': 0.0, 
                'duration': 10.0,
                'expected': 'Move forward (positive ΔX)'
            },
            {
                'name': 'SURGE BACKWARD', 
                'vx': -0.5,  # CORRECT: Negative X = backward
                'vz': 0.0, 
                'yaw': 0.0, 
                'duration': 10.0,
                'expected': 'Move backward (negative ΔX)'
            },
            
            # YAW: Standard ROS convention
            {
                'name': 'YAW LEFT (CCW)', 
                'vx': 0.0, 
                'vz': 0.0, 
                'yaw': 0.4,  # CORRECT: Positive = CCW
                'duration': 10.0,
                'expected': 'Rotate left (counter-clockwise)'
            },
            {
                'name': 'YAW RIGHT (CW)', 
                'vx': 0.0, 
                'vz': 0.0, 
                'yaw': -0.4,  # CORRECT: Negative = CW
                'duration': 10.0,
                'expected': 'Rotate right (clockwise)'
            },
        ]
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 Simple Demo - Direct Thruster Control')
        self.get_logger().info('='*70)
        self.get_logger().info('   Using SIMPLE direct thruster mapper')
        self.get_logger().info('   Standard ROS conventions (no inversions)')
        self.get_logger().info('   Duration: 10 seconds per movement')
        self.get_logger().info('   Total time: ~60 seconds')
        self.get_logger().info('   Waiting for odometry...')
        self.get_logger().info('='*70)
    
    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('✅ Odometry received!')
            self.get_logger().info(f'   Starting position:')
            self.get_logger().info(f'     X = {msg.pose.pose.position.x:.3f}m')
            self.get_logger().info(f'     Y = {msg.pose.pose.position.y:.3f}m')
            self.get_logger().info(f'     Z = {msg.pose.pose.position.z:.3f}m')
            
            # Start demo after 3 seconds
            self.demo_start_timer = self.create_timer(3.0, self.start_demo)
        
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def start_demo(self):
        """Start the demo sequence"""
        self.demo_active = True
        self.current_movement = 0
        self.movement_start_time = time.time()
        self.movement_start_position = self.current_position
        
        current_move = self.movements[0]
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎬 DEMO STARTED')
        self.get_logger().info('='*70)
        self.get_logger().info(f"▶️  Movement 1/6: {current_move['name']}")
        self.get_logger().info(f"   Command: vx={current_move['vx']}, vz={current_move['vz']}, yaw={current_move['yaw']}")
        self.get_logger().info(f"   Expected: {current_move['expected']}")
        self.get_logger().info('='*70)
        
        # Cancel the start timer
        self.demo_start_timer.cancel()
    
    def control_loop(self):
        """Main control loop - runs at 20 Hz"""
        if not self.demo_active:
            return
        
        # Get current movement
        current = self.movements[self.current_movement]
        
        # Check if current movement is complete
        elapsed = time.time() - self.movement_start_time
        
        if elapsed >= current['duration']:
            # Log results before transitioning
            self.log_movement_result(current)
            
            # Movement complete, move to next
            self.current_movement += 1
            
            if self.current_movement >= len(self.movements):
                # All movements complete
                self.demo_complete()
                return
            
            # Start next movement
            self.movement_start_time = time.time()
            self.movement_start_position = self.current_position
            
            next_move = self.movements[self.current_movement]
            
            self.get_logger().info('='*70)
            self.get_logger().info(
                f"▶️  Movement {self.current_movement + 1}/6: {next_move['name']}"
            )
            self.get_logger().info(
                f"   Command: vx={next_move['vx']}, vz={next_move['vz']}, yaw={next_move['yaw']}"
            )
            self.get_logger().info(f"   Expected: {next_move['expected']}")
            self.get_logger().info('='*70)
        
        # Execute current movement
        cmd = Twist()
        cmd.linear.x = current['vx']
        cmd.linear.y = 0.0
        cmd.linear.z = current['vz']
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = current['yaw']
        
        self.cmd_vel_pub.publish(cmd)
        
        # Periodic status update
        if int(elapsed * 10) % 20 == 0:  # Every 2 seconds
            self.log_progress(current, elapsed)
    
    def log_progress(self, movement: dict, elapsed: float):
        """Log progress during movement"""
        if self.current_position and self.movement_start_position:
            dx = self.current_position[0] - self.movement_start_position[0]
            dy = self.current_position[1] - self.movement_start_position[1]
            dz = self.current_position[2] - self.movement_start_position[2]
            
            self.get_logger().info(
                f"⏱️  {movement['name']}: {elapsed:.1f}s | "
                f"ΔX={dx:+.3f}m, ΔY={dy:+.3f}m, ΔZ={dz:+.3f}m",
                throttle_duration_sec=1.9
            )
    
    def log_movement_result(self, movement: dict):
        """Log detailed results after movement completes"""
        if self.current_position and self.movement_start_position:
            dx = self.current_position[0] - self.movement_start_position[0]
            dy = self.current_position[1] - self.movement_start_position[1]
            dz = self.current_position[2] - self.movement_start_position[2]
            
            # Check if movement was correct
            success = self.verify_movement(movement, dx, dy, dz)
            status = "✅ CORRECT" if success else "❌ INCORRECT"
            
            self.get_logger().info('─'*70)
            self.get_logger().info(f"📊 RESULT: {movement['name']} - {status}")
            self.get_logger().info(f"   ΔX = {dx:+.3f}m")
            self.get_logger().info(f"   ΔY = {dy:+.3f}m")
            self.get_logger().info(f"   ΔZ = {dz:+.3f}m (depth change)")
            self.get_logger().info(f"   Expected: {movement['expected']}")
            self.get_logger().info('─'*70)
    
    def verify_movement(self, movement: dict, dx: float, dy: float, dz: float) -> bool:
        """Verify if movement was correct"""
        name = movement['name']
        threshold = 0.05  # 5cm threshold
        
        if 'DOWN' in name:
            return dz < -threshold  # Should descend (negative Z)
        elif 'UP' in name:
            return dz > threshold   # Should ascend (positive Z)
        elif 'FORWARD' in name:
            return dx > threshold   # Should move forward (positive X)
        elif 'BACKWARD' in name:
            return dx < -threshold  # Should move backward (negative X)
        elif 'LEFT' in name or 'RIGHT' in name:
            # Yaw should not cause significant depth change
            return abs(dz) < 0.2  # Less than 20cm depth change during yaw
        
        return False
    
    def demo_complete(self):
        """Demo finished"""
        self.demo_active = False
        
        # Stop all motion
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        if self.current_position:
            final_depth = self.current_position[2]
            final_x = self.current_position[0]
            final_y = self.current_position[1]
            
            self.get_logger().info('='*70)
            self.get_logger().info('✅ DEMO COMPLETE - ALL MOVEMENTS TESTED')
            self.get_logger().info('='*70)
            self.get_logger().info(f'   Final position:')
            self.get_logger().info(f'     X = {final_x:.3f}m')
            self.get_logger().info(f'     Y = {final_y:.3f}m')
            self.get_logger().info(f'     Z = {final_depth:.3f}m')
            self.get_logger().info(f'   Final yaw: {math.degrees(self.current_yaw):.1f}°')
            self.get_logger().info('='*70)
            self.get_logger().info('   Review the results above:')
            self.get_logger().info('   - All ✅ = Thruster system working correctly!')
            self.get_logger().info('   - Any ❌ = Further debugging needed')
            self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all motion
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()