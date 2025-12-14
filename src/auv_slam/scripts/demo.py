#!/usr/bin/env python3
"""
Simple 3-Movement Demo - FIXED THRUSTER SIGNS
1. Heave down 10s, then up 10s
2. Surge forward 10s, then backward 10s  
3. Yaw left 10s, then right 10s

CRITICAL FIX: Vertical thrusters point DOWN
- Positive Z command = thrusters push water down = AUV goes UP
- Negative Z command = thrusters suck water up = AUV goes DOWN
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
        
        # Control timer (20 Hz like qualification)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Demo state
        self.demo_active = False
        self.current_movement = 0
        self.movement_start_time = 0.0
        
        # MOVEMENTS - 10 seconds each, CORRECTED SIGNS
        self.movements = [
            # HEAVE: Negative Z = DOWN, Positive Z = UP
            {'name': 'HEAVE DOWN', 'vx': 0.0, 'vz': -0.4, 'yaw': 0.0, 'duration': 10.0},
            {'name': 'HEAVE UP', 'vx': 0.0, 'vz': 0.4, 'yaw': 0.0, 'duration': 10.0},
            
            # SURGE: Positive X = FORWARD, Negative X = BACKWARD
            {'name': 'SURGE FORWARD', 'vx': 0.5, 'vz': 0.0, 'yaw': 0.0, 'duration': 10.0},
            {'name': 'SURGE BACKWARD', 'vx': -0.5, 'vz': 0.0, 'yaw': 0.0, 'duration': 10.0},
            
            # YAW: Positive = CCW (LEFT), Negative = CW (RIGHT)
            {'name': 'YAW LEFT', 'vx': 0.0, 'vz': 0.0, 'yaw': 0.4, 'duration': 10.0},
            {'name': 'YAW RIGHT', 'vx': 0.0, 'vz': 0.0, 'yaw': -0.4, 'duration': 10.0},
        ]
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 Simple Demo Node Started')
        self.get_logger().info('   Duration: 10 seconds per movement')
        self.get_logger().info('   Total time: ~60 seconds')
        self.get_logger().info('   Waiting for odometry...')
        self.get_logger().info('='*70)
    
    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('✅ Odometry received!')
            self.get_logger().info(f'   Starting depth: Z={msg.pose.pose.position.z:.2f}m')
            
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
        
        self.get_logger().info('='*70)
        self.get_logger().info('🎬 DEMO STARTED')
        self.get_logger().info('='*70)
        self.get_logger().info(f"▶️  Movement 1/6: {self.movements[0]['name']}")
        
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
            # Movement complete, move to next
            self.current_movement += 1
            
            if self.current_movement >= len(self.movements):
                # All movements complete
                self.demo_complete()
                return
            
            # Start next movement
            self.movement_start_time = time.time()
            self.get_logger().info(
                f"▶️  Movement {self.current_movement + 1}/6: "
                f"{self.movements[self.current_movement]['name']}"
            )
        
        # Execute current movement
        cmd = Twist()
        cmd.linear.x = current['vx']
        cmd.linear.y = 0.0
        cmd.linear.z = current['vz']
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = current['yaw']
        
        self.cmd_vel_pub.publish(cmd)
    
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
            self.get_logger().info('✅ DEMO COMPLETE')
            self.get_logger().info('='*70)
            self.get_logger().info(f'   Final position:')
            self.get_logger().info(f'     X = {final_x:.2f}m')
            self.get_logger().info(f'     Y = {final_y:.2f}m')
            self.get_logger().info(f'     Z = {final_depth:.2f}m')
            self.get_logger().info(f'   Final yaw: {math.degrees(self.current_yaw):.1f}°')
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