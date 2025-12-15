#!/usr/bin/env python3
"""
SIMPLE Direct Thruster Mapper
Direct control without complex matrix math - just map commands to thrusters
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class SimpleThrusterMapper(Node):
    def __init__(self):
        super().__init__('simple_thruster_mapper')
        
        self.declare_parameter('max_thrust', 10.0)
        self.max_thrust = self.get_parameter('max_thrust').value
        
        # Subscriptions
        self.twist_sub = self.create_subscription(
            Twist, '/rp2040/cmd_vel', self.twist_callback, 10)
        
        # Publishers for each thruster
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(Float64, f'/thruster{i}_cmd', 10)
            self.thruster_pubs.append(pub)
        
        self.last_cmd = None
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ SIMPLE Direct Thruster Mapper')
        self.get_logger().info('='*70)
        self.get_logger().info('   Direct command mapping (no matrix math)')
        self.get_logger().info(f'   Max thrust: {self.max_thrust} N')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):
        """
        Direct mapping of Twist commands to thrusters
        
        BlueROV2 Configuration:
              FRONT
           T1      T2
             \  /
              \/
              /\
             /  \
           T3      T4
              BACK
        
        T5 (front vertical), T6 (back vertical) - point DOWN
        
        Physics:
        - Positive thrust on vertical = push water DOWN = AUV goes UP
        - Negative thrust on vertical = pull water UP = AUV goes DOWN
        """
        self.last_cmd = msg
        
        # Extract commands
        surge = msg.linear.x    # Forward/backward
        sway = msg.linear.y     # Left/right  
        heave = msg.linear.z    # Up/down
        yaw = msg.angular.z     # Rotation
        
        # ==================================================================
        # HORIZONTAL THRUSTERS (T1-T4) - 45° vectored configuration
        # ==================================================================
        cos45 = 0.7071
        
        # T1: Front-Left (contributes to: surge, sway, yaw)
        t1 = cos45 * (surge + sway) - yaw * 2.0
        
        # T2: Front-Right (contributes to: surge, -sway, -yaw)
        t2 = cos45 * (surge - sway) + yaw * 2.0
        
        # T3: Back-Left (contributes to: -surge, sway, -yaw)
        t3 = cos45 * (-surge + sway) + yaw * 2.0
        
        # T4: Back-Right (contributes to: -surge, -sway, yaw)
        t4 = cos45 * (-surge - sway) - yaw * 2.0
        
        # ==================================================================
        # VERTICAL THRUSTERS (T5-T6) - Point DOWN
        # ==================================================================
        # CRITICAL FIX: Based on test results, negative thrust = AUV goes UP
        # So to go DOWN (heave < 0), we need POSITIVE thrust
        # To go UP (heave > 0), we need NEGATIVE thrust
        # Therefore: thrust = -heave
        
        t5 = -heave * 2.5  # Front vertical (stronger for better control)
        t6 = -heave * 2.5  # Back vertical
        
        # Apply limits and publish
        thrusts = [t1, t2, t3, t4, t5, t6]
        
        for i, thrust in enumerate(thrusts):
            # Clamp to max thrust
            thrust = max(-self.max_thrust, min(thrust, self.max_thrust))
            
            # Publish
            msg = Float64()
            msg.data = float(thrust)
            self.thruster_pubs[i].publish(msg)
    
    def publish_diagnostics(self):
        """Print diagnostic information"""
        if self.last_cmd:
            cmd = self.last_cmd
            self.get_logger().info(
                f'Cmd: vx={cmd.linear.x:.2f}, vy={cmd.linear.y:.2f}, '
                f'vz={cmd.linear.z:.2f}, yaw={cmd.angular.z:.2f}',
                throttle_duration_sec=0.9
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleThrusterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()