#!/usr/bin/env python3
"""
QUALIFICATION NAVIGATOR - FIXED U-TURN AND PROPER GATE ALIGNMENT
Strategy:
1. Align visually at 2.5m
2. Lock Compass Heading
3. Drive Blindly through gate
4. Proper U-turn with forward motion
5. Reverse approach with center alignment
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class QualificationNavigator(Node):
    def __init__(self):
        super().__init__('qualification_navigator')
        
        # State machine
        self.SUBMERGING = 0
        self.SEARCHING = 1
        self.APPROACHING = 2
        self.ALIGNING = 3
        self.PASSING_HEADING_LOCK = 5
        self.CLEARING = 6
        self.UTURN = 7
        self.POST_UTURN_ALIGN = 8
        self.REVERSE_APPROACHING = 10
        self.REVERSE_ALIGNING = 12
        self.REVERSE_PASSING_LOCK = 13
        self.REVERSE_CLEARING = 14
        self.COMPLETED = 15
        
        self.state = self.SUBMERGING
        self.gate_x_position = 0.0
        
        # Clearance Parameters (SAUVC Rules)
        self.auv_length = 0.46
        self.clearance_margin = 0.60 
        
        self.forward_clearance_x = self.gate_x_position + self.auv_length + self.clearance_margin
        self.reverse_clearance_x = self.gate_x_position - self.auv_length - self.clearance_margin
        
        # Params
        self.declare_parameter('mission_depth', -0.8)
        self.declare_parameter('passing_trigger_dist', 1.8)
        self.declare_parameter('passing_speed', 0.8)
        self.declare_parameter('uturn_speed', 0.5)  # INCREASED for proper U-turn
        self.declare_parameter('uturn_depth_bias', -0.4)
        
        self.mission_depth = self.get_parameter('mission_depth').value
        self.trigger_dist = self.get_parameter('passing_trigger_dist').value
        self.passing_speed = self.get_parameter('passing_speed').value
        self.uturn_speed = self.get_parameter('uturn_speed').value
        self.uturn_depth_bias = self.get_parameter('uturn_depth_bias').value
        
        # Variables
        self.gate_detected = False
        self.alignment_error = 0.0
        self.estimated_distance = 999.0
        self.frame_position = 0.0
        self.current_position = None
        self.current_yaw = 0.0
        self.current_depth = 0.0
        self.locked_yaw = 0.0 
        self.reverse_mode = False
        self.uturn_start_time = 0.0
        self.uturn_start_yaw = 0.0
        self.state_start_time = time.time()
        
        # Subscriptions
        self.create_subscription(Bool, '/qualification/gate_detected', self.gate_cb, 10)
        self.create_subscription(Float32, '/qualification/alignment_error', self.align_cb, 10)
        self.create_subscription(Float32, '/qualification/estimated_distance', self.dist_cb, 10)
        self.create_subscription(Float32, '/qualification/frame_position', self.frame_pos_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_cb, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/qualification/state', 10)
        self.reverse_mode_pub = self.create_publisher(Bool, '/mission/reverse_mode', 10)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('🚀 Qualification Navigator Started')
        
    def gate_cb(self, msg): 
        self.gate_detected = msg.data
    
    def align_cb(self, msg): 
        self.alignment_error = msg.data
    
    def dist_cb(self, msg): 
        self.estimated_distance = msg.data
    
    def frame_pos_cb(self, msg):
        self.frame_position = msg.data
    
    def odom_cb(self, msg):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        cmd = Twist()
        
        # Depth Control (Except during U-turn)
        if self.state != self.UTURN:
            cmd.linear.z = self.depth_control(self.mission_depth)
            
        # --- STATE MACHINE ---
        if self.state == self.SUBMERGING:
            if abs(self.current_depth - self.mission_depth) < 0.2:
                self.transition_to(self.SEARCHING)

        elif self.state == self.SEARCHING:
            if self.gate_detected and self.estimated_distance < 10.0:
                self.transition_to(self.APPROACHING)
            cmd.linear.x = 0.4
            cmd.angular.z = 0.2 * (1 if (time.time() % 6 < 3) else -1)

        elif self.state == self.APPROACHING:
            if self.estimated_distance < 3.0:
                self.transition_to(self.ALIGNING)
            cmd.linear.x = 0.5
            cmd.angular.z = -self.alignment_error * 1.5

        elif self.state == self.ALIGNING:
            # If close enough and aligned, LOCK HEADING and PASS
            if self.estimated_distance < self.trigger_dist and abs(self.frame_position) < 0.15:
                self.locked_yaw = self.current_yaw
                self.transition_to(self.PASSING_HEADING_LOCK)
                return
            
            # Visual alignment
            cmd.linear.x = 0.15
            cmd.angular.z = -self.frame_position * 2.5

        elif self.state == self.PASSING_HEADING_LOCK:
            # Check if we passed the clearance line
            if self.current_position[0] > self.forward_clearance_x:
                self.transition_to(self.UTURN)
                return

            # Maintain Locked Heading
            yaw_error = self.normalize_angle(self.locked_yaw - self.current_yaw)
            cmd.angular.z = yaw_error * 2.0
            cmd.linear.x = self.passing_speed

        elif self.state == self.UTURN:
            cmd = self.perform_uturn(cmd)

        elif self.state == self.POST_UTURN_ALIGN:
            self.reverse_mode = True
            self.reverse_mode_pub.publish(Bool(data=True))
            if self.gate_detected:
                self.transition_to(self.REVERSE_APPROACHING)
            cmd.angular.z = 0.3

        elif self.state == self.REVERSE_APPROACHING:
            if self.estimated_distance < 3.0:
                self.transition_to(self.REVERSE_ALIGNING)
            cmd.linear.x = 0.4
            cmd.angular.z = -self.frame_position * 1.5

        elif self.state == self.REVERSE_ALIGNING:
            # Align properly before locking heading
            if self.estimated_distance < self.trigger_dist and abs(self.frame_position) < 0.15:
                self.locked_yaw = self.current_yaw
                self.transition_to(self.REVERSE_PASSING_LOCK)
                return
            
            # Strong alignment correction
            cmd.linear.x = 0.15
            cmd.angular.z = -self.frame_position * 2.5

        elif self.state == self.REVERSE_PASSING_LOCK:
            # Check reverse clearance
            if self.current_position[0] < self.reverse_clearance_x:
                self.transition_to(self.COMPLETED)
                return

            # Maintain Locked Heading
            yaw_error = self.normalize_angle(self.locked_yaw - self.current_yaw)
            cmd.angular.z = yaw_error * 2.0
            cmd.linear.x = self.passing_speed

        elif self.state == self.COMPLETED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)
        self.state_pub.publish(String(data=self.get_state_name()))

    def perform_uturn(self, cmd):
        """FIXED: Proper U-turn with forward motion"""
        if self.uturn_start_time == 0:
            self.uturn_start_time = time.time()
            self.uturn_start_yaw = self.current_yaw
            
        angle_turned = abs(self.normalize_angle(self.current_yaw - self.uturn_start_yaw))
        
        if angle_turned > (math.pi - 0.2):
            self.transition_to(self.POST_UTURN_ALIGN)
            self.uturn_start_time = 0
            return cmd
        
        # FIXED: Move forward while turning for proper U-turn
        cmd.linear.x = self.uturn_speed  # Now 0.5 m/s
        cmd.angular.z = 0.4  # Increased turn rate
        
        # Maintain depth during U-turn
        err = self.mission_depth - self.current_depth
        cmd.linear.z = (err * 2.0) + self.uturn_depth_bias
        
        return cmd

    def depth_control(self, target):
        err = target - self.current_depth
        return max(-0.5, min(err * 1.0, 0.5))

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    def transition_to(self, new_state):
        """State transition with logging only at start and end"""
        old_state = self.get_state_name()
        elapsed = time.time() - self.state_start_time
        
        # Log state completion
        self.get_logger().info(f'✓ {old_state} completed ({elapsed:.1f}s)')
        
        # Update state
        self.state = new_state
        self.state_start_time = time.time()
        
        # Log new state start
        new_state_name = self.get_state_name()
        self.get_logger().info(f'→ {new_state_name} started')

    def get_state_name(self):
        names = {
            self.SUBMERGING: 'SUBMERGING',
            self.SEARCHING: 'SEARCHING',
            self.APPROACHING: 'APPROACHING',
            self.ALIGNING: 'ALIGNING',
            self.PASSING_HEADING_LOCK: 'PASSING',
            self.CLEARING: 'CLEARING',
            self.UTURN: 'U-TURN',
            self.POST_UTURN_ALIGN: 'POST_UTURN_ALIGN',
            self.REVERSE_APPROACHING: 'REVERSE_APPROACHING',
            self.REVERSE_ALIGNING: 'REVERSE_ALIGNING',
            self.REVERSE_PASSING_LOCK: 'REVERSE_PASSING',
            self.REVERSE_CLEARING: 'REVERSE_CLEARING',
            self.COMPLETED: 'COMPLETED'
        }
        return names.get(self.state, 'UNKNOWN')


def main(args=None):
    rclpy.init(args=args)
    node = QualificationNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Navigator shutdown')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()