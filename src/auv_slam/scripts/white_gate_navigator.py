#!/usr/bin/env python3
"""
FIXED White Gate Navigator
Proper depth control and thruster commands for orca4
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math


class WhiteGateNavigator(Node):
    def __init__(self):
        super().__init__('white_gate_navigator')
        
        self.bridge = CvBridge()
        
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CENTER_X = self.FRAME_WIDTH // 2
        self.CENTER_Y = self.FRAME_HEIGHT // 2
        
        # HSV range for white gate (bright white)
        self.LOWER_WHITE = np.array([0, 0, 200])
        self.UPPER_WHITE = np.array([180, 30, 255])
        
        # Detection thresholds
        self.MIN_DETECT_AREA = 1000
        self.SMALL_AREA = 15000
        self.MEDIUM_AREA = 40000
        self.LARGE_AREA = 80000
        
        # PWM constants (for direct thruster control)
        self.PWM_NEUTRAL = 1500
        self.PWM_MIN = 1100
        self.PWM_MAX = 1900
        
        # Target depth (negative is below surface)
        self.TARGET_DEPTH = -0.5
        
        # State machine
        self.SEARCHING = 0
        self.APPROACHING = 1
        self.ALIGNING = 2
        self.PASSING = 3
        self.COMPLETED = 4
        
        self.state = self.SEARCHING
        
        # Gate detection state
        self.gate_detected = False
        self.gate_cx = 0
        self.gate_cy = 0
        self.gate_area = 0
        self.mission_active = True
        self.frame_count = 0
        
        # Position tracking
        self.start_position = None
        self.current_position = None
        self.current_depth = 0.0
        self.passing_start_x = None
        
        # Subscriptions
        self.camera_sub = self.create_subscription(
            Image, '/front_left/image_raw', self.camera_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Publishers (T1-T6)
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(Float64, f'/thruster{i}_cmd', 10)
            self.thruster_pubs.append(pub)
        
        self.debug_image_pub = self.create_publisher(Image, '/autonomous/debug_image', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('✅ White Gate Navigator - FIXED VERSION')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Target Depth: {self.TARGET_DEPTH}m')
        self.get_logger().info('Waiting for camera and odometry...')
        self.get_logger().info('=' * 70)
    
    def odom_callback(self, msg):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(f'📍 Starting position: X={self.current_position[0]:.2f}m, '
                                  f'Y={self.current_position[1]:.2f}m, '
                                  f'Z={self.current_position[2]:.2f}m')
    
    def camera_callback(self, msg):
        if not self.mission_active:
            return
        
        self.frame_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Bridge error: {e}')
            return
        
        # Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create white mask
        white_mask = cv2.inRange(hsv_image, self.LOWER_WHITE, self.UPPER_WHITE)
        
        # Morphological operations
        kernel = np.ones((7, 7), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.gate_detected = False
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.MIN_DETECT_AREA:
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    self.gate_cx = int(M["m10"] / M["m00"])
                    self.gate_cy = int(M["m01"] / M["m00"])
                    self.gate_area = area
                    self.gate_detected = True
                    
                    # Visualization
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
                    cv2.circle(cv_image, (self.gate_cx, self.gate_cy), 15, (0, 0, 255), -1)
                    
                    cv2.putText(cv_image, f'Area: {int(area)}', (x, y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Draw center lines
        cv2.line(cv_image, (self.CENTER_X, 0), (self.CENTER_X, self.FRAME_HEIGHT), (255, 255, 0), 3)
        cv2.line(cv_image, (0, self.CENTER_Y), (self.FRAME_WIDTH, self.CENTER_Y), (255, 255, 0), 2)
        
        # State display
        state_names = ['SEARCHING', 'APPROACHING', 'ALIGNING', 'PASSING', 'COMPLETED']
        state_text = state_names[self.state]
        cv2.putText(cv_image, f'State: {state_text}', (10, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        if self.gate_detected:
            cv2.putText(cv_image, 'GATE DETECTED', (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Depth info
        cv2.putText(cv_image, f'Depth: {self.current_depth:.2f}m', (10, self.FRAME_HEIGHT - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except:
            pass
    
    def control_loop(self):
        if not self.mission_active or self.current_position is None:
            self.send_neutral()
            return
        
        # State machine
        if self.state == self.SEARCHING:
            self.searching_behavior()
        elif self.state == self.APPROACHING:
            self.approaching_behavior()
        elif self.state == self.ALIGNING:
            self.aligning_behavior()
        elif self.state == self.PASSING:
            self.passing_behavior()
        elif self.state == self.COMPLETED:
            self.completed_behavior()
    
    def searching_behavior(self):
        """Search for gate - move forward slowly while rotating"""
        if self.gate_detected:
            self.get_logger().info('🎯 Gate detected! Switching to APPROACHING')
            self.state = self.APPROACHING
            return
        
        # Depth control
        depth_cmd = self.get_depth_command()
        
        # Forward motion (slow)
        forward_speed = 0.3
        
        # Rotation for scanning
        rotation_speed = 0.2
        
        # T1, T2: Front thrusters (forward + rotation)
        # T3, T4: Back thrusters (forward - rotation) 
        # T5, T6: Vertical thrusters (depth)
        
        t1 = 1500 + int(forward_speed * 300) + int(rotation_speed * 200)
        t2 = 1500 + int(forward_speed * 300) - int(rotation_speed * 200)
        t3 = 1500 - int(forward_speed * 300) - int(rotation_speed * 200)
        t4 = 1500 - int(forward_speed * 300) + int(rotation_speed * 200)
        t5 = 1500 + depth_cmd
        t6 = 1500 - depth_cmd  # T6 has negative coefficient
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
        
        if self.frame_count % 40 == 0:
            self.get_logger().info(f'🔍 Searching... Depth: {self.current_depth:.2f}m')
    
    def approaching_behavior(self):
        """Approach gate with yaw corrections"""
        if not self.gate_detected:
            self.get_logger().warn('Lost gate! Returning to search')
            self.state = self.SEARCHING
            return
        
        if self.gate_area > self.MEDIUM_AREA:
            self.get_logger().info('📍 Close to gate - ALIGNING')
            self.state = self.ALIGNING
            return
        
        # Depth control
        depth_cmd = self.get_depth_command()
        
        # Forward speed based on distance
        if self.gate_area < self.SMALL_AREA:
            forward_speed = 0.6  # Far away
        else:
            forward_speed = 0.4  # Getting closer
        
        # Yaw correction
        horizontal_error = self.gate_cx - self.CENTER_X
        yaw_correction = horizontal_error / self.FRAME_WIDTH  # Normalized -0.5 to 0.5
        yaw_correction = max(-0.3, min(yaw_correction, 0.3))
        
        # Apply commands
        t1 = 1500 + int(forward_speed * 300) - int(yaw_correction * 300)
        t2 = 1500 + int(forward_speed * 300) + int(yaw_correction * 300)
        t3 = 1500 - int(forward_speed * 300) + int(yaw_correction * 300)
        t4 = 1500 - int(forward_speed * 300) - int(yaw_correction * 300)
        t5 = 1500 + depth_cmd
        t6 = 1500 - depth_cmd
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
        
        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'➡️ Approaching: Area={int(self.gate_area)}, Error={horizontal_error}px'
            )
    
    def aligning_behavior(self):
        """Fine alignment before passing"""
        if not self.gate_detected:
            self.state = self.SEARCHING
            return
        
        if self.gate_area > self.LARGE_AREA:
            self.get_logger().info('🚀 Very close - PASSING!')
            self.state = self.PASSING
            self.passing_start_x = self.current_position[0]
            return
        
        # Depth control
        depth_cmd = self.get_depth_command()
        
        horizontal_error = self.gate_cx - self.CENTER_X
        
        if abs(horizontal_error) < 100:
            # Well aligned - slow forward
            forward_speed = 0.3
            yaw_correction = horizontal_error / self.FRAME_WIDTH * 0.2
        else:
            # Need alignment - minimal forward, more rotation
            forward_speed = 0.1
            yaw_correction = horizontal_error / self.FRAME_WIDTH * 0.5
        
        t1 = 1500 + int(forward_speed * 300) - int(yaw_correction * 300)
        t2 = 1500 + int(forward_speed * 300) + int(yaw_correction * 300)
        t3 = 1500 - int(forward_speed * 300) + int(yaw_correction * 300)
        t4 = 1500 - int(forward_speed * 300) - int(yaw_correction * 300)
        t5 = 1500 + depth_cmd
        t6 = 1500 - depth_cmd
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
        
        if self.frame_count % 20 == 0:
            self.get_logger().info(f'🔄 Aligning: Error={horizontal_error}px')
    
    def passing_behavior(self):
        """Full speed through gate"""
        if self.passing_start_x is None:
            self.passing_start_x = self.current_position[0]
        
        distance_traveled = self.current_position[0] - self.passing_start_x
        
        if distance_traveled > 4.0:  # Passed through gate
            self.get_logger().info('✅ GATE PASSED! Mission complete!')
            self.state = self.COMPLETED
            self.mission_active = False
            return
        
        # Depth control
        depth_cmd = self.get_depth_command()
        
        # Maximum forward speed
        forward_speed = 1.0
        
        t1 = 1500 + int(forward_speed * 400)
        t2 = 1500 + int(forward_speed * 400)
        t3 = 1500 - int(forward_speed * 400)
        t4 = 1500 - int(forward_speed * 400)
        t5 = 1500 + depth_cmd
        t6 = 1500 - depth_cmd
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
        
        self.get_logger().info(
            f'🚀 PASSING: {distance_traveled:.2f}m / 4.0m',
            throttle_duration_sec=0.5
        )
    
    def completed_behavior(self):
        """Mission complete - stop"""
        self.send_neutral()
    
    def get_depth_command(self):
        """Calculate depth control command"""
        depth_error = self.TARGET_DEPTH - self.current_depth
        
        if abs(depth_error) < 0.15:
            return 0
        
        # PWM change based on error
        pwm_change = int(abs(depth_error) * 400)
        pwm_change = min(pwm_change, 300)
        
        if depth_error < 0:  # Need to go down
            return -pwm_change
        else:  # Need to go up
            return pwm_change
    
    def send_thrusters(self, t1, t2, t3, t4, t5, t6):
        """Send commands to all thrusters"""
        # Clamp values
        t1 = max(self.PWM_MIN, min(t1, self.PWM_MAX))
        t2 = max(self.PWM_MIN, min(t2, self.PWM_MAX))
        t3 = max(self.PWM_MIN, min(t3, self.PWM_MAX))
        t4 = max(self.PWM_MIN, min(t4, self.PWM_MAX))
        t5 = max(self.PWM_MIN, min(t5, self.PWM_MAX))
        t6 = max(self.PWM_MIN, min(t6, self.PWM_MAX))
        
        thrusters = [t1, t2, t3, t4, t5, t6]
        
        for i, pwm in enumerate(thrusters):
            msg = Float64()
            msg.data = float(pwm)
            self.thruster_pubs[i].publish(msg)
    
    def send_neutral(self):
        """Stop all thrusters"""
        self.send_thrusters(1500, 1500, 1500, 1500, 1500, 1500)


def main(args=None):
    rclpy.init(args=args)
    node = WhiteGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_neutral()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()