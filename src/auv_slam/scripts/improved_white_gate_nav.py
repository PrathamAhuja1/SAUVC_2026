#!/usr/bin/env python3
"""
FIXED Autonomous Navigation for White Gate
Corrected camera topic and enhanced detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math


class FixedWhiteGateNavigator(Node):
    def __init__(self):
        super().__init__('white_gate_navigator')
        
        self.bridge = CvBridge()
        
        # Vision settings
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CENTER_X = self.FRAME_WIDTH // 2
        self.CENTER_Y = self.FRAME_HEIGHT // 2
        
        # Enhanced white detection - covers white and off-white
        self.LOWER_WHITE = np.array([0, 0, 180])    # Lower threshold for off-white
        self.UPPER_WHITE = np.array([180, 40, 255]) # Allow slight color tint
        
        # Detection parameters
        self.MIN_DETECT_AREA = 800   # Reduced for farther gate
        self.APPROACH_AREA = 60000   # When gate fills ~20% of frame
        
        # Control gains (smooth and responsive)
        self.K_SWAY = 0.010      # Horizontal alignment
        self.K_HEAVE = 0.008     # Vertical alignment
        self.K_SURGE = 0.4       # Forward speed
        
        # State machine
        self.SEARCHING = 0
        self.APPROACHING = 1
        self.ALIGNING = 2
        self.PASSING = 3
        self.COMPLETED = 4
        
        self.state = self.SEARCHING
        self.state_start_time = None
        
        # State variables
        self.gate_detected = False
        self.gate_cx = 0
        self.gate_cy = 0
        self.gate_area = 0
        self.mission_active = True
        self.frame_count = 0
        
        # Position tracking
        self.start_position = None
        self.current_position = None
        self.passing_start_x = None
        
        # FIXED: Correct camera topic from URDF
        self.camera_sub = self.create_subscription(
            Image,
            '/front_left/image_raw',  # Changed from /orca4_ign/front_left/image_raw
            self.camera_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/autonomous/debug_image', 10)
        
        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🤖 FIXED WHITE GATE AUTONOMOUS NAVIGATOR')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Camera Topic: /front_left/image_raw')
        self.get_logger().info(f'Frame Size: {self.FRAME_WIDTH}x{self.FRAME_HEIGHT}')
        self.get_logger().info(f'Detection: Enhanced white/off-white HSV filtering')
        self.get_logger().info('='*70)
    
    def odom_callback(self, msg):
        """Track robot position"""
        self.current_position = msg.pose.pose.position
        
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(
                f'📍 Start Position: X={self.current_position.x:.2f}, '
                f'Y={self.current_position.y:.2f}, Z={self.current_position.z:.2f}'
            )
    
    def detect_white_gate(self, cv_image):
        """
        Enhanced white gate detection
        Returns: (detected, center_x, center_y, area, debug_image)
        """
        self.frame_count += 1
        
        # Strong Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(cv_image, (15, 15), 0)
        
        # Convert to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Create white mask with enhanced parameters
        mask = cv2.inRange(hsv, self.LOWER_WHITE, self.UPPER_WHITE)
        
        # Aggressive morphological operations to clean up
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
        mask = cv2.dilate(mask, kernel_large, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Debug visualization
        debug_img = cv_image.copy()
        
        # Show mask overlay
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        debug_img = cv2.addWeighted(debug_img, 0.6, mask_colored, 0.4, 0)
        
        # Draw crosshair
        cv2.line(debug_img, (self.CENTER_X, 0), (self.CENTER_X, self.FRAME_HEIGHT), 
                (0, 255, 255), 2)
        cv2.line(debug_img, (0, self.CENTER_Y), (self.FRAME_WIDTH, self.CENTER_Y), 
                (0, 255, 255), 2)
        
        # Status text
        cv2.putText(debug_img, f'Frame: {self.frame_count}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if not contours:
            cv2.putText(debug_img, 'NO GATE DETECTED', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(debug_img, 'Searching...', (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
            return False, 0, 0, 0, debug_img
        
        # Find largest white contour (the gate)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.MIN_DETECT_AREA:
            cv2.putText(debug_img, f'OBJECT TOO SMALL: {int(area)}px', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
            # Draw all contours in gray
            cv2.drawContours(debug_img, contours, -1, (128, 128, 128), 2)
            return False, 0, 0, 0, debug_img
        
        # Calculate center
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return False, 0, 0, 0, debug_img
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Draw detection (thick green)
        cv2.drawContours(debug_img, [largest_contour], -1, (0, 255, 0), 4)
        cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(debug_img, (cx, cy), 15, (0, 0, 255), -1)
        cv2.circle(debug_img, (cx, cy), 20, (255, 255, 255), 3)
        cv2.line(debug_img, (self.CENTER_X, self.CENTER_Y), (cx, cy), (255, 0, 255), 3)
        
        # Annotations
        cv2.putText(debug_img, '🎯 GATE DETECTED', (x, y - 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        cv2.putText(debug_img, f'Area: {int(area)} px²', (x, y - 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(debug_img, f'Size: {w}x{h}', (x, y - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Error display
        error_x = cx - self.CENTER_X
        error_y = cy - self.CENTER_Y
        cv2.putText(debug_img, f'Error X: {error_x:+d} px', (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(debug_img, f'Error Y: {error_y:+d} px', (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Distance indicator
        if area < 10000:
            dist_text = "VERY FAR"
            color = (0, 0, 255)
        elif area < 20000:
            dist_text = "FAR"
            color = (0, 100, 255)
        elif area < 40000:
            dist_text = "MEDIUM"
            color = (0, 165, 255)
        elif area < self.APPROACH_AREA:
            dist_text = "CLOSE"
            color = (0, 255, 255)
        else:
            dist_text = "VERY CLOSE!"
            color = (0, 255, 0)
        
        cv2.putText(debug_img, f'Distance: {dist_text}', (10, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        
        # Progress bar for area
        max_bar_width = 400
        bar_width = int((area / 100000) * max_bar_width)
        bar_width = min(bar_width, max_bar_width)
        cv2.rectangle(debug_img, (10, 170), (10 + bar_width, 190), color, -1)
        cv2.rectangle(debug_img, (10, 170), (10 + max_bar_width, 190), (255, 255, 255), 2)
        
        return True, cx, cy, area, debug_img
    
    def camera_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect gate
            detected, cx, cy, area, debug_img = self.detect_white_gate(cv_image)
            
            # Update state
            self.gate_detected = detected
            if detected:
                self.gate_cx = cx
                self.gate_cy = cy
                self.gate_area = area
            
            # Add state info to debug image
            state_names = ['SEARCHING', 'APPROACHING', 'ALIGNING', 'PASSING', 'COMPLETED']
            state_color = (255, 255, 0) if detected else (128, 128, 128)
            cv2.putText(debug_img, f'State: {state_names[self.state]}', (10, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, state_color, 2)
            
            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Debug image publish error: {e}')
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
    
    def control_loop(self):
        """Main control state machine"""
        
        if not self.mission_active:
            self.stop()
            return
        
        cmd = Twist()
        
        if self.state == self.SEARCHING:
            cmd = self.searching_behavior(cmd)
        elif self.state == self.APPROACHING:
            cmd = self.approaching_behavior(cmd)
        elif self.state == self.ALIGNING:
            cmd = self.aligning_behavior(cmd)
        elif self.state == self.PASSING:
            cmd = self.passing_behavior(cmd)
        elif self.state == self.COMPLETED:
            cmd = self.completed_behavior(cmd)
        
        self.cmd_vel_pub.publish(cmd)
    
    def searching_behavior(self, cmd):
        """Search for gate with slow forward movement and rotation"""
        if self.gate_detected:
            self.get_logger().info('🎯 Gate detected! Transitioning to APPROACHING')
            self.transition_to(self.APPROACHING)
            return cmd
        
        # Search pattern: slow forward + gentle rotation
        cmd.linear.x = 0.3
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.2  # Gentle rotation to scan area
        
        return cmd
    
    def approaching_behavior(self, cmd):
        """Approach gate with alignment"""
        if not self.gate_detected:
            self.get_logger().warn('⚠️ Gate lost! Returning to SEARCHING')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Check if close enough to align
        if self.gate_area > 40000:
            self.get_logger().info('📍 Close enough! Transitioning to ALIGNING')
            self.transition_to(self.ALIGNING)
            return cmd
        
        # Calculate alignment errors
        error_x = self.gate_cx - self.CENTER_X
        error_y = self.gate_cy - self.CENTER_Y
        
        # Control commands with smooth proportional control
        cmd.linear.x = self.K_SURGE
        cmd.linear.y = error_x * self.K_SWAY
        cmd.linear.z = -error_y * self.K_HEAVE
        cmd.angular.z = 0.0
        
        return cmd
    
    def aligning_behavior(self, cmd):
        """Precise alignment before passing"""
        if not self.gate_detected:
            self.get_logger().warn('⚠️ Gate lost during alignment!')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Calculate errors
        error_x = self.gate_cx - self.CENTER_X
        error_y = self.gate_cy - self.CENTER_Y
        
        # Check alignment quality
        if abs(error_x) < 80 and abs(error_y) < 80 and self.gate_area > self.APPROACH_AREA:
            self.get_logger().info('✅ Well aligned! Transitioning to PASSING')
            self.passing_start_x = self.current_position.x if self.current_position else 0
            self.transition_to(self.PASSING)
            return cmd
        
        # Precise alignment
        cmd.linear.x = 0.2 if self.gate_area < self.APPROACH_AREA else 0.0
        cmd.linear.y = error_x * self.K_SWAY * 1.8
        cmd.linear.z = -error_y * self.K_HEAVE * 1.8
        cmd.angular.z = 0.0
        
        return cmd
    
    def passing_behavior(self, cmd):
        """Pass through gate at good speed"""
        
        # Check if passed through (traveled 3 meters)
        if self.current_position and self.passing_start_x is not None:
            distance_traveled = self.current_position.x - self.passing_start_x
            
            if distance_traveled > 3.0:
                self.get_logger().info('🎉 Gate passed successfully!')
                self.transition_to(self.COMPLETED)
                return cmd
        
        # Full speed forward with minimal corrections
        cmd.linear.x = 1.0
        
        # Light corrections if gate still visible
        if self.gate_detected:
            error_x = self.gate_cx - self.CENTER_X
            error_y = self.gate_cy - self.CENTER_Y
            
            if abs(error_x) > 150:
                cmd.linear.y = error_x * self.K_SWAY * 0.3
            if abs(error_y) > 150:
                cmd.linear.z = -error_y * self.K_HEAVE * 0.3
        
        return cmd
    
    def completed_behavior(self, cmd):
        """Mission completed - stop"""
        self.mission_active = False
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        
        self.get_logger().info('='*70)
        self.get_logger().info('🏆 MISSION COMPLETE!')
        self.get_logger().info('='*70)
        
        return cmd
    
    def transition_to(self, new_state):
        """Transition to new state"""
        self.state = new_state
        self.state_start_time = self.get_clock().now()
    
    def stop(self):
        """Emergency stop"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FixedWhiteGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down gracefully...')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()