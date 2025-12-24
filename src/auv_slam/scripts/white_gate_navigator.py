#!/usr/bin/env python3
"""
CORRECTED White Gate Navigator - Proper Thruster Configuration
Accounts for negative thrust coefficients on T3, T4, T6
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


class WhiteGateNavigatorFixed(Node):
    def __init__(self):
        super().__init__('white_gate_navigator')
        
        self.bridge = CvBridge()
        
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CENTER_X = self.FRAME_WIDTH // 2
        self.CENTER_Y = self.FRAME_HEIGHT // 2
        
        # HSV range for white gate (bright white)
        self.LOWER_WHITE = np.array([0, 0, 180])
        self.UPPER_WHITE = np.array([180, 40, 255])
        
        # Detection thresholds
        self.MIN_DETECT_AREA = 800
        self.SMALL_AREA = 10000
        self.MEDIUM_AREA = 30000
        self.LARGE_AREA = 60000
        
        # PWM constants
        self.PWM_NEUTRAL = 1500
        self.PWM_MIN = 1300
        self.PWM_MAX = 1800
        
        # Target depth
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
        
        # Publishers (in URDF order: T1, T2, T3, T4, T5, T6)
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(Float64, f'/thruster{i}_cmd', 10)
            self.thruster_pubs.append(pub)
        
        self.debug_image_pub = self.create_publisher(Image, '/autonomous/debug_image', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('White Gate Navigator - CORRECTED Thruster Configuration')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Target Depth: {self.TARGET_DEPTH}m')
        self.get_logger().info('Thruster Mapping:')
        self.get_logger().info('  T1: Front-Right (+0.02)')
        self.get_logger().info('  T2: Front-Left (+0.02)')
        self.get_logger().info('  T3: Back-Right (-0.02) [NEGATIVE]')
        self.get_logger().info('  T4: Back-Left (-0.02) [NEGATIVE]')
        self.get_logger().info('  T5: Vertical-Right (+0.02)')
        self.get_logger().info('  T6: Vertical-Left (-0.02) [NEGATIVE]')
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
    
    def camera_callback(self, msg):
        if not self.mission_active:
            return
        
        self.frame_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return
        
        # Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Create white mask
        white_mask = cv2.inRange(hsv_image, self.LOWER_WHITE, self.UPPER_WHITE)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
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
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(cv_image, (self.gate_cx, self.gate_cy), 10, (0, 0, 255), -1)
                    
                    cv2.putText(cv_image, f'Area: {int(area)}', (x, y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw center lines
        cv2.line(cv_image, (self.CENTER_X, 0), (self.CENTER_X, self.FRAME_HEIGHT), (255, 0, 0), 2)
        cv2.line(cv_image, (0, self.CENTER_Y), (self.FRAME_WIDTH, self.CENTER_Y), (255, 0, 0), 2)
        
        # State display
        state_text = ['SEARCHING', 'APPROACHING', 'ALIGNING', 'PASSING', 'COMPLETED'][self.state]
        cv2.putText(cv_image, f'State: {state_text}', (10, 30),
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
        
        # Depth control
        depth_error = self.TARGET_DEPTH - self.current_depth
        depth_pwm = self.depth_control(depth_error)
        
        # State machine
        if self.state == self.SEARCHING:
            self.searching_behavior(depth_pwm)
        elif self.state == self.APPROACHING:
            self.approaching_behavior(depth_pwm)
        elif self.state == self.ALIGNING:
            self.aligning_behavior(depth_pwm)
        elif self.state == self.PASSING:
            self.passing_behavior(depth_pwm)
        elif self.state == self.COMPLETED:
            self.completed_behavior()
    
    def depth_control(self, error):
        """
        Depth control accounting for T5 (+0.02) and T6 (-0.02)
        Returns tuple: (t5_pwm, t6_pwm)
        """
        if abs(error) < 0.1:
            return (1500, 1500)
        
        # To go DOWN (negative error):
        #   T5 (+coeff): need LOW PWM
        #   T6 (-coeff): need HIGH PWM
        # To go UP (positive error):
        #   T5 (+coeff): need HIGH PWM
        #   T6 (-coeff): need LOW PWM
        
        pwm_change = int(abs(error) * 300)
        pwm_change = min(pwm_change, 200)
        
        if error < 0:  # Go DOWN
            t5_pwm = 1500 - pwm_change  # Lower for T5
            t6_pwm = 1500 + pwm_change  # Higher for T6
        else:  # Go UP
            t5_pwm = 1500 + pwm_change  # Higher for T5
            t6_pwm = 1500 - pwm_change  # Lower for T6
        
        return (t5_pwm, t6_pwm)
    
    def searching_behavior(self, depth_pwm):
        """Search pattern with proper thruster control"""
        if self.gate_detected:
            self.get_logger().info('🎯 Gate detected - switching to APPROACHING')
            self.state = self.APPROACHING
            return
        
        # Slow forward motion with rotation
        # Forward: T1, T2 high; T3, T4 low (because T3, T4 have negative coeffs)
        t1 = 1550  # Front-Right
        t2 = 1550  # Front-Left
        t3 = 1450  # Back-Right (negative coeff)
        t4 = 1450  # Back-Left (negative coeff)
        
        # Add gentle rotation for scanning
        rotation_pwm = 30
        t1 += rotation_pwm
        t3 += rotation_pwm
        t2 -= rotation_pwm
        t4 -= rotation_pwm
        
        t5, t6 = depth_pwm
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
    
    def approaching_behavior(self, depth_pwm):
        """Approach gate with alignment corrections"""
        if not self.gate_detected:
            self.state = self.SEARCHING
            return
        
        if self.gate_area > self.MEDIUM_AREA:
            self.get_logger().info('📍 Gate close - switching to ALIGNING')
            self.state = self.ALIGNING
            return
        
        # Calculate horizontal error
        horizontal_error = self.gate_cx - self.CENTER_X
        
        # Base forward speed (adaptive to distance)
        if self.gate_area < self.SMALL_AREA:
            base_speed = 100  # Far away - faster
        else:
            base_speed = 70   # Getting closer - slower
        
        # Forward motion base
        t1 = 1500 + base_speed  # Front-Right
        t2 = 1500 + base_speed  # Front-Left
        t3 = 1500 - base_speed  # Back-Right (negative coeff)
        t4 = 1500 - base_speed  # Back-Left (negative coeff)
        
        # Apply yaw correction
        yaw_correction = int(horizontal_error * 0.3)
        yaw_correction = max(-80, min(yaw_correction, 80))
        
        # For right turn (positive error): T1, T3 backward; T2, T4 forward
        t1 -= yaw_correction
        t3 -= yaw_correction
        t2 += yaw_correction
        t4 += yaw_correction
        
        t5, t6 = depth_pwm
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
    
    def aligning_behavior(self, depth_pwm):
        """Fine alignment at close range"""
        if not self.gate_detected:
            self.state = self.SEARCHING
            return
        
        if self.gate_area > self.LARGE_AREA:
            self.get_logger().info('🚀 Gate very close - PASSING!')
            self.state = self.PASSING
            self.passing_start_x = self.current_position[0]
            return
        
        horizontal_error = self.gate_cx - self.CENTER_X
        
        if abs(horizontal_error) < 80:
            # Well aligned - slow forward creep
            t1 = 1530
            t2 = 1530
            t3 = 1470
            t4 = 1470
        else:
            # Need alignment - rotate with minimal forward
            yaw_correction = int(horizontal_error * 0.5)
            yaw_correction = max(-100, min(yaw_correction, 100))
            
            t1 = 1510 - yaw_correction
            t2 = 1510 + yaw_correction
            t3 = 1490 - yaw_correction
            t4 = 1490 + yaw_correction
        
        t5, t6 = depth_pwm
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
    
    def passing_behavior(self, depth_pwm):
        """Full speed through gate"""
        if self.passing_start_x is None:
            self.passing_start_x = self.current_position[0]
        
        distance_traveled = self.current_position[0] - self.passing_start_x
        
        if distance_traveled > 3.5:
            self.get_logger().info('✅ GATE PASSED - Mission Complete!')
            self.state = self.COMPLETED
            self.mission_active = False
            return
        
        # Maximum forward thrust
        t1 = 1700  # Front-Right
        t2 = 1700  # Front-Left
        t3 = 1300  # Back-Right (negative coeff)
        t4 = 1300  # Back-Left (negative coeff)
        
        t5, t6 = depth_pwm
        
        self.send_thrusters(t1, t2, t3, t4, t5, t6)
        
        self.get_logger().info(
            f'🚀 PASSING: {distance_traveled:.2f}m / 3.5m',
            throttle_duration_sec=0.5
        )
    
    def completed_behavior(self):
        """Stop all motion"""
        self.send_neutral()
    
    def send_thrusters(self, t1, t2, t3, t4, t5, t6):
        """
        Send commands to all 6 thrusters in URDF order
        T1: Front-Right, T2: Front-Left, T3: Back-Right, T4: Back-Left
        T5: Vertical-Right, T6: Vertical-Left
        """
        # Clamp all values
        t1 = max(self.PWM_MIN, min(t1, self.PWM_MAX))
        t2 = max(self.PWM_MIN, min(t2, self.PWM_MAX))
        t3 = max(self.PWM_MIN, min(t3, self.PWM_MAX))
        t4 = max(self.PWM_MIN, min(t4, self.PWM_MAX))
        t5 = max(self.PWM_MIN, min(t5, self.PWM_MAX))
        t6 = max(self.PWM_MIN, min(t6, self.PWM_MAX))
        
        # Publish in URDF order: T1, T2, T3, T4, T5, T6
        thrusters = [t1, t2, t3, t4, t5, t6]
        
        for i, pwm in enumerate(thrusters):
            msg = Float64()
            msg.data = float(pwm)
            self.thruster_pubs[i].publish(msg)
        
        # Periodic logging
        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f'T1:{int(t1)} T2:{int(t2)} T3:{int(t3)} T4:{int(t4)} T5:{int(t5)} T6:{int(t6)}',
                throttle_duration_sec=1.0
            )
    
    def send_neutral(self):
        """Send neutral PWM to all thrusters"""
        self.send_thrusters(1500, 1500, 1500, 1500, 1500, 1500)


def main(args=None):
    rclpy.init(args=args)
    node = WhiteGateNavigatorFixed()
    
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