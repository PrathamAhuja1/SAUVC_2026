#!/usr/bin/env python3

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
        
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CENTER_X = self.FRAME_WIDTH // 2
        self.CENTER_Y = self.FRAME_HEIGHT // 2
        
        self.LOWER_WHITE = np.array([0, 0, 180])
        self.UPPER_WHITE = np.array([180, 40, 255])
        
        self.MIN_DETECT_AREA = 800
        self.APPROACH_AREA = 60000
        
        self.K_SWAY = 0.010
        self.K_HEAVE = 0.008
        self.K_SURGE = 0.4
        
        self.TARGET_DEPTH = -0.5
        
        self.SEARCHING = 0
        self.APPROACHING = 1
        self.ALIGNING = 2
        self.PASSING = 3
        self.COMPLETED = 4
        
        self.state = self.SEARCHING
        self.state_start_time = None
        
        self.gate_detected = False
        self.gate_cx = 0
        self.gate_cy = 0
        self.gate_area = 0
        self.mission_active = True
        self.frame_count = 0
        
        self.start_position = None
        self.current_position = None
        self.current_depth = 0.0
        self.passing_start_x = None
        
        self.camera_sub = self.create_subscription(
            Image, '/front_left/image_raw', self.camera_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/autonomous/debug_image', 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('White Gate Navigator Initialized')
        self.get_logger().info(f'Target Depth: {self.TARGET_DEPTH}m')
    
    def odom_callback(self, msg):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = msg.pose.pose.position
        
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(
                f'Start: X={self.current_position.x:.2f}, '
                f'Y={self.current_position.y:.2f}, Z={self.current_position.z:.2f}'
            )
    
    def depth_control(self):
        depth_error = self.TARGET_DEPTH - self.current_depth
        deadband = 0.15
        
        if abs(depth_error) < deadband:
            return 0.0
        
        if abs(depth_error) < 0.4:
            z_cmd = depth_error * 0.6
        else:
            z_cmd = depth_error * 1.0
        
        return max(-0.8, min(z_cmd, 0.8))
    
    def detect_white_gate(self, cv_image):
        self.frame_count += 1
        
        blurred = cv2.GaussianBlur(cv_image, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.LOWER_WHITE, self.UPPER_WHITE)
        
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
        mask = cv2.dilate(mask, kernel_large, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        debug_img = cv_image.copy()
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        debug_img = cv2.addWeighted(debug_img, 0.6, mask_colored, 0.4, 0)
        
        cv2.line(debug_img, (self.CENTER_X, 0), (self.CENTER_X, self.FRAME_HEIGHT), (0, 255, 255), 2)
        cv2.line(debug_img, (0, self.CENTER_Y), (self.FRAME_WIDTH, self.CENTER_Y), (0, 255, 255), 2)
        
        cv2.putText(debug_img, f'Frame: {self.frame_count}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if not contours:
            cv2.putText(debug_img, 'NO GATE DETECTED', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return False, 0, 0, 0, debug_img
        
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.MIN_DETECT_AREA:
            cv2.putText(debug_img, f'OBJECT TOO SMALL: {int(area)}px', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
            cv2.drawContours(debug_img, contours, -1, (128, 128, 128), 2)
            return False, 0, 0, 0, debug_img
        
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return False, 0, 0, 0, debug_img
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        cv2.drawContours(debug_img, [largest_contour], -1, (0, 255, 0), 4)
        cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(debug_img, (cx, cy), 15, (0, 0, 255), -1)
        cv2.circle(debug_img, (cx, cy), 20, (255, 255, 255), 3)
        cv2.line(debug_img, (self.CENTER_X, self.CENTER_Y), (cx, cy), (255, 0, 255), 3)
        
        cv2.putText(debug_img, 'GATE DETECTED', (x, y - 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        cv2.putText(debug_img, f'Area: {int(area)} px', (x, y - 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        error_x = cx - self.CENTER_X
        error_y = cy - self.CENTER_Y
        cv2.putText(debug_img, f'Error X: {error_x:+d} px', (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(debug_img, f'Error Y: {error_y:+d} px', (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
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
            dist_text = "VERY CLOSE"
            color = (0, 255, 0)
        
        cv2.putText(debug_img, f'Distance: {dist_text}', (10, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        
        max_bar_width = 400
        bar_width = int((area / 100000) * max_bar_width)
        bar_width = min(bar_width, max_bar_width)
        cv2.rectangle(debug_img, (10, 170), (10 + bar_width, 190), color, -1)
        cv2.rectangle(debug_img, (10, 170), (10 + max_bar_width, 190), (255, 255, 255), 2)
        
        return True, cx, cy, area, debug_img
    
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detected, cx, cy, area, debug_img = self.detect_white_gate(cv_image)
            
            self.gate_detected = detected
            if detected:
                self.gate_cx = cx
                self.gate_cy = cy
                self.gate_area = area
            
            state_names = ['SEARCHING', 'APPROACHING', 'ALIGNING', 'PASSING', 'COMPLETED']
            state_color = (255, 255, 0) if detected else (128, 128, 128)
            cv2.putText(debug_img, f'State: {state_names[self.state]}', (10, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, state_color, 2)
            
            cv2.putText(debug_img, f'Depth: {self.current_depth:.2f}m', (10, 250),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Debug image publish error: {e}')
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
    
    def control_loop(self):
        if not self.mission_active:
            self.stop()
            return
        
        cmd = Twist()
        
        cmd.linear.z = self.depth_control()
        
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
        if self.gate_detected:
            self.get_logger().info('Gate detected - APPROACHING')
            self.transition_to(self.APPROACHING)
            return cmd
        
        cmd.linear.x = 0.3
        cmd.linear.y = 0.0
        cmd.angular.z = 0.2
        
        return cmd
    
    def approaching_behavior(self, cmd):
        if not self.gate_detected:
            self.get_logger().warn('Gate lost - SEARCHING')
            self.transition_to(self.SEARCHING)
            return cmd
        
        if self.gate_area > 40000:
            self.get_logger().info('Close enough - ALIGNING')
            self.transition_to(self.ALIGNING)
            return cmd
        
        error_x = self.gate_cx - self.CENTER_X
        error_y = self.gate_cy - self.CENTER_Y
        
        cmd.linear.x = self.K_SURGE
        cmd.linear.y = error_x * self.K_SWAY
        cmd.angular.z = 0.0
        
        return cmd
    
    def aligning_behavior(self, cmd):
        if not self.gate_detected:
            self.get_logger().warn('Gate lost during alignment')
            self.transition_to(self.SEARCHING)
            return cmd
        
        error_x = self.gate_cx - self.CENTER_X
        error_y = self.gate_cy - self.CENTER_Y
        
        if abs(error_x) < 80 and abs(error_y) < 80 and self.gate_area > self.APPROACH_AREA:
            self.get_logger().info('Well aligned - PASSING')
            self.passing_start_x = self.current_position.x if self.current_position else 0
            self.transition_to(self.PASSING)
            return cmd
        
        cmd.linear.x = 0.2 if self.gate_area < self.APPROACH_AREA else 0.0
        cmd.linear.y = error_x * self.K_SWAY * 1.8
        cmd.angular.z = 0.0
        
        return cmd
    
    def passing_behavior(self, cmd):
        if self.current_position and self.passing_start_x is not None:
            distance_traveled = self.current_position.x - self.passing_start_x
            
            if distance_traveled > 3.0:
                self.get_logger().info('Gate passed successfully')
                self.transition_to(self.COMPLETED)
                return cmd
        
        cmd.linear.x = 1.0
        
        if self.gate_detected:
            error_x = self.gate_cx - self.CENTER_X
            error_y = self.gate_cy - self.CENTER_Y
            
            if abs(error_x) > 150:
                cmd.linear.y = error_x * self.K_SWAY * 0.3
        
        return cmd
    
    def completed_behavior(self, cmd):
        self.mission_active = False
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        self.get_logger().info('MISSION COMPLETE')
        
        return cmd
    
    def transition_to(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
    
    def stop(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FixedWhiteGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()