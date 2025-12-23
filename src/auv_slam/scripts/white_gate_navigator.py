#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math


class WhiteGateNavigatorPWM(Node):
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
        self.SMALL_AREA = 10000
        self.MEDIUM_AREA = 30000
        self.LARGE_AREA = 60000
        
        self.PWM_NEUTRAL = 1500
        self.PWM_MIN = 1300
        self.PWM_MAX = 1800
        
        self.TARGET_DEPTH = -0.5
        
        self.SEARCHING = 0
        self.APPROACHING = 1
        self.ALIGNING = 2
        self.PASSING = 3
        self.COMPLETED = 4
        
        self.state = self.SEARCHING
        
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
        
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(Float64, f'/thruster{i}_cmd', 10)
            self.thruster_pubs.append(pub)
        
        self.debug_image_pub = self.create_publisher(Image, '/autonomous/debug_image', 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('White Gate Navigator PWM Initialized')
        self.get_logger().info(f'Target Depth: {self.TARGET_DEPTH}m')
    
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
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        white_mask = cv2.inRange(hsv_image, self.LOWER_WHITE, self.UPPER_WHITE)
        
        kernel = np.ones((5, 5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        
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
                    
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(cv_image, (self.gate_cx, self.gate_cy), 10, (0, 0, 255), -1)
                    
                    cv2.putText(cv_image, f'Area: {int(area)}', (x, y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.line(cv_image, (self.CENTER_X, 0), (self.CENTER_X, self.FRAME_HEIGHT), (255, 0, 0), 2)
        cv2.line(cv_image, (0, self.CENTER_Y), (self.FRAME_WIDTH, self.CENTER_Y), (255, 0, 0), 2)
        
        state_text = ['SEARCHING', 'APPROACHING', 'ALIGNING', 'PASSING', 'COMPLETED'][self.state]
        cv2.putText(cv_image, f'State: {state_text}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except:
            pass
    
    def control_loop(self):
        if not self.mission_active or self.current_position is None:
            self.send_pwm_values(1500, 1500, 1500, 1500, 1500, 1500)
            return
        
        depth_error = self.TARGET_DEPTH - self.current_depth
        depth_pwm = self.depth_control(depth_error)
        
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
        if abs(error) < 0.1:
            return 1500
        
        pwm_range = 200
        pwm_change = int(error * pwm_range * 2.0)
        pwm_change = max(-pwm_range, min(pwm_change, pwm_range))
        
        return 1500 - pwm_change
    
    def searching_behavior(self, depth_pwm):
        if self.gate_detected:
            self.get_logger().info('Gate detected - switching to APPROACHING')
            self.state = self.APPROACHING
            return
        
        surge_left_pwm = 1600
        surge_right_pwm = 1600
        back_left_pwm = 1400
        back_right_pwm = 1400
        forward_left_pwm = 1500
        forward_right_pwm = 1550
        
        self.send_pwm_values(back_left_pwm, forward_right_pwm, surge_left_pwm, 
                            surge_right_pwm, back_right_pwm, forward_left_pwm)
    
    def approaching_behavior(self, depth_pwm):
        if not self.gate_detected:
            self.state = self.SEARCHING
            return
        
        if self.gate_area > self.MEDIUM_AREA:
            self.get_logger().info('Gate close enough - switching to ALIGNING')
            self.state = self.ALIGNING
            return
        
        horizontal_error = self.gate_cx - self.CENTER_X
        
        speed_factor = 1.0
        if self.gate_area < self.SMALL_AREA:
            speed_factor = 1.3
        elif self.gate_area < self.MEDIUM_AREA:
            speed_factor = 1.0
        
        base_forward = int(1500 + 150 * speed_factor)
        base_backward = int(1500 - 150 * speed_factor)
        
        yaw_correction = int(horizontal_error * 0.3)
        yaw_correction = max(-100, min(yaw_correction, 100))
        
        surge_left_pwm = base_forward
        surge_right_pwm = base_forward
        back_left_pwm = base_backward - yaw_correction
        back_right_pwm = base_backward + yaw_correction
        forward_left_pwm = 1500 - yaw_correction
        forward_right_pwm = 1500 + yaw_correction
        
        self.send_pwm_values(back_left_pwm, forward_right_pwm, surge_left_pwm, 
                            surge_right_pwm, back_right_pwm, forward_left_pwm)
    
    def aligning_behavior(self, depth_pwm):
        if not self.gate_detected:
            self.state = self.SEARCHING
            return
        
        if self.gate_area > self.LARGE_AREA:
            self.get_logger().info('Gate very close - switching to PASSING')
            self.state = self.PASSING
            self.passing_start_x = self.current_position[0]
            return
        
        horizontal_error = self.gate_cx - self.CENTER_X
        
        if abs(horizontal_error) < 80:
            slow_forward = 1550
            slow_backward = 1450
            
            surge_left_pwm = slow_forward
            surge_right_pwm = slow_forward
            back_left_pwm = slow_backward
            back_right_pwm = slow_backward
            forward_left_pwm = 1500
            forward_right_pwm = 1500
        else:
            yaw_correction = int(horizontal_error * 0.5)
            yaw_correction = max(-120, min(yaw_correction, 120))
            
            surge_left_pwm = 1520
            surge_right_pwm = 1520
            back_left_pwm = 1480 - yaw_correction
            back_right_pwm = 1480 + yaw_correction
            forward_left_pwm = 1500 - yaw_correction
            forward_right_pwm = 1500 + yaw_correction
        
        self.send_pwm_values(back_left_pwm, forward_right_pwm, surge_left_pwm, 
                            surge_right_pwm, back_right_pwm, forward_left_pwm)
    
    def passing_behavior(self, depth_pwm):
        if self.passing_start_x is None:
            self.passing_start_x = self.current_position[0]
        
        distance_traveled = self.current_position[0] - self.passing_start_x
        
        if distance_traveled > 3.5:
            self.get_logger().info('Passed through gate - MISSION COMPLETE')
            self.state = self.COMPLETED
            self.mission_active = False
            return
        
        full_forward = 1700
        full_backward = 1300
        
        surge_left_pwm = full_forward
        surge_right_pwm = full_forward
        back_left_pwm = full_backward
        back_right_pwm = full_backward
        forward_left_pwm = 1500
        forward_right_pwm = 1500
        
        self.send_pwm_values(back_left_pwm, forward_right_pwm, surge_left_pwm, 
                            surge_right_pwm, back_right_pwm, forward_left_pwm)
    
    def completed_behavior(self):
        self.send_pwm_values(1500, 1500, 1500, 1500, 1500, 1500)
    
    def send_pwm_values(self, back_left, forward_right, surge_left, surge_right, back_right, forward_left):
        pwm_values = [back_left, forward_right, surge_left, surge_right, back_right, forward_left]
        
        for i, pwm in enumerate(pwm_values):
            pwm = max(self.PWM_MIN, min(pwm, self.PWM_MAX))
            msg = Float64()
            msg.data = float(pwm)
            self.thruster_pubs[i].publish(msg)
        
        if self.frame_count % 20 == 0:
            pwm_str = '/'.join([str(int(p)) for p in pwm_values])
            self.get_logger().info(f'{pwm_str}')


def main(args=None):
    rclpy.init(args=args)
    node = WhiteGateNavigatorPWM()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_pwm_values(1500, 1500, 1500, 1500, 1500, 1500)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()