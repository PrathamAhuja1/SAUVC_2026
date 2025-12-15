#!/usr/bin/env python3
"""
FIXED Qualification Detector - Always Aims for Gate Center
Key fixes:
1. When both posts visible: Use midpoint
2. When one post visible: Infer center based on gate geometry (1.5m width)
3. Simplified debug image with only distance and state
4. Minimal logging
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class QualificationDetector(Node):
    def __init__(self):
        super().__init__('qualification_gate_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.image_width = 640
        self.image_height = 480
        
        # Orange HSV range
        self.orange_lower = np.array([0, 20, 40])
        self.orange_upper = np.array([35, 255, 255])
        
        self.min_area = 300
        self.gate_detection_history = deque(maxlen=2)
        
        # Gate geometry - 1.5m width
        self.gate_width_meters = 1.5
        self.gate_half_width = self.gate_width_meters / 2.0  # 0.75m
        
        # Position tracking
        self.gate_x_position = 0.0
        self.current_position = None
        self.reverse_mode = False
        self.current_state = "UNKNOWN"
        
        # Camera parameters
        self.fx = None  # Focal length
        
        # Smoothing
        self.center_history = deque(maxlen=3)
        
        # QoS
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, 
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_callback, qos_sensor)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_forward/camera_info', self.cam_info_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        self.reverse_mode_sub = self.create_subscription(
            Bool, '/mission/reverse_mode', self.reverse_cb, 10)
        self.state_sub = self.create_subscription(
            String, '/qualification/state', self.state_cb, 10)
            
        # Publishers
        self.gate_detected_pub = self.create_publisher(Bool, '/qualification/gate_detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/qualification/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/qualification/estimated_distance', 10)
        self.gate_center_pub = self.create_publisher(Point, '/qualification/center_point', 10)
        self.debug_pub = self.create_publisher(Image, '/qualification/debug_image', 10)
        self.frame_position_pub = self.create_publisher(Float32, '/qualification/frame_position', 10)
        
        self.get_logger().info('✓ Detector Started')

    def reverse_cb(self, msg): 
        self.reverse_mode = msg.data
    
    def state_cb(self, msg):
        self.current_state = msg.data
    
    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    
    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.image_width = msg.width
            self.image_height = msg.height
            self.fx = self.camera_matrix[0, 0]

    def image_callback(self, msg):
        if self.camera_matrix is None or self.current_position is None: 
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError: 
            return
        
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # Distance Estimation from odometry
        current_x = self.current_position[0]
        if not self.reverse_mode: 
            estimated_distance = abs(self.gate_x_position - current_x)
        else: 
            estimated_distance = abs(current_x - self.gate_x_position)
        
        # Detect Orange posts
        orange_mask = cv2.inRange(hsv_image, self.orange_lower, self.orange_upper)
        kernel = np.ones((5, 5), np.uint8)
        orange_mask = cv2.dilate(orange_mask, kernel, iterations=2)
        
        contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        posts = []
        
        for cnt in contours:
            if cv2.contourArea(cnt) > self.min_area:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    x, y, wb, hb = cv2.boundingRect(cnt)
                    posts.append({'cx': cx, 'cy': cy, 'x': x, 'y': y, 'w': wb, 'h': hb})

        gate_detected = False
        frame_position = 0.0
        gate_center_x = w // 2
        gate_center_y = h // 2

        if len(posts) >= 2:
            # ============================================================
            # CASE 1: BOTH POSTS VISIBLE - Use actual midpoint
            # ============================================================
            posts.sort(key=lambda p: p['cx'])
            gate_detected = True
            
            left_post = posts[0]
            right_post = posts[1]
            
            # Calculate true center
            raw_center_x = (left_post['cx'] + right_post['cx']) // 2
            raw_center_y = (left_post['cy'] + right_post['cy']) // 2
            
            # Simple smoothing
            if len(self.center_history) > 0:
                prev = self.center_history[-1]
                gate_center_x = int(0.4 * raw_center_x + 0.6 * prev[0])
                gate_center_y = int(0.4 * raw_center_y + 0.6 * prev[1])
            else:
                gate_center_x = raw_center_x
                gate_center_y = raw_center_y
            
            self.center_history.append((gate_center_x, gate_center_y))
            frame_position = (gate_center_x - w/2) / (w/2)
            
            # Visual markers
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (0, 255, 0), 3)
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 20, (0, 255, 0), -1)
            cv2.rectangle(debug_img, (left_post['x'], left_post['y']), 
                         (left_post['x']+left_post['w'], left_post['y']+left_post['h']), 
                         (0, 255, 0), 2)
            cv2.rectangle(debug_img, (right_post['x'], right_post['y']), 
                         (right_post['x']+right_post['w'], right_post['y']+right_post['h']), 
                         (0, 255, 0), 2)

        elif len(posts) == 1:
            # ============================================================
            # CASE 2: ONE POST VISIBLE - Infer center from gate geometry
            # ============================================================
            gate_detected = True
            post = posts[0]
            
            # Estimate distance to post for pixel-to-meter conversion
            # Using post height as reference (gate posts are ~1.4m tall)
            post_height_pixels = post['h']
            post_height_meters = 1.4
            
            if post_height_pixels > 0 and self.fx:
                # Distance = (RealHeight * FocalLength) / PixelHeight
                distance_to_post = (post_height_meters * self.fx) / post_height_pixels
                
                # Convert gate half-width (0.75m) to pixels at this distance
                pixels_per_meter = self.fx / distance_to_post
                gate_half_width_pixels = self.gate_half_width * pixels_per_meter
                
                # Determine if this is left or right post
                post_center_x = post['cx']
                image_center_x = w / 2
                
                if post_center_x < image_center_x:
                    # LEFT post visible -> gate center is to the RIGHT
                    inferred_center_x = int(post_center_x + gate_half_width_pixels)
                else:
                    # RIGHT post visible -> gate center is to the LEFT
                    inferred_center_x = int(post_center_x - gate_half_width_pixels)
                
                # Clamp to image bounds
                inferred_center_x = max(0, min(w-1, inferred_center_x))
                gate_center_x = inferred_center_x
                gate_center_y = post['cy']
                
            else:
                # Fallback: Simple offset
                if post['cx'] < w/2:
                    gate_center_x = post['cx'] + int(w * 0.15)
                else:
                    gate_center_x = post['cx'] - int(w * 0.15)
                gate_center_y = post['cy']
            
            frame_position = (gate_center_x - w/2) / (w/2)
            
            # Visual markers
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (255, 165, 0), 3)
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 20, (255, 165, 0), -1)
            cv2.rectangle(debug_img, (post['x'], post['y']), 
                         (post['x']+post['w'], post['y']+post['h']), 
                         (255, 165, 0), 2)
            
            # Draw arrow to inferred center
            cv2.arrowedLine(debug_img, (post['cx'], post['cy']), 
                           (gate_center_x, gate_center_y), (255, 255, 0), 2)

        # ============================================================
        # SIMPLIFIED DEBUG IMAGE - Only distance and state
        # ============================================================
        # Draw center line
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 1)
        
        # Simplified status overlay - ONLY distance and state
        status_lines = [
            f"Distance: {estimated_distance:.1f}m",
            f"State: {self.current_state}"
        ]
        
        # Simple status box
        box_height = len(status_lines) * 40 + 20
        cv2.rectangle(debug_img, (10, 10), (400, box_height), (0, 0, 0), -1)
        cv2.rectangle(debug_img, (10, 10), (400, box_height), (0, 255, 0), 2)
        
        for i, line in enumerate(status_lines):
            cv2.putText(debug_img, line, (20, 45 + i*40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        
        # Temporal filtering
        self.gate_detection_history.append(gate_detected)
        confirmed_gate = sum(self.gate_detection_history) >= 1
        
        # Publish all data
        self.gate_detected_pub.publish(Bool(data=confirmed_gate))
        if confirmed_gate:
            self.alignment_pub.publish(Float32(data=float(frame_position)))
            self.distance_pub.publish(Float32(data=float(estimated_distance)))
            self.frame_position_pub.publish(Float32(data=float(frame_position)))
            
            center_msg = Point()
            center_msg.x = float(gate_center_x)
            center_msg.y = float(gate_center_y)
            center_msg.z = float(estimated_distance)
            self.gate_center_pub.publish(center_msg)
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QualificationDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()