#!/usr/bin/env python3
"""
CLEAN Qualification Detector
Logic: Pure detection. No locking.
The Navigator handles 'blind' driving using compass heading.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32
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
        
        # Orange HSV range (Adjust for your simulator/pool)
        self.orange_lower = np.array([0, 20, 40])
        self.orange_upper = np.array([35, 255, 255])
        
        self.min_area = 300
        self.gate_detection_history = deque(maxlen=2)
        
        # Position tracking
        self.gate_x_position = 0.0
        self.current_position = None
        self.reverse_mode = False
        
        # Smoothing
        self.center_history = deque(maxlen=4)
        self.center_smoothing_alpha = 0.4
        
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
            
        # Publishers
        self.gate_detected_pub = self.create_publisher(Bool, '/qualification/gate_detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/qualification/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/qualification/estimated_distance', 10)
        self.gate_center_pub = self.create_publisher(Point, '/qualification/center_point', 10)
        self.debug_pub = self.create_publisher(Image, '/qualification/debug_image', 10)
        self.frame_position_pub = self.create_publisher(Float32, '/qualification/frame_position', 10)
        
        self.get_logger().info('✅ CLEAN Detector Started (Passive Mode)')

    def reverse_cb(self, msg): self.reverse_mode = msg.data
    
    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    
    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.image_width = msg.width

    def image_callback(self, msg):
        if self.camera_matrix is None or self.current_position is None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError: return
        
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # Distance Estimation
        current_x = self.current_position[0]
        if not self.reverse_mode: estimated_distance = abs(self.gate_x_position - current_x)
        else: estimated_distance = abs(current_x - self.gate_x_position)
        
        # Detect Orange
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
                    posts.append({'cx': cx, 'cy': cy, 'x': x, 'w': wb})
                    cv2.rectangle(debug_img, (x, y), (x+wb, y+hb), (0, 255, 0), 2)

        gate_detected = False
        frame_position = 0.0
        gate_center_x = w // 2

        if len(posts) >= 2:
            # Full Gate Detected
            posts.sort(key=lambda p: p['cx'])
            gate_detected = True
            
            # Calculate midpoint
            raw_center_x = (posts[0]['cx'] + posts[1]['cx']) // 2
            raw_center_y = (posts[0]['cy'] + posts[1]['cy']) // 2
            
            # Simple Smoothing
            if len(self.center_history) > 0:
                prev = self.center_history[-1]
                gate_center_x = int(self.center_smoothing_alpha * raw_center_x + (1 - self.center_smoothing_alpha) * prev[0])
            else:
                gate_center_x = raw_center_x
            
            self.center_history.append((gate_center_x, raw_center_y))
            frame_position = (gate_center_x - w/2) / (w/2)
            
            # Debug Visuals
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (0, 255, 0), 2)

        elif len(posts) == 1:
            # Partial detection - Report but Navigator will ignore if close
            gate_detected = True
            gate_center_x = posts[0]['cx']
            frame_position = (gate_center_x - w/2) / (w/2)
            cv2.putText(debug_img, "PARTIAL", (gate_center_x, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)

        # Publish
        self.gate_detection_history.append(gate_detected)
        confirmed = sum(self.gate_detection_history) >= 1
        
        self.gate_detected_pub.publish(Bool(data=confirmed))
        if confirmed:
            self.alignment_pub.publish(Float32(data=float(frame_position)))
            self.distance_pub.publish(Float32(data=float(estimated_distance)))
            self.frame_position_pub.publish(Float32(data=float(frame_position)))
            
        # Debug Center Line
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(QualificationDetector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()