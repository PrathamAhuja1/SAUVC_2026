#!/usr/bin/env python3

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
import message_filters


class StereoQualificationDetector(Node):
    def __init__(self):
        super().__init__('stereo_qualification_detector')
        self.bridge = CvBridge()
        
        self.left_camera_matrix = None
        self.right_camera_matrix = None
        self.left_dist_coeffs = None
        self.right_dist_coeffs = None
        self.image_size = None
        
        self.baseline = 0.16
        self.focal_length = None
        
        self.orange_lower = np.array([0, 20, 40])
        self.orange_upper = np.array([35, 255, 255])
        
        self.min_area = 300
        self.gate_detection_history = deque(maxlen=3)
        self.reverse_mode = False
        
        self.expected_gate_width_meters = 1.5
        
        self.gate_x_position = 0.0
        self.current_position = None
        
        self.center_history = deque(maxlen=5)
        self.distance_history = deque(maxlen=10)
        self.smoothing_alpha = 0.4
        
        self.stereo_matcher = cv2.StereoBM_create(numDisparities=16*10, blockSize=15)
        self.stereo_matcher.setMinDisparity(0)
        self.stereo_matcher.setSpeckleWindowSize(100)
        self.stereo_matcher.setSpeckleRange(32)
        self.stereo_matcher.setDisp12MaxDiff(1)
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, 
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.left_image_sub = message_filters.Subscriber(
            self, Image, '/front_left/image_raw', qos_profile=qos_sensor)
        self.right_image_sub = message_filters.Subscriber(
            self, Image, '/front_right/image_raw', qos_profile=qos_sensor)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_image_sub, self.right_image_sub], 10, 0.1)
        self.ts.registerCallback(self.stereo_callback)
        
        self.left_cam_info_sub = self.create_subscription(
            CameraInfo, '/front_left/camera_info', self.left_cam_info_callback, 10)
        self.right_cam_info_sub = self.create_subscription(
            CameraInfo, '/front_right/camera_info', self.right_cam_info_callback, 10)
        
        self.reverse_mode_sub = self.create_subscription(
            Bool, '/mission/reverse_mode', self.reverse_mode_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        self.gate_detected_pub = self.create_publisher(Bool, '/qualification/gate_detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/qualification/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/qualification/estimated_distance', 10)
        self.confidence_pub = self.create_publisher(Float32, '/qualification/confidence', 10) 
        self.partial_gate_pub = self.create_publisher(Bool, '/qualification/partial_detection', 10)
        self.gate_center_pub = self.create_publisher(Point, '/qualification/center_point', 10)
        self.debug_pub = self.create_publisher(Image, '/qualification/debug_image', 10)
        self.disparity_pub = self.create_publisher(Image, '/qualification/disparity_image', 10)
        self.status_pub = self.create_publisher(String, '/qualification/status', 10)
        self.frame_position_pub = self.create_publisher(Float32, '/qualification/frame_position', 10)
        
        self.get_logger().info('Stereo Qualification Detector initialized')
    
    def reverse_mode_callback(self, msg: Bool):
        self.reverse_mode = msg.data
        if msg.data:
            self.center_history.clear()
            self.distance_history.clear()
    
    def odom_callback(self, msg: Odometry):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
    
    def left_cam_info_callback(self, msg: CameraInfo):
        if self.left_camera_matrix is None:
            self.left_camera_matrix = np.array(msg.k).reshape((3, 3))
            self.left_dist_coeffs = np.array(msg.d)
            self.image_size = (msg.width, msg.height)
            self.focal_length = self.left_camera_matrix[0, 0]
            self.get_logger().info(f'Left camera initialized: {msg.width}x{msg.height}, f={self.focal_length:.1f}')
    
    def right_cam_info_callback(self, msg: CameraInfo):
        if self.right_camera_matrix is None:
            self.right_camera_matrix = np.array(msg.k).reshape((3, 3))
            self.right_dist_coeffs = np.array(msg.d)

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        if self.left_camera_matrix is None or self.right_camera_matrix is None:
            return
        
        if self.current_position is None:
            return
        
        try:
            left_image = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_image = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Bridge error: {e}')
            return
        
        h, w = left_image.shape[:2]
        
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
        
        disparity_map = self.stereo_matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
        
        disparity_normalized = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX)
        disparity_colored = cv2.applyColorMap(disparity_normalized.astype(np.uint8), cv2.COLORMAP_JET)
        
        hsv_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2HSV)
        
        orange_mask = cv2.inRange(hsv_image, self.orange_lower, self.orange_upper)
        kernel = np.ones((5, 5), np.uint8)
        orange_mask_clean = cv2.dilate(orange_mask, kernel, iterations=3)
        
        orange_contours, _ = cv2.findContours(
            orange_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        posts = []
        for cnt in orange_contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            
            disparity_roi = disparity_map[max(0, cy-20):min(h, cy+20), 
                                         max(0, cx-20):min(w, cx+20)]
            valid_disparities = disparity_roi[disparity_roi > 0]
            
            if len(valid_disparities) > 0:
                avg_disparity = np.median(valid_disparities)
                
                if avg_disparity > 1.0:
                    distance = (self.focal_length * self.baseline) / avg_disparity
                else:
                    distance = 999.0
            else:
                distance = 999.0
            
            posts.append({
                'center': (cx, cy),
                'area': area,
                'bbox': (x, y, w_box, h_box),
                'x_pos': cx,
                'width_pixels': w_box,
                'distance': distance
            })
        
        debug_img = left_image.copy()
        
        gate_detected = False
        partial_gate = False
        alignment_error = 0.0
        estimated_distance = 999.0
        gate_center_x = w // 2
        gate_center_y = h // 2
        frame_position = 0.0
        confidence = 0.0
        detection_method = "NONE"
        
        if len(posts) >= 2:
            posts_sorted = sorted(posts, key=lambda p: p['x_pos'])
            left_post = posts_sorted[0]
            right_post = posts_sorted[1]
            
            detected_center_x = (left_post['center'][0] + right_post['center'][0]) // 2
            detected_center_y = (left_post['center'][1] + right_post['center'][1]) // 2
            
            gate_detected = True
            partial_gate = False
            confidence = 1.0
            detection_method = "STEREO_BOTH_POSTS"
            
            if len(self.center_history) > 0:
                prev_center = self.center_history[-1]
                gate_center_x = int(self.smoothing_alpha * detected_center_x + 
                                   (1 - self.smoothing_alpha) * prev_center[0])
                gate_center_y = int(self.smoothing_alpha * detected_center_y + 
                                   (1 - self.smoothing_alpha) * prev_center[1])
            else:
                gate_center_x = detected_center_x
                gate_center_y = detected_center_y
            
            self.center_history.append((gate_center_x, gate_center_y))
            
            frame_position = (gate_center_x - w/2) / (w/2)
            alignment_error = frame_position
            
            left_distance = left_post['distance']
            right_distance = right_post['distance']
            
            valid_distances = [d for d in [left_distance, right_distance] if d < 900]
            if valid_distances:
                estimated_distance = np.mean(valid_distances)
                self.distance_history.append(estimated_distance)
                
                if len(self.distance_history) > 3:
                    estimated_distance = np.median(list(self.distance_history))
            
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 25, (0, 255, 255), -1)
            cv2.line(debug_img, left_post['center'], right_post['center'], (0, 255, 0), 3)
            
            for post in [left_post, right_post]:
                x, y, w_b, h_b = post['bbox']
                cv2.rectangle(debug_img, (x, y), (x+w_b, y+h_b), (255, 0, 255), 3)
                
                post_dist = post['distance']
                if post_dist < 900:
                    cv2.putText(debug_img, f"{post_dist:.2f}m", 
                               (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.putText(debug_img, f"STEREO: {estimated_distance:.2f}m", 
                       (gate_center_x - 100, gate_center_y - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
        
        elif len(posts) == 1:
            post = posts[0]
            post_x = post['center'][0]
            post_y = post['center'][1]
            
            gate_detected = True
            partial_gate = True
            detection_method = "STEREO_ONE_POST"
            
            current_x = self.current_position[0]
            if not self.reverse_mode:
                odometry_distance = abs(self.gate_x_position - current_x)
            else:
                odometry_distance = abs(current_x - self.gate_x_position)
            
            if post['distance'] < 900:
                estimated_distance = post['distance']
                self.distance_history.append(estimated_distance)
                confidence = 0.8
            else:
                estimated_distance = max(0.5, min(odometry_distance, 15.0))
                confidence = 0.5
            
            if len(self.distance_history) > 3:
                estimated_distance = np.median(list(self.distance_history))
            
            if estimated_distance > 0.5:
                expected_gate_width_pixels = (self.expected_gate_width_meters * self.focal_length) / estimated_distance
            else:
                expected_gate_width_pixels = w * 0.6
            
            frame_center = w / 2
            
            if post_x < frame_center:
                inferred_center_x = int(post_x + expected_gate_width_pixels / 2)
                post_label = "LEFT POST"
                confidence *= 0.9
            else:
                inferred_center_x = int(post_x - expected_gate_width_pixels / 2)
                post_label = "RIGHT POST"
                confidence *= 0.9
            
            inferred_center_x = max(50, min(inferred_center_x, w - 50))
            
            if len(self.center_history) > 0:
                prev_center = self.center_history[-1]
                gate_center_x = int(self.smoothing_alpha * inferred_center_x + 
                                   (1 - self.smoothing_alpha) * prev_center[0])
                gate_center_y = int(self.smoothing_alpha * post_y + 
                                   (1 - self.smoothing_alpha) * prev_center[1])
            else:
                gate_center_x = inferred_center_x
                gate_center_y = post_y
            
            self.center_history.append((gate_center_x, gate_center_y))
            
            frame_position = (gate_center_x - w/2) / (w/2)
            alignment_error = frame_position
            
            x, y, w_b, h_b = post['bbox']
            cv2.rectangle(debug_img, (x, y), (x+w_b, y+h_b), (0, 165, 255), 3)
            cv2.circle(debug_img, (post_x, post_y), 15, (0, 165, 255), -1)
            
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 30, (255, 165, 0), 3)
            cv2.line(debug_img, (post_x, post_y), (gate_center_x, gate_center_y), 
                    (255, 165, 0), 2, cv2.LINE_AA)
            
            cv2.putText(debug_img, f"{post_label} {estimated_distance:.2f}m", 
                       (gate_center_x - 120, gate_center_y - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
        
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        if gate_detected:
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (0, 255, 0), 3)
        
        status_y = 40
        cv2.putText(debug_img, f"{detection_method}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        status_y += 35
        
        cv2.putText(debug_img, f"Distance: {estimated_distance:.2f}m | Conf: {confidence:.2f}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        status_y += 35
        
        if gate_detected:
            status_color = (0, 255, 0) if not partial_gate else (255, 165, 0)
            cv2.putText(debug_img, f"Alignment: {alignment_error:+.3f}", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        if self.reverse_mode:
            cv2.putText(debug_img, "REVERSE MODE", 
                       (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        
        self.gate_detection_history.append(gate_detected)
        confirmed = sum(self.gate_detection_history) >= 1
        
        self.gate_detected_pub.publish(Bool(data=confirmed))
        self.partial_gate_pub.publish(Bool(data=partial_gate))
        self.confidence_pub.publish(Float32(data=confidence))
        
        if confirmed:
            self.alignment_pub.publish(Float32(data=float(alignment_error)))
            self.distance_pub.publish(Float32(data=float(estimated_distance)))
            self.frame_position_pub.publish(Float32(data=float(frame_position)))
            
            center_msg = Point()
            center_msg.x = float(gate_center_x)
            center_msg.y = float(gate_center_y)
            center_msg.z = float(estimated_distance)
            self.gate_center_pub.publish(center_msg)
            
            status_text = f"{detection_method} | Dist:{estimated_distance:.2f}m | Conf:{confidence:.2f}"
            self.status_pub.publish(String(data=status_text))
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = left_msg.header
            self.debug_pub.publish(debug_msg)
            
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity_colored, "bgr8")
            disparity_msg.header = left_msg.header
            self.disparity_pub.publish(disparity_msg)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = StereoQualificationDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()