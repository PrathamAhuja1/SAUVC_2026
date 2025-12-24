#!/usr/bin/env python3
"""
ROBUST Gate Detector - Filters Out Thin Flares
Location: src/auv_slam/scripts/gate_detector_node.py

Key improvements:
1. Minimum width threshold to reject thin flares (1.6cm)
2. Aspect ratio constraints for gate posts (25cm wide blocks)
3. Area-based filtering
4. Geometric validation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class RobustGateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # HSV ranges for gate colors
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])
        
        self.green_lower = np.array([40, 100, 100])
        self.green_upper = np.array([80, 255, 255])
        
        # =====================================================================
        # CRITICAL: Detection parameters to filter out flares
        # =====================================================================
        # Gate posts: 25cm x 25cm blocks (LARGE)
        # Flares: 1.6cm diameter (THIN)
        self.min_stripe_area = 500           # Gate blocks are large
        self.min_stripe_width_pixels = 20    # Reject thin flares
        self.min_aspect_ratio = 1.5          # Taller than wide
        self.max_aspect_ratio = 8.0          # But not extremely thin
        
        # Expected gate geometry
        self.gate_width = 1.5  # meters
        
        self.gate_detection_history = deque(maxlen=5)
        self.frame_count = 0
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_callback, qos_sensor)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_forward/camera_info', self.cam_info_callback, 10)
        
        # Publishers
        self.gate_detected_pub = self.create_publisher(Bool, '/gate/detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/gate/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/gate/estimated_distance', 10)
        self.gate_center_pub = self.create_publisher(Point, '/gate/center_point', 10)
        self.debug_pub = self.create_publisher(Image, '/gate/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/gate/status', 10)
        self.frame_position_pub = self.create_publisher(Float32, '/gate/frame_position', 10)
        self.confidence_pub = self.create_publisher(Float32, '/gate/detection_confidence', 10)
        self.partial_gate_pub = self.create_publisher(Bool, '/gate/partial_detection', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ ROBUST Gate Detector - Flare Filtering Enabled')
        self.get_logger().info('   - Filters thin objects (flares: 1.6cm diameter)')
        self.get_logger().info('   - Detects wide gate posts (25cm blocks)')
        self.get_logger().info('   - Geometric validation for gate structure')
        self.get_logger().info('='*70)
    
    def cam_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.image_width = msg.width
            self.image_height = msg.height
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.get_logger().info(f'Camera: {self.image_width}x{self.image_height}, fx={self.fx:.1f}')
    
    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return
        
        self.frame_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # Create color masks
        red_mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        
        # Morphological operations to clean up noise
        kernel = np.ones((5, 5), np.uint8)
        red_mask_clean = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask_clean = cv2.morphologyEx(red_mask_clean, cv2.MORPH_OPEN, kernel)
        green_mask_clean = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        green_mask_clean = cv2.morphologyEx(green_mask_clean, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        red_contours, _ = cv2.findContours(red_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Detect gate stripes with STRICT filtering
        red_stripe = self.find_gate_stripe(red_contours, debug_img, (0, 0, 255), "RED POST")
        green_stripe = self.find_gate_stripe(green_contours, debug_img, (0, 255, 0), "GREEN POST")
        
        # Gate detection logic
        gate_detected = False
        partial_gate = False
        alignment_error = 0.0
        estimated_distance = 999.0
        gate_center_x = w // 2
        gate_center_y = h // 2
        frame_position = 0.0
        confidence = 0.0
        
        if red_stripe and green_stripe:
            # FULL GATE - both posts visible
            gate_detected = True
            partial_gate = False
            confidence = 1.0
            
            gate_center_x = (red_stripe['center'][0] + green_stripe['center'][0]) // 2
            gate_center_y = (red_stripe['center'][1] + green_stripe['center'][1]) // 2
            
            # Calculate alignment
            pixel_error = gate_center_x - (w / 2)
            alignment_error = pixel_error / (w / 2)
            frame_position = alignment_error
            
            # Estimate distance from stripe separation
            stripe_distance = abs(red_stripe['center'][0] - green_stripe['center'][0])
            if stripe_distance > 30:
                estimated_distance = (self.gate_width * self.fx) / stripe_distance
                estimated_distance = max(0.5, min(estimated_distance, 50.0))
            
            # Visualization
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 25, (255, 0, 255), -1)
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (255, 0, 255), 4)
            cv2.line(debug_img, red_stripe['center'], green_stripe['center'], (0, 255, 255), 5)
            
            cv2.putText(debug_img, f"GATE {estimated_distance:.1f}m", 
                       (gate_center_x - 80, gate_center_y - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 3)
        
        elif red_stripe or green_stripe:
            # PARTIAL GATE - only one post visible
            gate_detected = True
            partial_gate = True
            confidence = 0.6
            
            stripe = red_stripe if red_stripe else green_stripe
            stripe_name = "RED POST" if red_stripe else "GREEN POST"
            
            gate_center_x = stripe['center'][0]
            gate_center_y = stripe['center'][1]
            
            # Infer center position based on which post is visible
            if red_stripe:  # Left post visible
                desired_x = w * 0.35
            else:  # Right post visible
                desired_x = w * 0.65
            
            pixel_error = stripe['center'][0] - desired_x
            alignment_error = pixel_error / (w / 2)
            frame_position = alignment_error
            
            # Visualization
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 30, (255, 165, 0), 5)
            cv2.putText(debug_img, f"PARTIAL: {stripe_name}", 
                       (gate_center_x - 120, gate_center_y - 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 165, 0), 3)
        
        # Draw center line
        cv2.line(debug_img, (w//2, 0), (w//2, h), (0, 255, 255), 2)
        
        # Status overlay
        status_text = f"Frame {self.frame_count} | Conf: {confidence:.2f}"
        cv2.putText(debug_img, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if gate_detected:
            if partial_gate:
                status = f"PARTIAL @ {frame_position:+.2f}"
            else:
                status = f"FULL GATE {estimated_distance:.1f}m"
            cv2.putText(debug_img, status, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(debug_img, "NO GATE", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
        
        # Temporal filtering
        self.gate_detection_history.append(gate_detected)
        confirmed = sum(self.gate_detection_history) >= 2
        
        # Publish all data
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
            
            status_msg = f"{'PARTIAL' if partial_gate else 'FULL'} | Conf:{confidence:.2f}"
            self.status_pub.publish(String(data=status_msg))
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError:
            pass
    
    def find_gate_stripe(self, contours, debug_img, color, label):
        """
        Find gate stripe with STRICT filtering to reject flares
        
        Gate post characteristics:
        - Width: 25cm (large in image, typically >20 pixels)
        - Area: Large (>500 pixels for 25cm x 25cm blocks)
        - Aspect ratio: 1.5 - 8.0 (taller than wide, but not extremely thin)
        
        Flare characteristics (TO REJECT):
        - Width: 1.6cm (very thin in image, typically <10 pixels)
        - Area: Small (<200 pixels)
        - Aspect ratio: >15 (extremely thin and tall)
        """
        if not contours:
            return None
        
        best_stripe = None
        best_score = 0
        rejected_count = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # FILTER 1: Minimum area (reject small flares)
            if area < self.min_stripe_area:
                rejected_count += 1
                continue
            
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            if w_box == 0 or h_box == 0:
                continue
            
            # FILTER 2: Minimum width (reject thin flares)
            # This is the KEY filter - flares are ~1.6cm, gate posts are 25cm
            if w_box < self.min_stripe_width_pixels:
                rejected_count += 1
                # Visualize rejected flares
                cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), (128, 128, 128), 1)
                cv2.putText(debug_img, f"FLARE? W={w_box}px", (x, y-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                continue
            
            aspect_ratio = float(h_box) / w_box
            
            # FILTER 3: Aspect ratio range (reject extremely thin flares)
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                rejected_count += 1
                if aspect_ratio > self.max_aspect_ratio:
                    cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), (128, 128, 128), 1)
                    cv2.putText(debug_img, f"TOO THIN AR={aspect_ratio:.1f}", (x, y-5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
                continue
            
            # Score based on area and aspect ratio
            # Prefer larger, well-proportioned objects
            score = area * (1.0 + (aspect_ratio / 10.0))
            
            if score > best_score:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    best_stripe = {
                        'center': (cx, cy),
                        'bbox': (x, y, w_box, h_box),
                        'area': area,
                        'aspect': aspect_ratio,
                        'width': w_box,
                        'score': score
                    }
                    best_score = score
        
        if rejected_count > 0:
            self.get_logger().debug(f'{label}: Rejected {rejected_count} thin objects (likely flares)')
        
        # Visualize accepted stripe
        if best_stripe and debug_img is not None:
            cx, cy = best_stripe['center']
            x, y, w_box, h_box = best_stripe['bbox']
            
            # Draw thick rectangle around accepted gate post
            cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), color, 3)
            cv2.circle(debug_img, (cx, cy), 15, color, -1)
            
            # Label with details
            label_text = f"{label}"
            label_details = f"A:{int(best_stripe['area'])} W:{w_box}px AR:{best_stripe['aspect']:.1f}"
            cv2.putText(debug_img, label_text, (x, y-25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(debug_img, label_details, (x, y-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return best_stripe


def main(args=None):
    rclpy.init(args=args)
    node = RobustGateDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()