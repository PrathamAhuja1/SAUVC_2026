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