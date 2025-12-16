#!/usr/bin/env python3
"""
COMMIT ZONE Qualification Navigator
Works with geometric inference detector
No center locking - uses commit zone for final approach
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class CommitZoneNavigator(Node):
    def __init__(self):
        super().__init__('qualification_navigator')
        
        # State machine
        self.SUBMERGING = 0
        self.SEARCHING = 1
        self.APPROACHING = 2
        self.ALIGNING = 3
        self.FINAL_APPROACH = 4
        self.COMMITTING = 5        # NEW: Commit zone - go straight
        self.PASSING = 6
        self.CLEARING = 7
        self.UTURN = 8
        self.POST_UTURN_ALIGN = 9
        self.REVERSE_SEARCHING = 10
        self.REVERSE_APPROACHING = 11
        self.REVERSE_ALIGNING = 12
        self.REVERSE_FINAL_APPROACH = 13
        self.REVERSE_COMMITTING = 14  # NEW: Reverse commit zone
        self.REVERSE_PASSING = 15
        self.REVERSE_CLEARING = 16
        self.COMPLETED = 17
        
        self.state = self.SUBMERGING
        
        # --- DECLARE PARAMETERS ---
        # Navigation parameters
        self.declare_parameter('search_forward_speed', 0.4)
        self.declare_parameter('approach_speed', 0.6)
        self.declare_parameter('approach_stop_distance', 3.0)
        self.declare_parameter('alignment_distance', 3.0)
        self.declare_parameter('alignment_threshold', 0.06)
        self.declare_parameter('alignment_max_time', 20.0)
        self.declare_parameter('final_approach_speed', 0.5)
        
        # Commit zone parameters
        self.declare_parameter('commit_distance', 1.2)
        self.declare_parameter('commit_alignment_threshold', 0.10)
        self.declare_parameter('commit_speed', 0.8)
        
        self.declare_parameter('passing_trigger_distance', 0.6)
        self.declare_parameter('passing_speed', 1.0)
        
        # U-turn parameters
        self.declare_parameter('uturn_forward_speed', 0.3)
        self.declare_parameter('uturn_angular_speed', 0.4)
        self.declare_parameter('uturn_depth', -0.8)

        # Gate and geometry parameters
        self.declare_parameter('mission_depth', -0.8)
        self.declare_parameter('gate_x_position', 0.0)
        self.declare_parameter('auv_length', 0.46)
        self.declare_parameter('clearance_margin', 4.0) # Default updated to 4.0
        
        # --- GET PARAMETERS ---
        self.search_forward_speed = self.get_parameter('search_forward_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.approach_stop_distance = self.get_parameter('approach_stop_distance').value
        self.alignment_distance = self.get_parameter('alignment_distance').value
        self.alignment_threshold = self.get_parameter('alignment_threshold').value
        self.alignment_max_time = self.get_parameter('alignment_max_time').value
        self.final_approach_speed = self.get_parameter('final_approach_speed').value
        
        self.commit_distance = self.get_parameter('commit_distance').value
        self.commit_alignment_threshold = self.get_parameter('commit_alignment_threshold').value
        self.commit_speed = self.get_parameter('commit_speed').value
        
        self.passing_trigger_distance = self.get_parameter('passing_trigger_distance').value
        self.passing_speed = self.get_parameter('passing_speed').value
        
        self.uturn_forward_speed = self.get_parameter('uturn_forward_speed').value
        self.uturn_angular_speed = self.get_parameter('uturn_angular_speed').value
        self.uturn_depth = self.get_parameter('uturn_depth').value

        # Geometry & Mission
        self.mission_depth = self.get_parameter('mission_depth').value
        self.gate_x_position = self.get_parameter('gate_x_position').value
        self.auv_length = self.get_parameter('auv_length').value
        self.clearance_margin = self.get_parameter('clearance_margin').value
        
        # Calculate clearance targets based on parameters
        # This ensures we move clearance_margin distance past the gate
        self.forward_clearance_x = self.gate_x_position + self.auv_length + self.clearance_margin
        self.reverse_clearance_x = self.gate_x_position - self.auv_length - self.clearance_margin
        
        self.gate_width = 1.5
        
        # State variables
        self.gate_detected = False
        self.partial_gate = False
        self.alignment_error = 0.0
        self.estimated_distance = 999.0
        self.frame_position = 0.0
        self.confidence = 0.0
        self.current_depth = 0.0
        self.current_position = None
        self.current_yaw = 0.0
        
        self.passing_start_position = None
        self.alignment_start_time = 0.0
        self.state_start_time = time.time()
        self.uturn_start_yaw = 0.0
        self.uturn_start_time = 0.0
        self.uturn_start_x = 0.0
        self.reverse_mode = False
        
        self.first_pass_complete = False
        self.second_pass_complete = False
        
        self.gate_lost_time = 0.0
        self.gate_lost_timeout = 3.0
        self.mission_start_time = time.time()
        
        # Subscriptions
        self.create_subscription(Bool, '/qualification/gate_detected', self.gate_cb, 10)
        self.create_subscription(Float32, '/qualification/alignment_error', self.align_cb, 10)
        self.create_subscription(Float32, '/qualification/estimated_distance', self.dist_cb, 10)
        self.create_subscription(Float32, '/qualification/frame_position', self.frame_pos_cb, 10)
        self.create_subscription(Float32, '/qualification/confidence', self.conf_cb, 10)
        self.create_subscription(Bool, '/qualification/partial_detection', self.partial_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_cb, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/qualification/state', 10)
        self.reverse_mode_pub = self.create_publisher(Bool, '/mission/reverse_mode', 10)
        
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ COMMIT ZONE QUALIFICATION NAVIGATOR (UPDATED)')
        self.get_logger().info('='*70)
        self.get_logger().info('   ✓ No center locking')
        self.get_logger().info('   ✓ Geometric inference detection')
        self.get_logger().info(f'   ✓ Commit zone at {self.commit_distance}m')
        self.get_logger().info(f'   ✓ Clearance margin: {self.clearance_margin}m')
        self.get_logger().info(f'   ✓ Forward clearance target X: {self.forward_clearance_x:.2f}m')
        self.get_logger().info('='*70)
    
    def gate_cb(self, msg: Bool):
        self.gate_detected = msg.data
    
    def align_cb(self, msg: Float32):
        self.alignment_error = msg.data
    
    def dist_cb(self, msg: Float32):
        self.estimated_distance = msg.data
    
    def frame_pos_cb(self, msg: Float32):
        self.frame_position = msg.data
    
    def conf_cb(self, msg: Float32):
        self.confidence = msg.data
    
    def partial_cb(self, msg: Bool):
        self.partial_gate = msg.data
    
    def odom_cb(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def control_loop(self):
        cmd = Twist()
        
        # Depth control
        if self.state == self.PASSING or self.state == self.REVERSE_PASSING or \
           self.state == self.COMMITTING or self.state == self.REVERSE_COMMITTING:
            cmd.linear.z = self.gentle_depth_control(self.mission_depth)
        elif self.state == self.UTURN:
            pass  # U-turn handles its own depth
        else:
            cmd.linear.z = self.depth_control(self.mission_depth)
        
        # State machine
        if self.state == self.SUBMERGING:
            cmd = self.submerge(cmd)
        elif self.state == self.SEARCHING:
            cmd = self.searching(cmd)
        elif self.state == self.APPROACHING:
            cmd = self.approaching(cmd)
        elif self.state == self.ALIGNING:
            cmd = self.aligning(cmd)
        elif self.state == self.FINAL_APPROACH:
            cmd = self.final_approach(cmd)
        elif self.state == self.COMMITTING:
            cmd = self.committing(cmd)
        elif self.state == self.PASSING:
            cmd = self.passing(cmd)
        elif self.state == self.CLEARING:
            cmd = self.clearing(cmd)
        elif self.state == self.UTURN:
            cmd = self.uturn(cmd)
        elif self.state == self.POST_UTURN_ALIGN:
            cmd = self.post_uturn_align(cmd)
        elif self.state == self.REVERSE_SEARCHING:
            cmd = self.searching(cmd)
        elif self.state == self.REVERSE_APPROACHING:
            cmd = self.approaching(cmd)
        elif self.state == self.REVERSE_ALIGNING:
            cmd = self.aligning(cmd)
        elif self.state == self.REVERSE_FINAL_APPROACH:
            cmd = self.final_approach(cmd)
        elif self.state == self.REVERSE_COMMITTING:
            cmd = self.committing(cmd)
        elif self.state == self.REVERSE_PASSING:
            cmd = self.passing(cmd)
        elif self.state == self.REVERSE_CLEARING:
            cmd = self.reverse_clearing(cmd)
        elif self.state == self.COMPLETED:
            cmd = self.completed(cmd)
        
        self.cmd_vel_pub.publish(cmd)
        self.state_pub.publish(String(data=self.get_state_name()))
    
    def depth_control(self, target_depth: float) -> float:
        """Normal depth control"""
        depth_error = target_depth - self.current_depth
        deadband = 0.15
        
        if abs(depth_error) < deadband:
            return 0.0
        
        if abs(depth_error) < 0.4:
            z_cmd = depth_error * 0.4
        elif abs(depth_error) < 0.8:
            z_cmd = depth_error * 0.6
        else:
            z_cmd = depth_error * 0.8
        
        return max(-0.6, min(z_cmd, 0.6))
    
    def gentle_depth_control(self, target_depth: float) -> float:
        """Gentle depth control during passage/commit"""
        depth_error = target_depth - self.current_depth
        deadband = 0.25
        
        if abs(depth_error) < deadband:
            return 0.0
        
        z_cmd = depth_error * 0.2
        return max(-0.3, min(z_cmd, 0.3))
    
    def submerge(self, cmd: Twist) -> Twist:
        if abs(self.mission_depth - self.current_depth) < 0.3:
            if time.time() - self.state_start_time > 3.0:
                self.get_logger().info('✅ Submerged - starting search')
                self.reverse_mode_pub.publish(Bool(data=self.reverse_mode))
                self.transition_to(self.SEARCHING)
        return cmd
    
    def searching(self, cmd: Twist) -> Twist:
        if self.gate_detected and self.estimated_distance < 999:
            self.get_logger().info(f'🎯 Gate found at {self.estimated_distance:.2f}m')
            if self.reverse_mode:
                self.transition_to(self.REVERSE_APPROACHING)
            else:
                self.transition_to(self.APPROACHING)
            return cmd
        
        cmd.linear.x = self.search_forward_speed
        cmd.angular.z = 0.3 if (time.time() % 8 < 4) else -0.3
        return cmd
    
    def approaching(self, cmd: Twist) -> Twist:
        if not self.gate_detected:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.3
            return cmd
        
        if self.estimated_distance <= self.approach_stop_distance:
            self.get_logger().info(f'🛑 Reached 3m - ALIGNING')
            if self.reverse_mode:
                self.transition_to(self.REVERSE_ALIGNING)
            else:
                self.transition_to(self.ALIGNING)
            return cmd
        
        cmd.linear.x = self.approach_speed
        cmd.angular.z = -self.frame_position * 1.0
        return cmd
    
    def aligning(self, cmd: Twist) -> Twist:
        if not self.gate_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3
            return cmd
        
        if self.alignment_start_time == 0.0:
            self.alignment_start_time = time.time()
            self.get_logger().info(f'🎯 ALIGNING at 3m')
        
        elapsed = time.time() - self.alignment_start_time
        
        if elapsed > self.alignment_max_time:
            self.get_logger().warn('⏰ Alignment timeout')
            self.alignment_start_time = 0.0
            if self.reverse_mode:
                self.transition_to(self.REVERSE_FINAL_APPROACH)
            else:
                self.transition_to(self.FINAL_APPROACH)
            return cmd
        
        is_aligned = abs(self.frame_position) < self.alignment_threshold
        has_confidence = self.confidence > 0.8 or self.partial_gate  # Accept partial
        
        if is_aligned and has_confidence:
            self.get_logger().info(f'✅ ALIGNED ({elapsed:.1f}s)')
            self.alignment_start_time = 0.0
            if self.reverse_mode:
                self.transition_to(self.REVERSE_FINAL_APPROACH)
            else:
                self.transition_to(self.FINAL_APPROACH)
            return cmd
        
        quality = abs(self.frame_position)
        if quality > 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.frame_position * 4.0
        elif quality > 0.1:
            cmd.linear.x = 0.1
            cmd.angular.z = -self.frame_position * 3.0
        else:
            cmd.linear.x = 0.15
            cmd.angular.z = -self.frame_position * 2.0
        
        return cmd
    
    def final_approach(self, cmd: Twist) -> Twist:
        """Approach from 3m to commit distance (1.2m)"""
        
        # Check if reached commit distance
        if self.estimated_distance <= self.commit_distance:
            if abs(self.frame_position) < self.commit_alignment_threshold:
                self.get_logger().info(
                    f'🎯 ENTERING COMMIT ZONE at {self.estimated_distance:.2f}m '
                    f'(aligned: {self.frame_position:+.3f})'
                )
                if self.reverse_mode:
                    self.transition_to(self.REVERSE_COMMITTING)
                else:
                    self.transition_to(self.COMMITTING)
                return cmd
            else:
                # Too misaligned - emergency correction
                self.get_logger().warn(
                    f'⚠️ At commit point but misaligned ({self.frame_position:+.3f})!'
                )
                cmd.linear.x = 0.1
                cmd.angular.z = -self.frame_position * 4.0
                return cmd
        
        # Normal final approach
        if abs(self.frame_position) > 0.10:
            cmd.linear.x = self.final_approach_speed * 0.6
            cmd.angular.z = -self.frame_position * 3.0
        else:
            cmd.linear.x = self.final_approach_speed
            cmd.angular.z = -self.frame_position * 1.5
        
        return cmd
    
    def committing(self, cmd: Twist) -> Twist:
        """
        COMMIT ZONE: Go straight with minimal corrections
        From 1.2m to passing trigger (0.6m)
        """
        
        # Check if reached passing trigger
        if self.estimated_distance <= self.passing_trigger_distance:
            self.get_logger().info(
                f'🚀 PASSING TRIGGER at {self.estimated_distance:.2f}m'
            )
            self.passing_start_position = self.current_position
            if self.reverse_mode:
                self.transition_to(self.REVERSE_PASSING)
            else:
                self.transition_to(self.PASSING)
            return cmd
        
        # COMMIT: Mostly straight with very gentle corrections
        cmd.linear.x = self.commit_speed
        
        # Only apply corrections if:
        # 1. Gate is detected
        # 2. Alignment error is significant (>15%)
        # 3. We're not too close (>0.8m)
        
        if self.gate_detected and abs(self.frame_position) > 0.15 and self.estimated_distance > 0.8:
            # Gentle correction
            cmd.angular.z = -self.frame_position * 0.8
            self.get_logger().info(
                f'🎯 COMMIT (correcting): dist={self.estimated_distance:.2f}m, '
                f'pos={self.frame_position:+.3f}',
                throttle_duration_sec=0.5
            )
        else:
            # Pure straight
            cmd.angular.z = 0.0
            self.get_logger().info(
                f'➡️ COMMIT (straight): dist={self.estimated_distance:.2f}m',
                throttle_duration_sec=0.5
            )
        
        return cmd
    
    def passing(self, cmd: Twist) -> Twist:
        """Full passage - pure forward thrust"""
        if self.passing_start_position is None:
            self.passing_start_position = self.current_position
            direction = "REVERSE" if self.reverse_mode else "FORWARD"
            self.get_logger().info(f'🚀 {direction} PASSAGE STARTED')
        
        if self.current_position:
            current_x = self.current_position[0]
            
            if not self.reverse_mode:
                auv_back_x = current_x - self.auv_length
                back_passed = auv_back_x > self.gate_x_position
            else:
                auv_back_x = current_x + self.auv_length
                back_passed = auv_back_x < self.gate_x_position
            
            if back_passed:
                self.get_logger().info('✅ AUV BACK PASSED GATE → CLEARING')
                
                if not self.reverse_mode:
                    self.first_pass_complete = True
                    self.transition_to(self.CLEARING)
                else:
                    self.second_pass_complete = True
                    self.transition_to(self.REVERSE_CLEARING)
                return cmd
        
        # Pure forward during passing
        cmd.linear.x = self.passing_speed
        cmd.angular.z = 0.0
        return cmd
    
    def clearing(self, cmd: Twist) -> Twist:
        """Forward clearance - moves 4m past gate if clearance_margin is 4.0"""
        if self.current_position:
            current_x = self.current_position[0]
            
            if current_x >= self.forward_clearance_x:
                self.get_logger().info('✅ CLEARANCE COMPLETE → U-TURN')
                self.uturn_start_time = 0.0
                self.transition_to(self.UTURN)
                return cmd
            
            distance_needed = self.forward_clearance_x - current_x
            self.get_logger().info(
                f'🏃 CLEARING: need {distance_needed:.2f}m more',
                throttle_duration_sec=0.4
            )
        
        cmd.linear.x = 0.8
        return cmd
    
    def uturn(self, cmd: Twist) -> Twist:
        """U-turn with depth control"""
        
        if self.uturn_start_time == 0.0:
            self.uturn_start_yaw = self.current_yaw
            self.uturn_start_time = time.time()
            self.uturn_start_x = self.current_position[0] if self.current_position else 0.0
            self.get_logger().info('🔄 STARTING U-TURN')
        
        angle_turned = abs(self.normalize_angle(self.current_yaw - self.uturn_start_yaw))
        
        if angle_turned > (math.pi - 0.17):
            self.get_logger().info('✅ U-TURN COMPLETE → POST-ALIGN')
            self.reverse_mode = True
            self.reverse_mode_pub.publish(Bool(data=True))
            self.uturn_start_time = 0.0
            self.transition_to(self.POST_UTURN_ALIGN)
            return cmd
        
        cmd.linear.x = self.uturn_forward_speed
        cmd.angular.z = self.uturn_angular_speed
        
        # Depth control during U-turn
        depth_error = self.uturn_depth - self.current_depth
        
        if abs(depth_error) > 0.2:
            cmd.linear.z = depth_error * 1.5
            cmd.linear.z = max(-0.8, min(cmd.linear.z, 0.8))
        elif abs(depth_error) > 0.1:
            cmd.linear.z = depth_error * 1.0
            cmd.linear.z = max(-0.5, min(cmd.linear.z, 0.5))
        else:
            cmd.linear.z = depth_error * 0.5
        
        return cmd
    
    def post_uturn_align(self, cmd: Twist) -> Twist:
        """Align with gate after U-turn"""
        
        if self.gate_detected:
            if abs(self.frame_position) < 0.15:
                self.get_logger().info('✅ POST-UTURN ALIGNED')
                self.transition_to(self.REVERSE_APPROACHING)
                return cmd
            else:
                cmd.linear.x = 0.2
                cmd.angular.z = -self.frame_position * 2.0
        else:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.2
        
        return cmd
    
    def reverse_clearing(self, cmd: Twist) -> Twist:
        """Reverse clearance - moves past gate by margin"""
        if self.current_position:
            current_x = self.current_position[0]
            
            if current_x <= self.reverse_clearance_x:
                self.get_logger().info('='*70)
                self.get_logger().info('🎉 QUALIFICATION COMPLETE!')
                self.get_logger().info('='*70)
                self.transition_to(self.COMPLETED)
                return cmd
            
            distance_needed = current_x - self.reverse_clearance_x
            self.get_logger().info(
                f'🏃 REVERSE CLEARING: need {distance_needed:.2f}m',
                throttle_duration_sec=0.4
            )
        
        cmd.linear.x = 0.8
        return cmd
    
    def completed(self, cmd: Twist) -> Twist:
        """Mission complete - stay in place"""
        
        if not hasattr(self, '_completion_reported'):
            self._completion_reported = False
        
        if not self._completion_reported:
            total_time = time.time() - self.mission_start_time
            
            self.get_logger().info('='*70)
            self.get_logger().info('🏆 QUALIFICATION COMPLETE!')
            self.get_logger().info(f'   Pass 1: {"✅" if self.first_pass_complete else "❌"}')
            self.get_logger().info(f'   Pass 2: {"✅" if self.second_pass_complete else "❌"}')
            self.get_logger().info(f'   Total time: {total_time:.1f}s')
            
            if self.first_pass_complete and self.second_pass_complete:
                self.get_logger().info('   🏆 POINTS: 2 - QUALIFIED FOR FINALS!')
            elif self.first_pass_complete:
                self.get_logger().info('   ⚠️ POINTS: 1')
            
            self.get_logger().info('='*70)
            self._completion_reported = True
        
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        # Maintain depth
        depth_error = self.mission_depth - self.current_depth
        if abs(depth_error) > 0.15:
            cmd.linear.z = depth_error * 0.5
            cmd.linear.z = max(-0.4, min(cmd.linear.z, 0.4))
        else:
            cmd.linear.z = 0.0
        
        return cmd
    
    def transition_to(self, new_state: int):
        """Transition to new state"""
        old_name = self.get_state_name()
        self.state = new_state
        self.state_start_time = time.time()
        new_name = self.get_state_name()
        
        self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')
    
    def get_state_name(self) -> str:
        """Get state name"""
        names = {
            self.SUBMERGING: 'SUBMERGING',
            self.SEARCHING: 'SEARCHING',
            self.APPROACHING: 'APPROACHING',
            self.ALIGNING: 'ALIGNING',
            self.FINAL_APPROACH: 'FINAL_APPROACH',
            self.COMMITTING: 'COMMITTING',
            self.PASSING: 'PASSING',
            self.CLEARING: 'CLEARING',
            self.UTURN: 'UTURN',
            self.POST_UTURN_ALIGN: 'POST_UTURN_ALIGN',
            self.REVERSE_SEARCHING: 'REVERSE_SEARCHING',
            self.REVERSE_APPROACHING: 'REVERSE_APPROACHING',
            self.REVERSE_ALIGNING: 'REVERSE_ALIGNING',
            self.REVERSE_FINAL_APPROACH: 'REVERSE_FINAL_APPROACH',
            self.REVERSE_COMMITTING: 'REVERSE_COMMITTING',
            self.REVERSE_PASSING: 'REVERSE_PASSING',
            self.REVERSE_CLEARING: 'REVERSE_CLEARING',
            self.COMPLETED: 'COMPLETED',
        }
        return names.get(self.state, 'UNKNOWN')
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = CommitZoneNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()