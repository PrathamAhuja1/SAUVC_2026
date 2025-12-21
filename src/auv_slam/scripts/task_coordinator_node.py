#!/usr/bin/env python3
"""
Task Coordinator - Manages Sequential Mission Tasks
Location: src/auv_slam/scripts/task_coordinator_node.py

Coordinates:
1. Flare Bumping Task
2. Gate Navigation Task
3. Additional tasks

Ensures proper task sequencing and prevents interference
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time


class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        
        # Task states
        self.FLARE_TASK = 0
        self.TRANSITION_TO_GATE = 1
        self.GATE_TASK = 2
        self.MISSION_COMPLETE = 3
        
        self.current_task = self.FLARE_TASK
        
        # Task completion flags
        self.flare_mission_complete = False
        self.gate_mission_complete = False
        
        # Transition delay
        self.transition_start_time = 0.0
        self.transition_delay = 5.0  # 5 seconds between tasks
        
        # Subscriptions
        self.flare_complete_sub = self.create_subscription(
            Bool, '/flare/mission_complete', self.flare_complete_callback, 10)
        self.gate_complete_sub = self.create_subscription(
            Bool, '/gate/mission_complete', self.gate_complete_callback, 10)
        
        # Publishers
        self.flare_enable_pub = self.create_publisher(Bool, '/flare/task_enable', 10)
        self.gate_enable_pub = self.create_publisher(Bool, '/gate/task_enable', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)
        
        # Timer
        self.timer = self.create_timer(0.5, self.update_task_state)
        
        # Initial state - Enable flare task
        self.enable_flare_task()
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ Task Coordinator Initialized')
        self.get_logger().info('='*70)
        self.get_logger().info('📋 MISSION SEQUENCE:')
        self.get_logger().info('   1. Flare Bumping Task (Active)')
        self.get_logger().info('   2. Gate Navigation Task (Disabled)')
        self.get_logger().info('='*70)
    
    def flare_complete_callback(self, msg: Bool):
        """Flare task completion callback"""
        if msg.data and not self.flare_mission_complete:
            self.flare_mission_complete = True
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 FLARE TASK COMPLETE!')
            self.get_logger().info('='*70)
    
    def gate_complete_callback(self, msg: Bool):
        """Gate task completion callback"""
        if msg.data and not self.gate_mission_complete:
            self.gate_mission_complete = True
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 GATE TASK COMPLETE!')
            self.get_logger().info('='*70)
    
    def update_task_state(self):
        """Update task coordinator state machine"""
        
        if self.current_task == self.FLARE_TASK:
            # Wait for flare task to complete
            if self.flare_mission_complete:
                self.get_logger().info('🔄 Transitioning from Flare Task to Gate Task...')
                self.disable_flare_task()
                self.transition_start_time = time.time()
                self.current_task = self.TRANSITION_TO_GATE
        
        elif self.current_task == self.TRANSITION_TO_GATE:
            # Wait for transition delay
            elapsed = time.time() - self.transition_start_time
            if elapsed >= self.transition_delay:
                self.get_logger().info('='*70)
                self.get_logger().info('🚀 STARTING GATE NAVIGATION TASK')
                self.get_logger().info('='*70)
                self.enable_gate_task()
                self.current_task = self.GATE_TASK
        
        elif self.current_task == self.GATE_TASK:
            # Wait for gate task to complete
            if self.gate_mission_complete:
                self.get_logger().info('='*70)
                self.get_logger().info('🏆 ALL MISSION TASKS COMPLETE!')
                self.get_logger().info('='*70)
                self.disable_gate_task()
                self.current_task = self.MISSION_COMPLETE
        
        elif self.current_task == self.MISSION_COMPLETE:
            # Mission finished
            pass
        
        # Publish mission status
        status = self.get_mission_status()
        self.mission_status_pub.publish(String(data=status))
    
    def enable_flare_task(self):
        """Enable flare detection and navigation"""
        self.flare_enable_pub.publish(Bool(data=True))
        self.get_logger().info('✅ Flare Task ENABLED')
    
    def disable_flare_task(self):
        """Disable flare detection and navigation"""
        self.flare_enable_pub.publish(Bool(data=False))
        self.get_logger().info('⏸️ Flare Task DISABLED')
    
    def enable_gate_task(self):
        """Enable gate detection and navigation"""
        self.gate_enable_pub.publish(Bool(data=True))
        self.get_logger().info('✅ Gate Task ENABLED')
    
    def disable_gate_task(self):
        """Disable gate detection and navigation"""
        self.gate_enable_pub.publish(Bool(data=False))
        self.get_logger().info('⏸️ Gate Task DISABLED')
    
    def get_mission_status(self) -> str:
        """Get current mission status string"""
        if self.current_task == self.FLARE_TASK:
            return "FLARE_TASK_ACTIVE"
        elif self.current_task == self.TRANSITION_TO_GATE:
            return "TRANSITIONING_TO_GATE"
        elif self.current_task == self.GATE_TASK:
            return "GATE_TASK_ACTIVE"
        elif self.current_task == self.MISSION_COMPLETE:
            return "MISSION_COMPLETE"
        else:
            return "UNKNOWN"


def main(args=None):
    rclpy.init(args=args)
    node = TaskCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()