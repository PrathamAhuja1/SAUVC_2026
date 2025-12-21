#!/usr/bin/env python3
"""
Interactive Flare Order Prompt
Runs at mission start, blocks until valid order is entered
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


def main(args=None):
    print("\n" + "="*70)
    print("🎯 SAUVC FLARE BUMPING TASK - ORDER INPUT")
    print("="*70)
    print("\nValid flare orders:")
    print("  r-y-b  →  Red → Yellow → Blue")
    print("  r-b-y  →  Red → Blue → Yellow")
    print("  y-r-b  →  Yellow → Red → Blue")
    print("  y-b-r  →  Yellow → Blue → Red")
    print("  b-r-y  →  Blue → Red → Yellow")
    print("  b-y-r  →  Blue → Yellow → Red")
    print("\n" + "="*70)
    
    valid_orders = ['r-y-b', 'r-b-y', 'y-r-b', 'y-b-r', 'b-r-y', 'b-y-r']
    
    order = None
    while order not in valid_orders:
        try:
            order = input("\n📝 Enter flare order (e.g., r-y-b): ").lower().strip()
            
            if order not in valid_orders:
                print(f"❌ Invalid order '{order}'. Please use format: r-y-b, r-b-y, etc.")
            else:
                break
        except (KeyboardInterrupt, EOFError):
            print("\n\n❌ Order input cancelled. Mission aborted.")
            sys.exit(1)
    
    order_map = {'r': 'RED', 'y': 'YELLOW', 'b': 'BLUE'}
    colors = order.split('-')
    order_description = ' → '.join([order_map[c] for c in colors])
    
    print("\n" + "="*70)
    print("✅ ORDER CONFIRMED")
    print("="*70)
    print(f"   Sequence: {order_description}")
    print(f"   Notation: {order.upper()}")
    print("="*70)
    print("\n📡 Sending order to AUV...")
    
    rclpy.init(args=args)
    node = Node('flare_order_prompt')
    publisher = node.create_publisher(String, '/flare/mission_order', 10)
    
    msg = String()
    msg.data = order
    
    for i in range(10):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    
    print("✅ Order transmitted successfully!")
    print("🚀 Mission starting...\n")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()