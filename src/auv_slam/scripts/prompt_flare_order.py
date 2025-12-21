#!/usr/bin/env python3
"""
Interactive Flare Order Prompt - SHORT FORM SUPPORTED
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


def print_banner():
    banner = """
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║        🎯  SAUVC FLARE BUMPING TASK - ORDER INPUT  🎯           ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝

The AUV is currently STABILIZING at safe depth (-0.8m)
It will NOT move until you enter the flare order!

═══════════════════════════════════════════════════════════════════

VALID FLARE ORDERS (SHORT FORM):
  r-y-b  →  Red → Yellow → Blue
  r-b-y  →  Red → Blue → Yellow
  y-r-b  →  Yellow → Red → Blue
  y-b-r  →  Yellow → Blue → Red
  b-r-y  →  Blue → Red → Yellow
  b-y-r  →  Blue → Yellow → Red

═══════════════════════════════════════════════════════════════════
"""
    print(banner)


def main(args=None):
    print_banner()
    
    valid_orders = ['r-y-b', 'r-b-y', 'y-r-b', 'y-b-r', 'b-r-y', 'b-y-r']
    
    order = None
    while order not in valid_orders:
        try:
            print("\n📝 Enter flare order (e.g., r-y-b): ", end='', flush=True)
            order = input().lower().strip()
            
            if order not in valid_orders:
                print(f"\n❌ Invalid order '{order}'!")
                print("   Please use: r-y-b, r-b-y, y-r-b, y-b-r, b-r-y, or b-y-r")
                print("   Try again...")
            else:
                break
        except (KeyboardInterrupt, EOFError):
            print("\n\n❌ Order input cancelled. Mission aborted.")
            sys.exit(1)
    
    # Display confirmation
    order_map = {'r': 'RED', 'y': 'YELLOW', 'b': 'BLUE'}
    colors = order.split('-')
    order_description = ' → '.join([order_map[c] for c in colors])
    
    print("\n" + "="*70)
    print("✅ ORDER CONFIRMED!")
    print("="*70)
    print(f"   Sequence: {order_description}")
    print(f"   Notation: {order.upper()}")
    print("="*70)
    print("\n📡 Sending order to AUV...", flush=True)
    
    # Initialize ROS and send order
    rclpy.init(args=args)
    node = Node('flare_order_prompt')
    publisher = node.create_publisher(String, '/flare/mission_order', 10)
    
    # Wait for publisher to be ready
    time.sleep(0.5)
    
    msg = String()
    msg.data = order  # Send SHORT form (r-y-b)
    
    # Send order multiple times to ensure delivery
    print("   Transmitting...", end='', flush=True)
    for i in range(15):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        if i % 3 == 0:
            print(".", end='', flush=True)
        time.sleep(0.1)
    
    print(" Done!")
    print("\n✅ Order transmitted successfully!")
    print("🚀 Mission starting in 3 seconds...")
    print("\n" + "="*70)
    print("You can now monitor the mission in the main terminal.")
    print("This window will remain open for reference.")
    print("="*70 + "\n")
    
    # Keep window open
    try:
        print("Press Ctrl+C to close this window...\n")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print("\nClosing order prompt window...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()