#!/usr/bin/env python3
"""
Enhanced Flare Order Prompt - Same Terminal Version
Properly handles input in the same terminal as the launch command
Location: src/auv_slam/scripts/prompt_flare_order.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


def print_banner():
    """Display the order input banner"""
    banner = """
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║        🎯  SAUVC FLARE BUMPING TASK - ORDER INPUT  🎯           ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝

The AUV is currently STABILIZING at safe depth (-0.8m)
⚠️  All mission tasks are PAUSED until you enter the flare order!

═══════════════════════════════════════════════════════════════════

VALID FLARE ORDERS (use short form):
  
  r-y-b  →  Red → Yellow → Blue
  r-b-y  →  Red → Blue → Yellow
  y-r-b  →  Yellow → Red → Blue
  y-b-r  →  Yellow → Blue → Red
  b-r-y  →  Blue → Red → Yellow
  b-y-r  →  Blue → Yellow → Red

═══════════════════════════════════════════════════════════════════
"""
    print(banner, flush=True)


def main(args=None):
    """Main function to prompt for and send flare order"""
    
    # Print banner immediately
    print_banner()
    
    # Valid orders (short form only)
    valid_orders = ['r-y-b', 'r-b-y', 'y-r-b', 'y-b-r', 'b-r-y', 'b-y-r']
    
    order = None
    attempts = 0
    max_attempts = 10
    
    # Get valid order from user
    while order not in valid_orders and attempts < max_attempts:
        attempts += 1
        try:
            print(f"\n📝 Enter flare order (e.g., r-y-b): ", end='', flush=True)
            
            # Read input with timeout handling
            order_input = input()
            order = order_input.lower().strip()
            
            if order not in valid_orders:
                print(f"\n❌ Invalid order '{order}'!", flush=True)
                print("   Please use one of: r-y-b, r-b-y, y-r-b, y-b-r, b-r-y, b-y-r", flush=True)
                
                if attempts >= max_attempts:
                    print(f"\n⚠️ Maximum attempts ({max_attempts}) reached. Mission aborted.", flush=True)
                    sys.exit(1)
                
                print("   Try again...", flush=True)
            else:
                break
                
        except (KeyboardInterrupt, EOFError):
            print("\n\n❌ Order input cancelled. Mission aborted.", flush=True)
            sys.exit(1)
        except Exception as e:
            print(f"\n❌ Error reading input: {e}", flush=True)
            print("   Try again...", flush=True)
    
    if order not in valid_orders:
        print("\n❌ Failed to get valid order. Mission aborted.", flush=True)
        sys.exit(1)
    
    # Display confirmation
    order_map = {'r': 'RED', 'y': 'YELLOW', 'b': 'BLUE'}
    colors = order.split('-')
    order_description = ' → '.join([order_map[c] for c in colors])
    
    print("\n" + "="*70, flush=True)
    print("✅ ORDER CONFIRMED!", flush=True)
    print("="*70, flush=True)
    print(f"   Sequence: {order_description}", flush=True)
    print(f"   Notation: {order.upper()}", flush=True)
    print("="*70, flush=True)
    print("\n📡 Transmitting order to AUV...", end='', flush=True)
    
    # Initialize ROS and send order
    try:
        rclpy.init(args=args)
    except Exception as e:
        print(f"\n❌ Failed to initialize ROS2: {e}", flush=True)
        sys.exit(1)
    
    node = Node('flare_order_prompt')
    publisher = node.create_publisher(String, '/flare/mission_order', 10)
    
    # Wait for publisher to be ready
    time.sleep(0.5)
    
    msg = String()
    msg.data = order  # Send SHORT form (r-y-b)
    
    # Send order multiple times to ensure delivery
    successful_transmissions = 0
    for i in range(15):
        try:
            publisher.publish(msg)
            successful_transmissions += 1
            rclpy.spin_once(node, timeout_sec=0.05)
            if i % 3 == 0:
                print(".", end='', flush=True)
            time.sleep(0.1)
        except Exception as e:
            print(f"\n⚠️ Transmission error: {e}", flush=True)
    
    if successful_transmissions > 0:
        print(" Done!", flush=True)
        print(f"\n✅ Order transmitted successfully! ({successful_transmissions}/15 messages sent)", flush=True)
        print("🚀 Mission starting in 3 seconds...", flush=True)
        print("\n" + "="*70, flush=True)
        print("Mission is now active. Monitor progress in the terminal.", flush=True)
        print("="*70 + "\n", flush=True)
        
        # Keep node alive briefly to ensure messages are sent
        time.sleep(3)
    else:
        print("\n❌ Failed to transmit order!", flush=True)
        sys.exit(1)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    print("Order prompt completed. Mission control active.\n", flush=True)


if __name__ == '__main__':
    main()