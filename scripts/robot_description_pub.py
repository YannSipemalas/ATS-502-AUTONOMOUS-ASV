#!/usr/bin/env python3
"""
robot_description_pub.py
Publish /robot_description dengan QoS TRANSIENT_LOCAL agar robot_state_publisher
dan RViz RobotModel mendapat URDF lengkap meski bergabung belakangan.
"""
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

def main():
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    if not urdf_path or not os.path.exists(urdf_path):
        print(f"[ERROR] File URDF tidak ditemukan: {urdf_path}", file=sys.stderr)
        sys.exit(1)

    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    rclpy.init()
    node = Node('robot_description_pub')

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE
    )
    pub = node.create_publisher(String, '/robot_description', qos)

    msg = String()
    msg.data = urdf_content
    pub.publish(msg)
    node.get_logger().info(f"Published /robot_description ({len(urdf_content)} chars)")

    # Tetap hidup agar subscriber yang bergabung belakangan tetap mendapat data
    rclpy.spin(node)

if __name__ == '__main__':
    main()
