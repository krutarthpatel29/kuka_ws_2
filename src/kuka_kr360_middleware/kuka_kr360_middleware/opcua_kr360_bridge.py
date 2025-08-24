#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Krutarth Patel

"""
Standalone KR360 OPC UA Bridge - No ROS package needed
Reads OPC UA joint positions and publishes to ROS 2 kr360_arm_controller
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from opcua import Client
import time

class KR360OPCUABridge(Node):
    def __init__(self):
        super().__init__('kr360_opcua_bridge')
        
        # KR360 controller topic
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/kr360_arm_controller/joint_trajectory', 
            10
        )

        # OPC UA client
        self.opcua_client = Client("opc.tcp://127.0.0.1:4840")
        
        try:
            self.opcua_client.connect()
            self.get_logger().info("Connected to KR360 OPC UA Server.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to OPC UA: {e}")
            return

        # Node ID where joint data is published
        self.joint_node = self.opcua_client.get_node("ns=2;i=2")
        self.last_sent = None  # Store the last sent value

        self.timer = self.create_timer(1.0, self.read_opcua_and_publish)

    def read_opcua_and_publish(self):
        try:
            value = self.joint_node.get_value()
            if isinstance(value, str):
                joint_values = [float(i) for i in value.split(',')]
            else:
                joint_values = list(value)

            # KR360 has 6 joints 
            if len(joint_values) != 6:
                self.get_logger().warn(f"Received invalid joint value length for KR360: {len(joint_values)} (expected 6)")
                self.get_logger().info(f"Received values: {joint_values}")
                return

            if joint_values == self.last_sent:
                return  # Avoid sending same command again

            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            
            # KR360 joint names (6 joints)
            traj.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6']
            
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.velocities = [0.0] * 6
            point.time_from_start.sec = 2  # 2 second execution time
            point.time_from_start.nanosec = 0
            
            traj.points.append(point)

            self.publisher_.publish(traj)
            self.last_sent = joint_values
            self.get_logger().info(f"Published KR360 joint trajectory: {joint_values}")
            
        except Exception as e:
            self.get_logger().error(f"Error reading from KR360 OPC UA: {e}")

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        try:
            self.opcua_client.disconnect()
            self.get_logger().info('Disconnected from OPC UA server')
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = KR360OPCUABridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KR360 Bridge stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
