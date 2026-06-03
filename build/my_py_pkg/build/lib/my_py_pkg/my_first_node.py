#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
	def __init__(self):
		super().__init__("py_test_node")
		self.get_logger().info("Hello ROS2")

def main(args=None):
	rclpy.init(args=None)
	node = MyNode()
	rclpy.spin(node)
	rclpy.shutdown()
if __name__=="__main__":
	main()
