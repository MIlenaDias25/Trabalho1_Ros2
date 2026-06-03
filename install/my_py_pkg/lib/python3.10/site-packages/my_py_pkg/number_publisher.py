#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
	def __init__(self):
		super().__init__("py_test_node")
		
		self.declare_parameter("number_to_publish", 102)
		self.declare_parameter("publish_frequency", 5.0)
		
		self.number = self.get_parameter("number_to_publish").value
		self.publish_frequency = self.get_parameter("publish_frequency").value
		
		self.number_publisher_ = self.create_publisher(Int64, "number", 10)
		self.number_timer_ = self.create_timer(1.0/ self.publish_frequency, self.publish_number)
		
	def publish_number(self):
		msg = Int64()
		msg.data = self.number
		self.number_publisher_.publish(msg)

def main(args=None):
	rclpy.init(args=None)
	node = NumberPublisherNode()
	rclpy.spin(node)
	rclpy.shutdown()
if __name__=="__main__":
	main()
