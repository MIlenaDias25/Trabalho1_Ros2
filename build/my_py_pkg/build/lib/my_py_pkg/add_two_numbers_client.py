#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
	def __init__(self):
		super().__init__("add_two_inst_node")
		self.get_logger().info("Client Started")
		self.call_add_two_ints_server(7,9)
		
	def call_add_two_ints_server(self, a, b):
		client = self.create_client(AddTwoInts, "add_two_ints")
		
		while not client.wait_for_service(1.0):
			self.get_logger().warn("Waiting for Server to Add the numbers.")
			
		request = AddTwoInts.Request()
		request.a = a
		request.b = b
		
		future = client.call_async(request)
		future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
		
	def callback_call_add_two_ints(self, future, a, b):
		try:
			response = future.result()
			self.get_logger().info(str(a)+ " + " + str(b) + " = " + str(response.sum))
		except Exception as e: 
			self.get_logger().info().error("Service call falled %r" % (e,))

def main(args=None):
	rclpy.init(args=None)
	node = AddTwoIntsClientNode()
	rclpy.spin(node)
	rclpy.shutdown()
if __name__=="__main__":
	main()
