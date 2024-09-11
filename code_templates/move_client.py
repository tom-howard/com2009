#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srv.srv import SetBool  # Import the standard SetBool service

class MoveServiceClient(Node):
    def _init_(self):
        super()._init_('move_service_client')  # Initialize the node with a name
        self.client = self.create_client(SetBool, 'setbool')  # Create a service client

        while not self.client.wait_for_service(timeout_sec=1.0):
            print('Waiting for the move_service to be available...')
        self.request= SetBool.Request()

    def send_request(self):
        request = SetBool.Request()  # Create a request object
        request.data = True  # Set the request data

        future = self.client.call_async(request)  # Call the service asynchronously
        rclpy.spin_until_future_complete(self, future)  # Wait until the future completes

        if future.result() is not None:
            print (f'Response: {future.result().success}, {future.result().message}')
        else:
            print ('Service call failed')

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    client = MoveServiceClient()  # Create a client node
    client.send_request()  # Send the request to the service
    client.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown rclpy

if _name_ == '_main_':
    main()