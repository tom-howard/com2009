#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class MoveService(Node):

    def _init_(self):
        super()._init_('move_service_server')
        self.service = self.create_service(SetBool, 'move_service', self.srv_callback)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        print("The 'move_service' server is ready to be called...")

    def srv_callback(self, request_from_client, response_from_server):
        vel = Twist()

        if request_from_client.data:  # request_signal is now .data in ROS2
            print("Server received a 'true' request and the robot will now move for 5 seconds...")

            start_time = self.get_clock().now()

            vel.linear.x = 0.1
            self.pub.publish(vel)

            print('Published the velocity command to /cmd_vel')
            while (self.get_clock().now() - start_time).seconds < 5:
                rclpy.spin_once(self, timeout_sec=0.1)  # Allows callbacks to be processed while waiting

            print ('5 seconds have elapsed, stopping the robot...')

            vel.linear.x = 0.0
            self.pub.publish(vel)

            response_from_server.success = True  # response_signal is now .success in ROS2
            response_from_server.message = "Request complete."
        else:
            response_from_server.success = False
            response_from_server.message = "Nothing happened, set request_signal to 'true' next time."
        return response_from_server

def main(args=None):
    rclpy.init(args=args)
    server = MoveService()
    rclpy.spin(server)

    # Clean up on shutdown
    server.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()