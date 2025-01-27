#!/usr/bin/env python3

import rclpy
from rclpy.node import Node # (1)!
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist # (2)!
from math import sqrt, pow, pi # (3)!

class Square(Node): # (4)!

    def __init__(self):
        super().__init__("square") # (5)!

        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        ) # (6)!

        self.state = 1
        self.change_state = True

        self.vel = Twist() # (7)!

        ctrl_rate = 10 
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        ) # (8)!

        self.timestamp = self.get_clock().now().nanoseconds # (9)!
        self.shutdown = False

        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )
    
    def timer_callback(self): # (10)!
        time_now = self.get_clock().now().nanoseconds
        elapsed_time = (time_now - self.timestamp) * 1e-9 # (11)!
        if self.change_state: # (12)!
            self.timestamp = self.get_clock().now().nanoseconds
            self.change_state = False
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.get_logger().info(
                f"Changing to state: {self.state}")
        elif self.state == 1: # (13)!
            if elapsed_time > 2:
                self.state = 2
                self.change_state = True
            else:
                self.vel.linear.x = 0.05
                self.vel.angular.z = 0.0
        elif self.state == 2: # (14)!
            if elapsed_time > 4:
                self.state = 1
                self.change_state = True
            else:
                self.vel.angular.z = 0.2
                self.vel.linear.x = 0.0

        self.get_logger().info(
            f"Publishing Velocities:\n"
            f"  linear.x: {self.vel.linear.x:.2f} [m/s] | angular.z: {self.vel.angular.z:.2f} [rad/s].",
            throttle_duration_sec=1,
        )
        self.vel_pub.publish(self.vel) # (15)!

    def on_shutdown(self): # (17)!
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

def main(args=None): # (16)!
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = Square()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()