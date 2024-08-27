#!/usr/bin/env python3
# A simple ROS2 Publisher

import rclpy # (1)!
from rclpy.node import Node

from std_msgs.msg import String # (2)!

class SimplePublisher(Node): # (3)!
    
    def __init__(self):
        super().__init__("simple_publisher") # (4)!
        
        self.my_publisher = self.create_publisher(
            msg_type=String,
            topic="my_topic",
            qos_profile=10,
        ) # (5)!

        publish_rate = 1 # Hz
        self.timer = self.create_timer(
            timer_period_sec=1/publish_rate, 
            callback=self.timer_callback
        ) # (6)!
                
        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised." # (7)!
        )

    def timer_callback(self): # (8)!
        ros_time = self.get_clock().now().seconds_nanoseconds()

        topic_msg = String()
        topic_msg.data = f"The ROS time is {ros_time[0]} (seconds)."
        self.my_publisher.publish(topic_msg)
        self.get_logger().info(f"Publishing: '{topic_msg.data}'")

def main(args=None): # (9)!
    rclpy.init(args=args)
    my_simple_publisher = SimplePublisher()
    rclpy.spin(my_simple_publisher)
    my_simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': # (10)!
    main()
