#!/usr/bin/env python3
# A simple ROS2 Subscriber

import rclpy # (1)!
from rclpy.node import Node

from example_interfaces.msg import String

class SimpleSubscriber(Node): # (2)! 

    def __init__(self): 
        super().__init__("simple_subscriber") # (3)!

        self.my_subscriber = self.create_subscription(
            msg_type=String,
            topic="{BLANK}",
            callback=self.msg_callback,
            qos_profile=10,
        ) # (4)!
        
        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        ) # (5)!

    def msg_callback(self, topic_message: String): # (6)!
        # (7)!
        self.get_logger().info(f"The '{self.get_name()}' node heard:") 
        self.get_logger().info(f"'{topic_message.data}'")
    
def main(args=None): # (8)!
    rclpy.init(args=args)
    my_simple_subscriber = SimpleSubscriber()
    rclpy.spin(my_simple_subscriber)
    my_simple_subscriber.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()