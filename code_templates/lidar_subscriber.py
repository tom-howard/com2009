#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions # (1)!

from sensor_msgs.msg import LaserScan # (2)!

import numpy as np # (3)!

class LidarSubscriber(Node): 

    def __init__(self): 
        super().__init__("lidar_subscriber")

        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) # (4)!
        
        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    def lidar_callback(self, scan_data: LaserScan): 
        left = scan_data.ranges[0:21] 
        right = scan_data.ranges[-20:] # (5)!
        front = np.array(right + left) # (6)!
        
        valid_data = front[front != float("inf")] # (7)!
        if np.shape(valid_data)[0] > 0: # (8)!
            single_point_average = valid_data.mean() # (9)!
        else:
            single_point_average = float("nan") # (10)!

        self.get_logger().info(
            f"LiDAR Reading (front): {single_point_average:.3f} meters.",
            throttle_duration_sec = 1,
        ) # (11)!
    
def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown request (Ctrl+C) detected...")
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main()