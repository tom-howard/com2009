#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):
    
    def __init__(self):
        super().__init__("line_follower")
        
        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10,
        )
        
        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="/cmd_vel",
            qos_profile=10
        )
        
        self.vel_cmd = Twist()
        self.shutdown = False
        
    def shutdown_ops(self):
        self.get_logger().info(
            "Shutting down..."
        )
        cv2.destroyAllWindows()
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def camera_callback(self, img_data):
        cvbridge_interface = CvBridge()
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"{e}")

        cv2.imshow("camera image", cv_img)

        height, width, _ = cv_img.shape
        ## TODO 1 (1)

        ## TODO 2 (2)

        ## TODO 3 (3)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"{node.get_name()} received a shutdown request (Ctrl+C)"
        )
    finally:
        node.shutdown_ops()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
