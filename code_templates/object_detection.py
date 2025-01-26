#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node # (1)!

import cv2
from cv_bridge import CvBridge, CvBridgeError # (2)!

from sensor_msgs.msg import Image # (3)!

from pathlib import Path # (4)!

class ObjectDetection(Node):

    def __init__(self): # (5)!
        super().__init__("object_detection")

        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )

        self.waiting_for_image = True # (6)!
    
    def camera_callback(self, img_data): # (7)!
        cvbridge_interface = CvBridge() # (8)!
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8"
            ) # (9)!
        except CvBridgeError as e:
            self.get_logger().warning(f"{e}")

        if self.waiting_for_image: # (10)!
            height, width, channels = cv_img.shape

            self.get_logger().info(
                f"Obtained an image of height {height}px and width {width}px."
            )

            self.show_image(img=cv_img, img_name="step1_original")

            self.waiting_for_image = False # (15)!
            cv2.destroyAllWindows() # (16)!

    def show_image(self, img, img_name, save_img=True): # (11)!
        
        self.get_logger().info("Opening the image in a new window...")
        cv2.imshow(img_name, img) # (12)!
        
        if save_img: # (13)!
            self.save_image(img, img_name)
        
        self.get_logger().info(
            "IMPORTANT: Close the image pop-up window to exit."
        )
        
        cv2.waitKey(0) # (14)!
    
    def save_image(self, img, img_name): # (17)!
        self.get_logger().info(f"Saving the image...")
        
        base_image_path = Path.home().joinpath("myrosdata/object_detection/")
        base_image_path.mkdir(parents=True, exist_ok=True) # (18)!
        full_image_path = base_image_path.joinpath(
            f"{img_name}.jpg") # (19)!

        cv2.imwrite(str(full_image_path), img) # (20)!
        
        self.get_logger().info(
            f"\nSaved an image to '{full_image_path}'\n"
            f"  - image dims: {img.shape[0]}x{img.shape[1]}px\n"
            f"  - file size: {full_image_path.stat().st_size} bytes"
        ) # (21)!
        
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    while node.waiting_for_image:
        rclpy.spin_once(node) # (22)!
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()