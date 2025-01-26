#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from pathlib import Path

class ObjectDetection(Node):

    def __init__(self):
        super().__init__("object_detection")

        self.camera_sub = self.create_subscription(
            msg_type=Image,
            topic="/camera/image_raw",
            callback=self.camera_callback,
            qos_profile=10
        )

        self.waiting_for_image = True
    
    def camera_callback(self, img_data):
        cvbridge_interface = CvBridge()
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8"
            )
        except CvBridgeError as e:
            self.get_logger().warning(f"{e}")

        if self.waiting_for_image:
            height, width, channels = cv_img.shape

            self.get_logger().info(
                f"Obtained an image of height {height}px and width {width}px."
            )

            self.show_image(img=cv_img, img_name="step1_original")

            crop_width = width - 400
            crop_height = 400
            crop_y0 = int((width / 2) - (crop_width / 2))
            crop_z0 = int((height / 2) - (crop_height / 2))
            cropped_img = cv_img[
                crop_z0:crop_z0+crop_height, 
                crop_y0:crop_y0+crop_width
            ]

            self.show_image(img=cropped_img, img_name="step2_cropping")

            hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
            lower_threshold = (115, 225, 100)
            upper_threshold = (130, 255, 255)
            img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)

            self.show_image(img=img_mask, img_name="step3_image_mask")

            filtered_img = cv2.bitwise_and(cropped_img, cropped_img, mask = img_mask)
            
            # Finding the Image Centroid: (1) 
            m = cv2.moments(img_mask) # (2)!
            cy = m['m10'] / (m['m00'] + 1e-5)
            cz = m['m01'] / (m['m00'] + 1e-5) # (3)!
            cv2.circle(
                filtered_img,
                (int(cy), int(cz)),
                10, (0, 0, 255), 2
            ) # (4)!

            self.show_image(img=filtered_img, img_name="step4_filtered_image")

            self.waiting_for_image = False
            cv2.destroyAllWindows()

    def show_image(self, img, img_name, save_img=True):
        
        self.get_logger().info("Opening the image in a new window...")
        cv2.imshow(img_name, img)
        
        if save_img:
            self.save_image(img, img_name)
        
        self.get_logger().info(
            "IMPORTANT: Close the image pop-up window to exit."
        )
        
        cv2.waitKey(0)
    
    def save_image(self, img, img_name):
        self.get_logger().info(f"Saving the image...")
        
        base_image_path = Path.home().joinpath("myrosdata/object_detection/")
        base_image_path.mkdir(parents=True, exist_ok=True)
        full_image_path = base_image_path.joinpath(
            f"{img_name}.jpg")

        cv2.imwrite(str(full_image_path), img)
        
        self.get_logger().info(
            f"\nSaved an image to '{full_image_path}'\n"
            f"  - image dims: {img.shape[0]}x{img.shape[1]}px\n"
            f"  - file size: {full_image_path.stat().st_size} bytes"
        )
        
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    while node.waiting_for_image:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()