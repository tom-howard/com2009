#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import CameraSweepFeedback, CameraSweepResult, CameraSweepAction
from sensor_msgs.msg import CompressedImage

# Import some helper functions from the tb3.py module within this package
from tb3 import Tb3Move, Tb3Odometry

# Import some other useful Python Modules
from math import radians
import datetime as dt
from pathlib import Path

class camerasweepActionServer():
    feedback = CameraSweepFeedback() 
    result = CameraSweepResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/camera_sweep_action_server", 
            CameraSweepAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw/compressed",
            CompressedImage, self.camera_callback)
        self.cv_image = CvBridge()

        self.robot_controller = Tb3Move()
        self.robot_odom = Tb3Odometry()
    
    def camera_callback(self, img):
        image_to_capture = self.cv_image.compressed_imgmsg_to_cv2(img, desired_encoding="passthrough")
        self.current_camera_image = image_to_capture
    
    def action_server_launcher(self, goal):

        success = True
        if goal.sweep_angle <= 0 or goal.sweep_angle > 180:
            print("Invalid sweep_angle! Select a value between 1 and 180 degrees.")
            success = False
        
        if goal.image_count <=0:
            print("I can't capture a negative number of images!")
            success = False
        elif goal.image_count > 50:
            print("Woah, too many images! I can do a maximum of 50.")
            success = False

        if not success:
            self.result.image_path = "None [ABORTED]"
            self.actionserver.set_aborted(self.result)
            return

        # calculate the angular increments over which to capture images:
        ang_incs = goal.sweep_angle/float(goal.image_count)
        # and the time it will take to perform the action:
        turn_vel = 0.2 # rad/s
        full_sweep_time = radians(goal.sweep_angle)/abs(turn_vel)

        print(f"\n#####\n"
            f"The 'camera_sweep_action_server' has been called.\n"
            f"Goal: capture {goal.image_count} images over a {goal.sweep_angle} degree sweep...\n\n"
            f"An image will therefore be captured every {ang_incs:.3f} degrees,\n"
            f"and the full sweep will take {full_sweep_time:.5f} seconds.\n\n"
            f"Commencing the action...\n"
            f"#####\n")
        
        # set the robot velocity:
        self.robot_controller.set_move_cmd(0.0, turn_vel)
        
        # Get the current robot odometry (yaw only):
        ref_yaw = self.robot_odom.yaw

        # Get the current date and time and create a timestamp string of it
        # (to use when we construct the image filename):
        start_time = dt.datetime.strftime(dt.datetime.now(),'%Y%m%d_%H%M%S')
        self.base_image_path = Path.home().joinpath(f"myrosdata/action_examples/{start_time}/")
        self.base_image_path.mkdir(parents=True, exist_ok=True)
        self.result.image_path = str(self.base_image_path).replace(str(Path.home()), "~")
        
        i = 0
        while i < goal.image_count:
            self.robot_controller.publish()
            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo("Cancelling the camera sweep.")

                self.result.image_path = f"{self.result.image_path} [PRE-EMPTED]"                
                self.actionserver.set_preempted(self.result)
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            
            if abs(self.robot_odom.yaw - ref_yaw) >= ang_incs:
                # increment the image counter
                i += 1
                
                # populate the feedback message and publish it:
                rospy.loginfo(f"Captured image {i}")
                self.feedback.current_image = i
                self.feedback.current_angle = abs(self.robot_odom.yaw)
                self.actionserver.publish_feedback(self.feedback)

                # update the reference odometry:
                ref_yaw = self.robot_odom.yaw

                # save the most recently captured image:
                cv2.imwrite(str(self.base_image_path.joinpath(f"img{i:03.0f}.jpg")), 
                    self.current_camera_image)
        
        if success:
            rospy.loginfo("Camera sweep completed successfully.")
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node("camera_sweep_action_server")
    camerasweepActionServer()
    rospy.spin()
