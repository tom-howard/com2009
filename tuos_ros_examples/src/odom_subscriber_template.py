#!/usr/bin/env python3

import rospy
# import the Odometry message from the nav_msgs package:
from nav_msgs.msg import Odometry
# additionally, import the "euler_from_quaternion" function from the tf library
# for converting the raw orientation values from the odometry message into euler angles:
from tf.transformations import euler_from_quaternion

class OdomSubscriber():

    def callback(self, topic_data: Odometry):
        # We're only interested in the pose part of the Odometry message,
        # so we'll extract this bit first:
        pose = topic_data.pose.pose
        # This contains information about both the "position" and "orientation"
        # of the robot, so let's extract those two parts out next:
        position = pose.position
        orientation = pose.orientation 
        # "position" data is provided in meters, so we don't need to do any
        # conversion on this, and can extract the relevant parts of this directly:
        pos_x = position.x
        pos_y = position.y
        pos_z = position.z
        # "orientation" data is in quaternions, so we need to convert this 
        # using the "euler_from_quaternion" function 
        # See here for further details:
        # https://tom-howard.github.io/ros/com2009/la1/week2/#euler_angs

        # Add your code here!

        # Here we print out the values that we're interested in:
        if self.counter > 10:
            self.counter = 0
            print(f"x = {pos_x:.3f} (m), y = ? (m), theta_z = ? (radians)")
        else:
            self.counter += 1

    def __init__(self):
        node_name = "odom_subscriber" # a name for our node (we can call it anything we like)
        rospy.init_node(node_name, anonymous=True)
        # When setting up the subscriber, the "odom" topic needs to be specified
        # and the message type (Odometry) needs to be provided
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        # an optional status message:
        rospy.loginfo(f"The '{node_name}' node is active...")
        
        self.counter = 0       # What's this bit for, do you think?

    def main_loop(self):
        # set the node to remain active until closed manually:
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = OdomSubscriber()
    subscriber_instance.main_loop()