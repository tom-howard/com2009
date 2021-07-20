#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

class getStartPos:

    def callback(self, topic_data):
        if self.startup:
            self.startup = False            
            
            print("Obtained the robot's starting position from the '/odom' topic:\n")
            
            orientation_x = topic_data.pose.pose.orientation.x
            orientation_y = topic_data.pose.pose.orientation.y
            orientation_z = topic_data.pose.pose.orientation.z
            orientation_w = topic_data.pose.pose.orientation.w

            position_x = topic_data.pose.pose.position.x
            position_y = topic_data.pose.pose.position.y
            position_z = topic_data.pose.pose.position.z

            print("Start position (x, y, z) [in meters]:\n{:.5f}, {:.5f}, {:.5f}.\n".format(
                                                position_x, position_y, position_z))
            print("Start orientation (x, y, z, w) [in quaternions]:\n{:.5f}, {:.5f}, {:.5f}, {:.5f}.\n".format(
                                                orientation_x, orientation_y, orientation_z, orientation_w))
            print("Converting orientation from quaternions to angles about the principal axes (roll, pitch and yaw).")
            (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                                        orientation_y, 
                                                        orientation_z, 
                                                        orientation_w],'sxyz')
            print("Orientation (roll, pitch, yaw) [in radians]:\n{:.5f}, {:.5f}, {:.5f}.\n".format(
                                                roll, pitch, yaw))
                                                
            print("Publishing this as an array to a new topic called '/odom_start'...")            
            self.odom_array.data = [position_x, position_y, position_z, roll, pitch, yaw]

    def __init__(self):
        node_name = "robot_start_position"
        rospy.loginfo("Initialising the '{}' rospy node".format(node_name))
        
        self.startup = True 
        
        self.odom_array = Float32MultiArray()
        
        self.pub = rospy.Publisher('odom_start', Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)

        rospy.init_node(node_name)

        self.rate = rospy.Rate(1) # Hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            self.pub.publish(self.odom_array)
            self.rate.sleep()

if __name__ == '__main__':
    main_instance = getStartPos()
    try:
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
