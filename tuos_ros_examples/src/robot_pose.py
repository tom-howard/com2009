#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import pandas as pd
from tf.transformations import euler_from_quaternion

pd.set_option("display.float_format", '{:0.3f}'.format)

class getRobotPose:

    def callback(self, topic_data):
        orientation_x = topic_data.pose.pose.orientation.x
        orientation_y = topic_data.pose.pose.orientation.y
        orientation_z = topic_data.pose.pose.orientation.z
        orientation_w = topic_data.pose.pose.orientation.w

        position_x = topic_data.pose.pose.position.x
        position_y = topic_data.pose.pose.position.y
        position_z = topic_data.pose.pose.position.z

        (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                orientation_y, orientation_z, orientation_w],
                                'sxyz')

        robot_odom = [position_x, position_y, position_z, roll, pitch, yaw]

        if self.startup:
            self.startup = False   
            self.df['initial'] = robot_odom
            self.got_start_data = True
        else:
            self.df['current'] = robot_odom
            if self.got_start_data:
                self.got_all_data = True

    def __init__(self):
        node_name = "robot_odometry_parser"
        rospy.loginfo(f"Initialising the '{node_name}' rospy node")
        
        self.startup = True 
        self.got_start_data = False
        self.got_all_data = False
        
        self.df = pd.DataFrame(index = ['linear_x','linear_y','linear_z',
                          'theta_x(Roll)','theta_y(Pitch)','theta_z(Yaw)'])
        self.df['units'] = ['meters','meters','meters',
                           'radians','radians','radians']
        
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)

        rospy.init_node(node_name)

        self.rate = rospy.Rate(1) # Hz

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            if self.got_all_data:
                self.df['delta'] = self.df['current'] - self.df['initial']
                df = self.df[['initial', 'current', 'delta', 'units']]

                print(f"=========== Rospy Time: {rospy.get_time():>12.1f} ===========")
                print(df)
                
            self.rate.sleep()

if __name__ == '__main__':
    main_instance = getRobotPose()
    try:
        main_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
