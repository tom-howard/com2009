#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion

class getEndPose:

    def callback1(self, topic_data):
        if self.got_start_pose and not self.got_end_pose:
            print("Obtained the robot's end position from the '/odom' topic:\n")
            
            orientation_x = topic_data.pose.pose.orientation.x
            orientation_y = topic_data.pose.pose.orientation.y
            orientation_z = topic_data.pose.pose.orientation.z
            orientation_w = topic_data.pose.pose.orientation.w

            position_x = topic_data.pose.pose.position.x
            position_y = topic_data.pose.pose.position.y
            position_z = topic_data.pose.pose.position.z
            
            print("End position (x, y, z) [in meters]:\n{:.5f}, {:.5f}, {:.5f}.\n".format(
                                                position_x, position_y, position_z))
            print("End orientation (x, y, z, w) [in quaternions]:\n{:.5f}, {:.5f}, {:.5f}, {:.5f}.\n".format(
                                                orientation_x, orientation_y, orientation_z, orientation_w))
            print("Converting orientation from quaternions to angles about the principal axes (roll, pitch and yaw).")
            (roll, pitch, yaw) = euler_from_quaternion([orientation_x, 
                                                        orientation_y, 
                                                        orientation_z, 
                                                        orientation_w],'sxyz')
            print("Orientation (roll, pitch, yaw) [in radians]:\n{:.5f}, {:.5f}, {:.5f}.\n".format(
                                                roll, pitch, yaw))
            
            self.df['end'] = [position_x, position_y, position_z, roll, pitch, yaw]
            
            self.got_end_pose = True
    
    def callback2(self, start_pose):
        if not self.got_start_pose:
            print("Obtained the robot's starting position from the '/odom_start' topic.\n")
            
            self.df['start'] = start_pose.data
            
            self.got_start_pose = True
    
    def __init__(self):
        node_name = "robot_end_position"
        rospy.loginfo("Initialising the '{}' rospy node".format(node_name))
        
        self.got_start_pose = False
        self.got_end_pose = False
        
        import pandas as pd
        pd.set_option("display.precision", 3,
              "display.float_format", '{:0.2f}'.format)
        
        self.df = pd.DataFrame(index = ['linear_x','linear_y','linear_z',
                          'theta_x(Roll)','theta_y(Pitch)','theta_z(Yaw)'])
        self.df['units'] = ['meters','meters','meters',
                           'radians','radians','radians']
        
        rospy.init_node(node_name)
        self.sub1 = rospy.Subscriber("odom", Odometry, self.callback1)
        self.sub2 = rospy.Subscriber("odom_start", Float32MultiArray, self.callback2)
        
    def main(self):
        while not (self.got_start_pose and self.got_end_pose):
            continue
        
        self.df['delta'] = self.df['end'] - self.df['start']
        df = self.df[['start', 'end', 'delta', 'units']]

        print("================= Summary =================")
        print(df)

if __name__ == '__main__':
    subscriber_instance = getEndPose()
    subscriber_instance.main()
