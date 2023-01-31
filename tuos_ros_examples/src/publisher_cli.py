#!/usr/bin/env python3
"""
A simple ROS publisher node with a Command-line Interface (CLI)

See here for further info:
https://github.com/tom-howard/COM2009/wiki/Launch-Files
"""

import rospy
from std_msgs.msg import String
# Import the "argparse" library:
import argparse

class Publisher():
    
    def __init__(self):
        self.node_name = "publisher_cli"
        topic_name = "chatter"

        self.pub = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz
        
        # Command-Line Interface:
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.node_name}' node.")
        cli.add_argument(
            "-colour",
            metavar="COL",
            default="Blue", 
            help="The name of a colour (for example)"
        )
        cli.add_argument(
            "-number",
            metavar="NUM",
            type=float, 
            default=0.1, 
            help="A value (what happens if NUM > 1?)"
        )
       
        # obtain the arguments passed to this node from the command-line:
        self.args = cli.parse_args(rospy.myargv()[1:])

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The '{self.node_name}' node is active.\n"
                    f"Publishing messages to '/{topic_name}'...")
        
        if self.args.number > 1:
            print(f"'-number' = {self.args.number}, which is GREATER THAN 1, so 'verbose mode' is enabled.")
            self.verbose = True
        else:
            print(f"'-number' = {self.args.number}, which is LESS THAN 1, so I'll keep quiet.")
            self.verbose = False

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            message = f"Searching for something '{self.args.colour}' in the environment."
            self.pub.publish(message)
            if self.verbose:
                print(message)
            self.rate.sleep()

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass