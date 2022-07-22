#!/usr/bin/env python3

import rospy
import rosnode
import datetime as dt

core_nodes = [
    '/turtlebot3_core', 
    '/turtlebot3_lds', 
    '/rosout', 
    '/camera/realsense2_camera', 
    '/camera/realsense2_camera_manager', 
    '/turtlebot3_diagnostics',
]

class tb3Status():
    
    def __init__(self):
        self.node_name = "tb3_status"
        self.startup = True
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1) # hz
                
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 

        timestamp = rospy.get_time()
        self.starttime = rospy.get_time()
        while (rospy.get_time() - timestamp) < 10:
            continue
        
        self.startup = False

    def shutdownhook(self):
        self.ctrl_c = True

    def main(self):
        timestamp = rospy.get_time()
        while self.startup:
            continue
        
        while not self.ctrl_c:
            if (rospy.get_time() - timestamp) > 10:
                active_nodes = rosnode.get_node_names()
                ts = dt.datetime.now().strftime("%d/%m/%Y %H:%M:%S")
                if all([i in active_nodes for i in core_nodes]):
                    rts = rospy.get_time() - self.starttime
                    if rts > 60:
                        runtimestring = f"{rts / 60:5.1f}"
                        minsecs="minutes"
                    else:
                        runtimestring = f"{rts:5.0f}"
                        minsecs="seconds"
                    print(
                        f"[{ts}] Waffle Status:    OK\n"
                        f"                       Nodes Active: {len(active_nodes):5d}\n"
                        f"                  Up Time ({minsecs}): {runtimestring}"
                        )
                else:
                    print(
                        f"[{ts}] Waffle Status: ERROR\n"
                        f"Core node(s) not running!"
                        )
                timestamp = rospy.get_time()
            
            self.rate.sleep()

if __name__ == '__main__':
    instance = tb3Status()
    try:
        instance.main()
    except rospy.ROSInterruptException:
        pass