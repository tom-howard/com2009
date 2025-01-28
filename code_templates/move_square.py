import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist # (1)!
from nav_msgs.msg import Odometry # (2)!

from part2_navigation.tb3_tools import quaternion_to_euler # (3)!
from math import sqrt, pow, pi # (4)!

class Square(Node):

    def __init__(self):
        super().__init__("move_square")
        
        self.first_message = False
        self.turn = False 

        self.vel_msg = Twist() # (5)! 
        # (6)!
        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0
        # (7)!
        self.yaw = 0.0 
        self.displacement = 0.0 
        
        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        )

        self.shutdown = False
        
        self.get_logger().info(
            f"The '{self.get_name()}' node is initialised."
        )

    def on_shutdown(self):
        print("Stopping the robot...")
        self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 
        
        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation) # (8)!
        
        self.x = pose.position.x # (9)!
        self.y = pose.position.y
        self.theta_z = abs(yaw) # abs(yaw) makes life much easier!!

        if not self.first_message: # (10)!
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z

    def timer_callback(self):
        # here is where the code to control the motion of the robot 
        # goes. Add code here to make the robot move in a square of
        # dimensions 1 x 1m...
        if self.turn:
            # turn by 90 degrees...
            

        else:
            # move forwards by 1m...
            

        # publish whatever velocity command has been set above:
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    move_square = Square()
    try:
        rclpy.spin(move_square)
    except KeyboardInterrupt:
        print(
            f"{move_square.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        move_square.on_shutdown()
        while not move_square.shutdown:
            continue
        move_square.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()