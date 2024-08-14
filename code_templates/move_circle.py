#!/usr/bin/env python3

import rclpy # (1)
from rclpy.node import Node

# (2)

class Circle(Node):

    def __init__(self):
        super().__init__("") # (3)

        self.publisher = self.create_publisher() # (4)
        rate = # (5)
        self.timer = self.create_timer(1/rate, self.timer_callback)

        self.shutdown = False
    
    def on_shutdown(self):
        # (6)
        
        self.shutdown = True

    def timer_callback(self):
        # (7)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=rclpy.signals.SignalHandlerOptions.NO
    )
    
    move_circle = # (8)
    
    try: # (9)
        rclpy.spin(move_circle)
    except KeyboardInterrupt:
        print("Shutdown requested with Ctrl+C")
    finally:
        move_circle.on_shutdown()
        
        while not move_circle.shutdown:
            continue

        move_circle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
