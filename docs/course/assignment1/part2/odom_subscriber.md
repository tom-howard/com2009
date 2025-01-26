---  
title: "An Odometry Subscriber Node"
---

## The Initial Code

Having copied the `subscriber.py` file from your `part1_pubsub` package, you'll start out with [the code discussed here](../part1/subscriber.md).

Let's look at what we need to change now.

## From Simple Subscriber to Odom Subscriber

### Imports

We will generally rely on `rclpy` and the `Node` class from the `rclpy.node` library, for most nodes that we will create, so our first two imports will remain the same. 

We won't be working with `String` type messages any more however, so we need to replace this line in order to import the correct message type. As we know from [earlier in Part 2](../part2.md#odometry-explained), the `/odom` topic uses messages of the type `nav_msgs/msg/Odometry`:

``` { .bash .no-copy }
$ ros2 topic info /odom
Type: nav_msgs/msg/Odometry
...
```

This tells us everything we need to know to construct the Python import statement correctly:

```py
from nav_msgs.msg import Odometry
```

We'll also need to import a handy function that should already exist as an importable module in your `part2_navigation` package called `tb3_tools`:

```py
from part2_navigation.tb3_tools import quaternion_to_euler
```

As the name suggests, we'll use this to convert the raw orientation values from `/odom` into their Euler Angle representation.

??? info
    This module can be found here: `part2_navigation/part2_navigation/tb3_tools.py`, if you want to have a look.

### Change the Class Name

Previously our class was called `#!python SimpleSubscriber()`, change this to something more appropriate now, e.g.: `#!python OdomSubscriber()`:

```py
class OdomSubscriber(Node):
```

### Initialising the Class

The structure of this remains largely the same, we just need to modify a few things: 

1. Change the name that is used to register the node on the ROS Network:

    ```python
    super().__init__("odom_subscriber")
    ```

1. Change the subscription parameters:

    ```python
    self.my_subscriber = self.create_subscription(
        msg_type=Odometry, # (1)!
        topic="odom", # (2)!
        callback=self.msg_callback, 
        qos_profile=10,
    )
    ```

    1. `/odom` uses the Odometry message type (as imported above)
    2. The topic name is `"odom"`, of course!

1. The final thing we'll do inside our class' `__init__` method (after we've set up the subscriber) is initialise a counter:

    ```py
    self.counter = 0 
    ```

    The reason for this will be explained shortly...

### Modifying the Message Callback

This is where the changes are a bit more significant:

```py
def msg_subscriber(self, topic_message: Odometry): # (1)!

    pose = topic_data.pose.pose # (2)!
    
    # (3)!
    pos_x = pose.position.x
    pos_y = pose.position.y
    pos_z = pose.position.z
    
    roll, pitch, yaw = quaternion_to_euler(pose.orientation) # (4)!

    if self.counter > 10: # (5)!
        self.counter = 0
        self.get_logger().info(
            f"x = {pos_x:.3f} (m), y = ? (m), theta_z = ? (radians)"
        ) # (6)!
    else:
        self.counter += 1

```

1. When defining the message callback, modify the *type annotation* for the `topic_message` input.
2. There are [two key parts to an odometry message](../part2.md#odom-base-fields): Pose and Twist.

    We're only really interested in the Pose part of the message here, so grab this first.

3. As we know by now, Pose contains information about both the "position" and "orientation" of the robot, we extract the position values first and assign them to the variables `pos_x`, `pos_y` and `pos_z`.
    
    Position data is provided in meters, so we don't need to do any conversion on this and can use the data directly.

4. Orientation data is in quaternions, so we convert this by passing it to the `quaternion_to_euler` function that we imported from `tb3_tools` earlier.

    This function provides us with the orientation of the robot about its 3 principal axes:

    * <code>&theta;<sub>x</sub></code>: "Roll"
    * <code>&theta;<sub>y</sub></code>: "Pitch"
    * <code>&theta;<sub>z</sub></code>: "Yaw"

5. Here we print out the values that we're interested in to the terminal.

    This callback function will execute every time a new message is published to the `odom` topic, which occurs at a rate of around 20 times per second (20 Hz).
        
    ??? tip
        We can use he `ros2 topic hz` function to tell us this:

        ``` { .txt .no-copy }
        $ ros2 topic hz /odom
        average rate: 18.358
        min: 0.037s max: 0.088s std dev: 0.01444s window: 20
        ``` 
    
    That's a lot of messages to be printed to the terminal every second! We therefore use an `if` statement and a `counter` to ensure that our `print` statement only executes for 1 in every 10 topic messages instead.

6. **Task**: Continue formatting the `print` message to display the three odometry values that are relevant to our robot!  

### Updating "Main"

The only thing left to do now is update any relevant parts of the `main` function to ensure that you are instantiating, spinning and shutting down your node correctly.

<p align="center">
  <a href="../../part2#odom_sub_ret">&#8592; Back to Part 2</a>
</p>