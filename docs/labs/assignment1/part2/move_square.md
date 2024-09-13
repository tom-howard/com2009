---  
title: "Odometry-based Navigation (Move Square)"  
---

**A combined publisher-subscriber node to achieve odometry-based control...**

Below you will find a template Python script to show you how you can both publish to `/cmd_vel` and subscribe to `/odom` in the same node.  This will help you build a *closed-loop* controller to make your robot follow a **square motion path** of size: **1m x 1m**. 

You can publish velocity commands to `/cmd_vel` to make the robot move, monitor the robot's *position* and *orientation* in real-time, determine when the desired movement has been completed, and then update the velocity commands accordingly.  

## Suggested Approach

Moving in a square can be achieved by switching between two different movement states sequentially: *Moving forwards* and *turning* on the spot. At the start of each movement step we can read the robot's current odometry, and then use this as a reference to compare to, and to tell us when the robot's position/orientation has changed by the required amount, e.g.:

1. With the robot stationary, **read the odometry** to determine its current X and Y position in the environment.
1. **Move forwards** until the robot's X and Y position indicate that it has moved linearly by 0.5m.
1. **Stop** moving forwards.
1. **Read the robot's odometry** to determine its current orientation ("yaw"/<code>&theta;<sub>z</sub></code>).
1. **Turn on the spot** until the robot's orientation changes by 90&deg;.
1. **Stop** turning.
1. Repeat.  

### The Code

```python title="move_square.py"
--8<-- "code_templates/move_square.py"
```

1. Import the `Twist` message for publishing velocity commands to `/cmd_vel`.
2. Import the `Odometry` message, for use when subscribing to the `/odom` topic.
3. Import the `quaternion_to_euler` function from `tb3_tools.py` to convert orientation from quaternions to Euler angles (about [the principal axes](../part2.md#principal-axes)).
4. Finally, import some useful mathematical operations (and `pi`), which may prove useful for this task:

    <center>

    | Mathematical Operation | Python Implementation |
    | :---: | :---: |
    | $\sqrt{a+b}$ | `#!python sqrt(a+b)` |
    | $a^{2}+(bc)^{3}$ | `#!python pow(a, 2) + pow(b*c, 3)` |
    | $\pi r^2$ | `#!python pi * pow(r, 2)` |

    </center>

5. Here we establish a `Twist` message, which we can populate with velocities and then publish to `/cmd_vel` within the `timer_callback()` method (in order to make the robot move).

6. Here, we define some variables that we can use to store relevant bits of odometry data while our node is running (and read it back to implement feedback control):
    * `self.x`, `self.y` and `self.theta_z` will be used by the `odom_callback()` to store the robot's **current** pose
    * `self.x0`, `self.y0` and `self.theta_z0` can be used in the `timer_callback()` method to keep a record of where the robot **was** at a given moment in time (and determine how far it has moved since that point)

7. We'll also need to keep track of how far the robot has travelled (or turned) in order to determine when sufficient movement has taken place to trigger a switch to the alternative state, i.e.:
    
    * `if` travelled 1 meter, `then`: turn
    * `if` turned 90&deg;, `then`: move forward

8. Here we obtain the robot's current orientation (in quaternions) and convert it to Euler angles (in radians) about [the principal axes](../part2.md#principal-axes), where:
    * "roll" = <code>&theta;<sub>x</sub></code>
    * "pitch" = <code>&theta;<sub>y</sub></code>
    * "yaw" = <code>&theta;<sub>z</sub></code>

9. We're only interested in `x`, `y` and <code>&theta;<sub>z</sub></code>, so we assign these to class variables `self.x`, `self.y` and `self.theta_z`, so that we can access them elsewhere within our `Square()` class.

10. Sometimes, it can take a few moments for the first topic message to come through, and it's useful to know when that's happened so that you know you are dealing with actual topic data! Here, we're just setting a flag to `True` once the callback function has executed for the first time (i.e. the first topic message *has* been received).
 
## Alternative Approach: Waypoint Tracking

A square motion path can be fully defined by the coordinates of its four corners, and we can make the robot move to each of these corners one-by-one, using its odometry system to monitor its real-time position, and adapting linear and angular velocities accordingly.

This is slightly more complicated, and you might want to wait until you have a bit more experience with ROS before tackling it this way.

<p align="center">
  <a href="../../part2#move_square_ret">&#8592; Back to Part 2</a>
</p>