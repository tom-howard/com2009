---  
title: Further (Essential) Exercises 
---

By working through the additional short exercises below, you will become aware of the key differences in how our TurtleBot3 Waffles work in the real world, compared to how they work in simulation. Be mindful of the differences that we are trying to highlight here and the implications that they will have on the applications that you develop. 

As a general rule, when developing code for real-world applications, it's a good idea to get the basics working in simulation first, but **always** test out your code in the real-world... *just because it works in simulation, **doesn't** automatically mean it will work on the real robots!!*

## :material-pen: Essential Exercise 1: Publishing Velocity Commands {#ex1}

This one actually applies to both the real Waffles *and* the simulation too. The issue is more critical when working with real thing though, since we're working with real hardware in a real-world environment, where things could get damaged or people could get hurt. 

In the previous exercises you made the robot move using the `teleop_keyboard` node, and also built a Python node to control the robot's velocity. Ultimately, making a robot move is achieved by publishing velocity commands (i.e. `geometry_msgs/msg/Twist` interface messages) to the `/cmd_vel` topic. Another way to do this is by using the `ros2 topic pub` command in a terminal. Run the following command and observe what the robot does:

```txt
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

... the robot should turn on the spot.

Enter ++ctrl+c++ now to stop the `ros2 topic pub` command, what happens now?

... the robot *continues* to turn on the spot!

In order to actually *stop* the robot, we need to run the command again, but this time set all the velocities back to zero:

```txt
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Why Does This Matter?

If we don't issue a stop command to a robot that is moving, then it will continue to move with the last velocity command that was sent to it. 

The same applies to ROS nodes when we shut them down: if a stop command is not sent to `cmd_vel` before the node stops running (i.e. via ++ctrl+c++) the robot will continue to move, which clearly isn't very good!

***You must ensure*** that your nodes are programmed with correct shutdown procedures, to ensure that the robot actually stops moving when a node is terminated. This is covered in [Part 2 of Assignment #1](../course/assignment1/part2.md#ex5), but you can also see this in action in [the Velocity Control Node that we created in the previous section](./basics.md#timedSquareCode). 

## :material-pen: Essential Exercise 2: Out of Range LiDAR Data {#ex2}

The robot's LiDAR sensor can only obtain measurements from objects within a certain distance range. In Part 3 we look at how to work out what this range is, using the `ros2 topic echo` command. Let's apply the same techniques to the real robot now to discover the **maximum** and **minimum** distances that the real robot's LiDAR sensor can measure:

```
ros2 topic echo /scan --field range_min --once
```
```
ros2 topic echo /scan --field range_max --once
```

The LiDAR sensor's measurement range is the same in simulation and on the real robots, **but** how out-of-range values are *reported* is different!

If the LiDAR sensor detects an object that falls within this range then it will report the exact distance to this object (in meters). Conversely, if it *doesn't* detect anything within this range then it will report a default *out-of-range* value instead. In simulation, the out-of-range value is `inf`.

!!! warning
    The *out-of-range* value reported by the real robot's LiDAR sensor is **not** `inf`!

Use the `ros2 topic echo` command to view the raw LiDAR data:

```
ros2 topic echo /scan --field ranges
```

See if you can position the robot in the environment so that some LiDAR measurements will fall outside the measurement range that you have just determined. How are these values reported in the terminal? 

### Why Does This Matter?

In Part 3 of Course Assignment #1 we illustrate how the LiDAR `ranges` array can be *filtered* to remove any out-of-range values: 

```py
valid_data = front[front != float("inf")] # (1)!
```

1. Assuming `front` is a `numpy` array (see [Part 3](../course/assignment1/part3.md)).

If you apply the same technique to a real-world application, then this filtering will be ineffective, because out-of-range values are not `inf` here... You'll need to adapt this for real-world scenarios[^lidar-hint].

[^lidar-hint]: **Exercise 2 Hint**: Out-of-range values on the real robots are actually reported as `0.0`!

## :material-pen: Essential Exercise 3: The Camera Image Topic {#ex3}

In Part 6 of Assignment #1 we work extensively with the robot's camera and the processing of the images that are captured by it. This is done in simulation (of course), where the image data is published to a topic called `/camera/image_raw`. The name of the camera image topic is **not the same** on the real robots!

With the real robot to hand now, use ROS command-line tools such as `ros2 topic list` and `ros2 topic info` to interrogate the real robot ROS Network and identify the name of the camera image topic used by the real robot.

### Why Does This Matter?

You'll likely do a lot of development work for your real-robot applications in simulation, outside the lab sessions. Some of these applications may involve camera data and image processing. If you set up a subscriber to a topic that doesn't exist, then **ROS will not warn you about it**! It will simply sit quietly and wait, assuming that the topic will (sooner or later) become available. As a result, if you are running an application on a real robot, that is subscribing to image data on the `/camera/image_raw` topic, then your application will never receive any image data and any associated callback functions will never execute![^cam-topic-hint]

[^cam-topic-hint]: **Exercise 3 Hint**: On the real robots, the camera image topic is `/camera/color/image_raw`!

## :material-pen: Essential Exercise 4: Camera Image Resolution {#ex4}

In Part 6 of Assignment #1 we also explore how images can be reduced in size by *cropping* them, to make the data processing a bit quicker and easier. We need to be aware of the original image size (i.e. *resolution*) however, in order to apply cropping techniques appropriately. 

As we learn in Part 6, it's possible to discover the resolution of each image that is published to the camera image topic by echoing the `height` and `width` parameters that are published as part of the image message interface (alongside the image data itself). We can of course do this now with `ros2 topic echo`, to determine the resolution of the camera images obtained on the real robot:

```
ros2 topic echo /camera/color/image_raw --field height --once
```

```
ros2 topic echo /camera/color/image_raw --field width --once
```

The outputs here will indicate the `height` and `width` of the images, in pixels. See how these compare with values that you obtain from the simulation, when you get to Part 6 of the Course.

!!! warning 
    The real robot's camera captures images at **a lower image resolution**! 

### Why Does This Matter?

We'll learn a lot about image cropping and other image processing techniques through simulation, but (as above) the native image resolution of the simulated robot's camera is *much larger* than that of the real robot. As such, if you apply the same cropping techniques to real-world applications, without adjustment, then you will end up cropping too much of the image out, leaving *nothing* to actually apply any further processing to![^img-res]

[^img-res]: **Exercise 4 Hint**: In *simulation*, camera images have a resolution of **1080x1920** pixels, whereas on the real robots the resolution is **640x480** pixels.

## :material-pen: Essential Exercise 5: Object Detection {#ex5}

In general, image detection gets a little more challenging in the real-world, where the same object might appear (to a robot's camera) to have slightly different colour tones under different light conditions, from different angles, in different levels of shade, etc. In simulation (again in Part 6 of the Course), you may build an extremely effective `colour_search.py` node to detect each of the four coloured pillars in the `tuos_simulations/coloured_pillars` world, but this might not perform as well in the real world without some fine-tuning

### Why Does This Matter?

**Always** test out your code in the real-world, just because it works in simulation, **doesn't** mean it will work on the real robots!!

## Summary

You will naturally do a fair bit of development work in simulation, where it's easier to test things out and less disastrous if things go wrong! Overall, you'll be able to develop things much faster this way, and you can do this outside of your weekly lab sessions too. Whilst you're doing this though, keep in mind all the differences that we have identified above, so that there are less nasty surprises when you come to deploy your ROS applications in the real world. 

Throughout the design phase, think about how your applications could be developed more flexibly to accommodate these variations, or how you could design things so that only small/quick changes/switches need to be made when you transition from testing in simulation, to deploying on a real Waffle. 