---  
title: "Part 3: SLAM and Autonomous Navigation "  
description: Explore the LiDAR sensor, the data that it generates, and how this can be of huge benefit for robotics applications. You'll see this in practice by leveraging the mapping and autonomous navigation tools within ROS 2. 
---
## Introduction

:material-pen: **Exercises**: X

:material-timer: **Estimated Completion Time**: Y

### Aims

From the work you did in Part 2 you may have started to appreciate the limitations associated with using odometry data alone as a feedback signal when trying to control a robot's position in its environment. In this next part you will explore an alternative data-stream that could be used to aid navigation further. You will leverage some existing ROS 2 libraries and TurtleBot3 packages to explore some really powerful **mapping** and **autonomous navigation** methods that are available within ROS.

### Intended Learning Outcomes

By the end of this session you will be able to:

//add to the ILOs// 

1. Interpret the data that is published to the `/scan` topic and use existing ROS tools to visualise this.
1. Use existing ROS tools to implement SLAM and build a map of an environment. 
1. Leverage existing ROS libraries to make a robot navigate an environment *autonomously*, using the map that you have generated.
1. Explain how these SLAM and Navigation tools are implemented and what information is required in order to make them work.

### Quick Links

* [Exercise 1: Using RViz to Visualise Robot Data](#ex1)
* [Exercise 2: Building a map of an environment with SLAM](#ex2)
* [Exercise 3: Navigating an Environment Autonomously](#ex3)

## Getting Started

**Step 1: Launch your ROS Environment**

If you haven't done so already, launch your ROS environment now:

1. Option 1 
1. Option 2
1. Option 3

**Step 2: Make Sure The Course Repo is Up-To-Date**

In Part 1 you should have [downloaded and installed The Course Repo](./part2.md#getting-started) into your ROS environment. Hopefully you've done this by now, but if you haven't then go back and do it now (you'll need it for some exercises here). If you *have* already done it, then (once again) it's worth just making sure it's all up-to-date, so run the following command now to do so:

***
**TERMINAL 1:**
```bash
cd ~/ros2_ws/src/tuos_ros/ && git pull
```

Then build with Colcon: 

```bash
cd ~/ros2_ws/ && colcon build
```

And finally, re-source your environment:

```bash
source ~/.bashrc
```
***

!!! warning
    If you have any other terminal instances open, then you'll need run `source ~/.bashrc` in these too, in order for any changes made by the Colcon build process to propagate through to these as well.

## Laser Displacement Data and The LiDAR Sensor {#lidar}

As you'll know from Part 2, odometry is really important for robot navigation, but it can be subject to drift and accumulated error over time.

#### :material-pen: Exercise 1: Using RViz to Visualise Robot Data {#ex1}

<a name="rviz"></a>We're now going to place the robot in a more interesting environment than the "empty world" we've used in the previous parts of this course so far...

1. In **TERMINAL 1** enter the following command to launch this:

    //need to check//
    ```bash
    ros2 launch turtlebot3_gazebo x.launch.py
    ```
    A Gazebo simulation should now be launched with a TurtleBot3 Waffle in a new environment:


1. Open a new terminal instance (**TERMINAL 2**) and enter the following:

    ***
    **TERMINAL 2:**
    ```bash
    ros2 launch tuos_simulations rviz.launch
    ```
    ***
    
    On running the command a new window should open:

    This is *RViz*, which is a ROS tool that allows us to *visualise* the data being measured by a robot in real-time. The red dots scattered around the robot represent *laser displacement data* which is measured by the LiDAR sensor located on the top of the robot.  This data allows the robot to measure the distance to any obstacles in its immediate surroundings. The LiDAR sensor spins continuously, sending out laser pulses as it does so. These laser pulses then bounce off any objects and are reflected back to the sensor. Distance can then be determined based on the time it takes for the pulses to complete the full journey (from the sensor, to the object, and back again), by a process called *"time of flight"*. Because the LiDAR sensor spins and performs this process continuously, a full 360&deg; scan of the environment can be generated.  In this case (because we are working in simulation here) the data represents the objects surrounding the robot in its *simulated environment*, so you should notice that the red dots produce an outline that resembles the objects in the world that is being simulated in Gazebo (or partially at least).
