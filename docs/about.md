---
title: "About this Site"
---

# About this Site

## Welcome

This is the home of The ROS2 Course Assignments for **COM2009**: a second-year undergraduate module for [The School of Computer Science](https://www.sheffield.ac.uk/cs). 

This course is designed to teach students how to use ROS2 (The Robot Operating System, Version 2) with TurtleBot3 Waffle robots, using a mix of simulation-based learning and real robot hardware. Most of the initial learning is done in simulation, after which students are able to apply their new-found ROS knowledge to our [real TurtleBot3 Waffle Robots](#robots).

The materials here are developed by [Dr Tom Howard](https://www.sheffield.ac.uk/engineering/diamond-engineering/our-staff/tom-howard), a University Teacher in the [Multidisciplinary Engineering Education Team](https://www.sheffield.ac.uk/engineering/diamond-engineering/about-us) in **The Diamond**.

These resources are *also* used to teach masters-level Mechatronic and Robotic Engineering students in [The School of Electrical and Electronic Engineering](https://www.sheffield.ac.uk/eee) (**ACS6121**). 

## License

![Creative Commons License CC BY-SA](https://i.creativecommons.org/l/by-sa/4.0/88x31.png){ align=left }

This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.

## The TurtleBot3 Waffle {#robots}

### Turtlebot what?!

To teach ROS here we use the [TurtleBot3 Waffle](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot, made by Robotis. This is the 3rd Generation Robot in the [TurtleBot family](http://wiki.ros.org/Robots/TurtleBot) (which has been the reference hardware platform for ROS since 2010). The TurtleBot Robot family exists to provide accessible and relatively low-cost hardware and open-source software on a robot platform, to encourage people to learn robotics and ROS and make it as easy as possible to do so.

### The (Free) TurtleBot3 eBook {#ebook}

The TurtleBot3 Waffle developers (Robotis) have written a book on programming robots with ROS. This is available as a *free eBook*, which you can [download here](https://www.pishrobot.com/wp-content/uploads/2021/05/ros-robot-programming-book-by-turtlebo3-developers-en.pdf), and we recommend that you do so! This is a great resource which provides a detailed introduction to what ROS is and how it works, as well as a comprehensive "Beginners Guide" to ROS programming. The other great thing about this is that it is tailored to the TurtleBot3 Robot specifically, providing examples of how to use a range of TurtleBot3 packages along with a detailed description of how they work.

We recommend that you have a look at this book to learn more about the concepts that you are exploring in this course.

### Our Waffles

Here in the Diamond we have a total of 50 *customised* TurtleBot3 Waffles (aka *"The Waffles"*) specifically for teaching the courses here:

<figure markdown>
  ![](./images/waffle/cabinet.jpg){width=500px} 
</figure>

Our robots are an enhanced version of the *TurtleBot3 WafflePi* that you can buy from Robotis. We've made a few adjustments, as shown below:

<figure markdown>
  ![](./images/waffle/features.png){width=800px}
</figure>

The Waffles have the following core hardware elements:

* An OpenCR Micro-Controller Board to power and control the wheel motors, distribute power to other hardware elements and provide an interface for additional sensors.
* An [UP Squared Single-Board Computer (SBC)](https://up-board.org/upsquared/specifications/) with an Intel Processor and 32GB of on-board eMMC storage. This board acts as the "brain" of the robot.
* Independent left and right wheel motors (DYNAMIXEL XM430’s) to drive the robot using a *differential drive* configuration.

This drive configuration allows the robots to move with the following **maximum velocities**: <a name="max_vels"></a>

<center>

| Velocity Component | Upper Limit | Units |
| :--- | :---: | :--- |
| *Linear* | 0.26 | m/s |
| *Angular* | 1.82 | rad/s |

</center>

In addition to this, the robots are equipped with the following sensors:

* A Light Detection and Ranging (or *LiDAR*) sensor, which spins continuously when the robot is in operation. This uses light in the form of laser pulses to allow the robot to measure the distance to surrounding objects, providing it with a 360&deg; view of its environment.
* An [Intel RealSense D435 Camera](https://www.intelrealsense.com/depth-camera-d435/) with left and right imaging sensors, allowing depth sensing as well as standard image capture.
* A 9-Axis Inertial Measurement Unit (or *IMU*) on-board the OpenCR Micro Controller board, which uses an accelerometer, gyroscope and magnetometer to measure the robot's specific force, acceleration and orientation. 
* Encoders in each of the DYNAMIXEL wheel motors, allowing measurement of speed and rotation count for each of the wheels.

#### Software

Our robots currently run [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) (or *"Humble"* for short). The courses here are therefore based around this version of ROS. The easiest way to install Humble is via Deb packages for Ubuntu Jammy (22.04). This is the setup we recommend and - as such - all out robotics hardware runs with this OS/Software setup.

To deliver the simulation-based parts of this course, we've created a custom simulation environment using the [Windows Subsystem for Linux (WSL)](https://docs.microsoft.com/en-us/windows/wsl/). This has been developed primarily to run on University of Sheffield Managed Desktop Computers, which run Windows 10, but it's also possible to run this on other machines too. We call this simulation environment *"WSL-ROS2"*. [See here for more details](./ros/wsl-ros/README.md).

[You can find out more about installing ROS on your own system here](./ros/README.md).

#### Laptops

In the Diamond, we have dedicated Robot Laptops running the same OS & ROS version as above. We use these when working with the robots in the lab. [See here for more details](./waffles/intro.md#laptops). 

## Version History

### Iteration 1

**Academic Year**: 2024-25 

* This course [used to live here](https://tom-howard.github.io/ros/), and was based on ROS 1 Noetic. This year, we've upgraded everything to ROS 2, re-written the courses and moved everything over to this new site. [See the previous version history here](https://tom-howard.github.io/ros/about/changelog/).
* Other notable changes:
    * COM2009 Assignment #2 now only involves three programming tasks (where previously it was four), with an additional assessment on documentation now included instead.
