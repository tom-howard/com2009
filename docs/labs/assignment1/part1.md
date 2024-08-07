---
title: "Part 1: Getting Started with ROS2" 
description: In the first part of this lab course you will learn the basics of ROS and become familiar with some key tools and principles of this framework, allowing you to program robots and work with ROS applications effectively. 
---

## Introduction

:material-pen: **Exercises**: X  
:material-timer: **Estimated Completion Time**: Y hours

### Aims

In the first part of this lab course you will learn the basics of ROS and become familiar with some key tools and principles of the framework which will allow you to program robots and work with ROS applications effectively.  For the most part, you will interact with ROS using the *Linux command line* and so you will also become familiar with some key Linux command line tools that will help you.  Finally, you will learn how to create some basic ROS Nodes using Python and get a taste of how ROS topics and messages work.

### Intended Learning Outcomes

By the end of this session you will be able to:  

1. Control a TurtleBot3 Robot, in simulation, using ROS.
1. Launch ROS applications using `roslaunch` and `rosrun`.
1. Interrogate running ROS applications using key ROS command line tools.
1. Create a ROS package comprised of multiple nodes and program these nodes (in Python) to communicate with one another using ROS Communication Methods.
1. Navigate a Linux filesystem and learn how to do various filesystem operations from within a Linux Terminal.

## First Steps

**Step 1: Accessing a ROS2 Environment for this Course**

If you haven't done so already, see here for all the details on [how to install or access a ROS environment for this course (TODO)]().

**Step 2: Launch ROS**

Launch your ROS environment.

1. OPTION 1
1. OPTION 2
1. etc...

Either way, you should now have access to ROS via a Linux terminal instance, and we'll refer to this terminal instance as **TERMINAL 1**.

**Step 3: Download The Course Repo**

<a name="course-repo"></a>

We've put together a few ROS packages specifically for this course. These all live within [this GitHub repo](https://github.com/tom-howard/tuos_ros), and you'll need to download and install this into your ROS environment now, before going any further.

1. In **TERMINAL 1**, create a *"ROS2 Workspace"* in your home directory[^ros2_ws]:

    [^ros2_ws]: What is a ROS2 Workspace? [You can find out more here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#background). 

    ```bash
    mkdir -p ~/ros2_ws/src/
    ```

1. Navigate into this using the `cd` Linux command:

    ```bash
    cd ~/ros2_ws/src/
    ```

1. Then, run the following command to clone the Course Repo from GitHub:

    ***
    **TERMINAL 1:**
    ```bash
    git clone https://github.com/tom-howard/tuos_ros.git -b humble
    ```
    
    [WIP: once complete, the `-b humble` bit shouldn't be needed]

    ***

1. Once this is done, you'll need to build this using a tool called *"Colcon"*[^colcon]:

    [^colcon]: What is **Colcon**? [Find out more here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#background).

    ***
    **TERMINAL 1:**
    ```bash
    cd ~/ros2_ws/ && colcon build
    ```
    ***

1. Add a line to the bashrc to source this overlay...

    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```

    Don't worry too much about what you just did, for now. We'll cover this in more detail throughout the course. That's it for now though, we'll start using some of the packages that we've just installed a bit later on..