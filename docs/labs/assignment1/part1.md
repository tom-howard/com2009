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
    cd ~/ros2_ws/ && colcon build --packages-up-to tuos_ros
    ```
    ***

1. Add a line to the bashrc to source this overlay...

    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```

    Don't worry too much about what you just did, for now. We'll cover this in more detail throughout the course. That's it for now though, we'll start using some of the packages that we've just installed a bit later on..






## Creating Your First ROS Applications

Shortly you will create some simple publisher and subscriber nodes in Python and send simple ROS messages between them. As we learnt earlier though, ROS applications must be created within *packages*, and so we need to create a package first in order to start creating our own ROS nodes. 

The `ros2` Command Line Interface (CLI) includes a tool to create a new ROS packages: `ros2 pkg create`.

It's important to work in a specific filesystem location when we create and work on our own ROS packages. These are called *"Workspaces"* and you should already have one ready to go within your local ROS environment[^workspaces]:

``` { .bash .no-copy }
~/ros2_ws/
```

!!! note 
    `~` is an alias for your home directory. So `cd ~/ros2_ws/` is the same as typing `cd /home/{your username}/ros2_ws/`.

[^workspaces]: [You can learn more about ROS2 Workspaces here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#background). 

#### :material-pen: Exercise XX: Creating your own ROS Package {#exx}

We'll be using Python throughout this course, and [The Official ROS2 Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) suggests a particular approach that should be used to create ROS2 packages for Python specifically. We are opting for [a slightly different approach here](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/) however, that will provide us with a little more flexibility and ease of use (particularly for things we'll do later on in the Assignment #1 course and in Assignment #2).   

1. Navigate into the `ros2_ws` folder by using the Linux `cd` command (**c**hange **d**irectory). In **TERMINAL 1** enter the following:

    ***
    **TERMINAL 1:**
    ```bash
    cd ~/ros2_ws/
    ```
    ***

1. Inside the workspace there is a directory called `src`. All new packages need to be located in the `src` folder, so we **must** be located here when we create a new package. So, use the `cd` command again to navigate into the `src` folder:

    ***
    **TERMINAL 1:**
    ```bash
    cd src
    ```
    ***

1. Now, use the `ros2 pkg create` tool to create a new package called `part1_pubsub`:

    ***
    **TERMINAL 1:**
    ```bash
    ros2 pkg create part1_pubsub --build-type ament_cmake
    ```
    ***
    
1. Navigate into this new package directory (using `cd`).

1. `tree` is a **Linux command** which shows us the content of the current directory in a nice tree-like format (you may need to install it first, using `sudo apt install tree`). Use `tree` now to show the current content of the `part1_pubsub` directory, as created by the `ros2 pkg create` tool.

    ``` { .txt .no-copy }
    ~/ros2_ws/src/part1_pubsub$ tree
    .
    ├── CMakeLists.txt
    ├── include
    │   └── part1_pubsub
    ├── package.xml
    └── src

    3 directories, 2 files
    ```

    Out of the box, this is formatted as a C++ package, so we'll need to add some more to it now to make it Python-friendly...
    
1. We'll now use two more Linux Commands:

    1. `mkdir` to make a new directory 
    1. `touch`: to make a new (empty) file

    First, make a folder within the `part1_pubsub` that's *also* called `part1_pubsub`!

    ```bash
    mkdir part1_pubsub
    ```

    Then, create an empty file in there called `__init__.py`:

    ```bash
    touch part1_pubsub/__init__.py
    ```

    Finally, create *another* directory called `scripts`:

    ```bash
    mkdir scripts
    ```

1. Let's use `tree` again to have a look at the content of our package now:

    ``` { .txt .no-copy }
    ~/ros2_ws/src/part1_pubsub$ tree
    .
    ├── CMakeLists.txt
    ├── include
    │   └── part1_pubsub
    ├── package.xml
    ├── part1_pubsub
    │   └── __init__.py
    ├── scripts
    └── src

    5 directories, 3 files
    ```

... this is only part of the process, and it already seems very long-winded... perhaps the best way is to explain this and then provide a pre-made template that can be downloaded (and modified...)

#### :material-pen: Exercise Y: Creating a publisher node {#exy}

