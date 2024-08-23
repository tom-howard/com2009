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

It's important to work in a specific filesystem location when we create and work on our own ROS packages. These are called *"Workspaces"* and you should already have one ready to go within your local ROS environment[^workspaces]:

``` { .bash .no-copy }
~/ros2_ws/src/
```

!!! note 
    `~` is an alias for your home directory. So `cd ~/ros2_ws/src/` is the same as typing `cd /home/{your username}/ros2_ws/src/`.

!!! warning "Important"
    All new packages **must** be located in the `src` folder of the workspace!!


[^workspaces]: [You can learn more about ROS2 Workspaces here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#background). 

#### :material-pen: Exercise XX: Creating your own ROS Package {#exx}

The `ros2` Command Line Interface (CLI) includes a tool to create a new ROS packages: `ros2 pkg create`. This tool supports two different *"build types,"* and packages are structured slightly differently in each case:

1. **CMake**: for packages containing nodes written in *C++*:
    
    `ros2 pkg create --build-type ament_cmake`
    
2. **Python**: for packages containing nodes written in well, er, *Python*!
    
    `ros2 pkg create --build-type ament_python`

You can learn more about all this from the [Official ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), if you're interested.

We'll be using Python throughout this course, but we'll actually take a slightly different approach to package creation that will provide us with a little more flexibility and ease of use (particularly for things we'll do later on in the Assignment #1 course and in Assignment #2). We've therefore created a helper script (inside the `tuos_ros` Course Repo) to help you create packages without using *either* of the above two scripts. The approach we'll take is based on [this tutorial from the Robotics Backend](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/), so feel free to look at this if you'd like to find out more. If not, then simply follow the steps below to create your first ROS package for this course, using the `create_pkg.sh` helper tool!

1. Navigate into the `tuos_ros` Course Repo that you downloaded earlier by using the Linux `cd` command (**c**hange **d**irectory). In **TERMINAL 1** enter the following:

    ***
    **TERMINAL 1:**
    ```bash
    cd ~/ros2_ws/src/tuos_ros/
    ```
    ***

1. Here you'll find the `create_pkg.sh` helper script. Run this now using the following command to create a new package called `part1_pubsub`:

    ***
    **TERMINAL 1:**
    ```bash
    ./create_pkg.sh part1_pubsub
    ```
    ***
    
1. Navigate into this new package directory (using `cd`):

    ***
    **TERMINAL 1:**
    ```bash
    cd ../part1_pubsub/
    ```
    ***

    !!! info
        `..` means "go back one directory", so that command above is telling `cd` to navigate *out* of the `tuos_ros` directory (and therefore back to `~/ros2_ws/src/`), and then go *into* the `part1_pubsub` directory from there.

1. `tree` is a **Linux command** which shows us the content of the current directory in a nice tree-like format (you may need to install it first, using `sudo apt install tree`). Use `tree` now to show the current content of the `part1_pubsub` directory:

    ``` { .txt .no-copy }
    ~/ros2_ws/src/part1_pubsub$ tree
    .
    ├── CMakeLists.txt
    ├── include
    │   └── part1_pubsub
    │       └── minimal_header.hpp
    ├── package.xml
    ├── part1_pubsub
    │   ├── __init__.py
    │   └── minimal_module.py
    ├── scripts
    │   └── minimal_node.py
    └── src
        └── miminal_node.cpp

    5 directories, 7 files
    ```

    * `scripts`: is a *directory* that will contain all the Python Nodes that we'll create (you'll notice a `minimal_node.py` already exists in there).
    * `part1_pubsub`: is a *directory* that we can use to store Python *modules*, that we can then import into our main Python nodes
        
        (`#!py from part1_pubsub.minimal_module import ...`, for example)
    
    * `package.xml` and `CMakeLists.txt`: are both *files* that define our package, and how it must be built (using `colcon build`). We'll explore these more shortly... 
    
... TODO from here

#### :material-pen: Exercise Y: Creating a publisher node {#exy}

