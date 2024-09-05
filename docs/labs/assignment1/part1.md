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

1. In **TERMINAL 1**, Navigate into the *"ROS2 Workspace"* using the `cd` command[^ros2_ws]:

    [^ros2_ws]: What is a ROS2 Workspace? [You can find out more here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#background). 

    ```bash
    cd ~/ros2_ws/src/
    ```

1. Then, run the following command to clone the Course Repo from GitHub:

    ***
    **TERMINAL 1:**
    ```bash
    git clone https://github.com/tom-howard/tuos_ros.git -b humble
    ```
    ***

1. Once this is done, you'll need to build this using a tool called *"Colcon"*[^colcon]:

    [^colcon]: What is **Colcon**? [Find out more here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#background).

    ***
    **TERMINAL 1:**
    ```bash
    cd ~/ros2_ws/ && colcon build --packages-up-to tuos_ros && source ~/.bashrc
    ```
    ***

Don't worry too much about what you just did, for now. We'll cover this in more detail throughout the course. That's it for now though, we'll start using some of the packages that we've just installed a bit later on..




## Creating Your First ROS Applications

Shortly you will create some simple publisher and subscriber nodes in Python and send simple ROS messages between them. As we learnt earlier though, ROS applications must be contained within *packages*, and so we need to create a package first in order to start creating our own ROS nodes. 

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

The `ros2` Command Line Interface (CLI) includes a tool to create a new ROS packages: `ros2 pkg create`. This tool supports two different *"build types:"*

1. **CMake** (for packages containing nodes written in *C++*):
    
    `ros2 pkg create --build-type ament_cmake`
    
2. **Python** (for packages containing nodes written in well, er, *Python*!):
    
    `ros2 pkg create --build-type ament_python`

    Packages are structured slightly differently in each case.

You can learn more about all this from the [Official ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) (if you're interested).

We'll be using Python throughout this course, but we'll actually take a slightly different approach to package creation that will provide us with a little more flexibility and ease of use (particularly for things we'll do later on in the Assignment #1 course and in Assignment #2). We've therefore created a helper script (inside the `tuos_ros` Course Repo) to help you create packages without using *either* of the above two commands. The approach we'll take is based on [this tutorial (courtesy of the Robotics Backend)](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/), so feel free to look at this if you'd like to find out more. Then, simply follow the steps below to create your first ROS package for this course, using the `create_pkg.sh` helper tool.

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
        `..` means "go back one directory," so that command above is telling `cd` to navigate *out* of the `tuos_ros` directory (and therefore back to `~/ros2_ws/src/`), and then go *into* the `part1_pubsub` directory from there.

1. `tree` is a **Linux command** which shows us the content of the current directory in a nice tree-like format. Use `tree` now to show the current content of the `part1_pubsub` directory:

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
        └── minimal_node.cpp

    5 directories, 7 files
    ```

    * `scripts`: is a *directory* that will contain all the Python Nodes that we'll create (you'll notice a `minimal_node.py` already exists in there).
    * `part1_pubsub`: is a *directory* that we can use to store Python *modules*, that we can then import into our main Python nodes
        
        (`#!py from part1_pubsub.minimal_module import ...`, for example)
    
    * `package.xml` and `CMakeLists.txt`: are both *files* that define our package, and how it must be built (using `colcon build`). We'll explore these more shortly... 

#### :material-pen: Exercise Y: Creating a publisher node {#exy}

1. From the root of your `part1_pubsub` package, navigate to the `scripts` folder using the `cd` command.
1. `touch` is a **Linux command** that we can use to create an empty file. Use this to create an empty file called `publisher.py`, which we will add content to shortly:

    ***
    **TERMINAL 1:**
    ```bash
    touch publisher.py
    ```
    ***

1. Use `ls` to verify that the file has been created, but use the `-l` option with this, so that the command provides its output in *"a long listing format"*:

    ***
    **TERMINAL 1:**
    ```bash
    ls -l
    ```
    ***

    This should output something similar to the following:

    ``` { .txt .no-copy }
    ~/ros2_ws/src/part1_pubsub/scripts$ ls -l
    total 4
    -rwxr-xr-x 1 student student 339 MMM DD HH:MM minimal_node.py
    -rw-r--r-- 1 student student   0 MMM DD HH:MM publisher.py
    ```

    This confirms that the file exists, and the `0` in the middle of the bottom line there indicates that the file is empty (i.e. its current size is 0 bytes), which is what we'd expect.

1. We therefore now need to open the file and add content to it. We'd recommend using Visual Studio Code (VS Code) as an IDE for this course, which can be launched with the following command in **TERMINAL 1**:

    ***
    **TERMINAL 1:**
    ```bash
    code ~
    ```
    ***

    [TODO: does this work for Docker??]

1. Using the VS Code File Explorer, locate the `publisher.py` file that you have just created (`ros2_ws/src/part1_pubsub/scripts/`) and click on the file to open it in the main editor. 

1. Once opened, copy [the code provided here](./part1/publisher.md) into the empty file and save it. <a name="pub_ret"></a>
    
    !!! note
        It's important that you understand how this code works, so make sure that you **read the annotations**!
    
1. Next, we need to add our `publisher.py` file as an executable to our package's `CMakeLists.txt`. This will ensure that it then gets built when we run `colcon build` (in the next step).

    In VS Code, open the `CMakeLists.txt` file that is at the root of your `part1_pubsub` package directory (`ros2_ws/src/part1_pubsub/CMakeLists.txt`). Locate the lines (near the bottom of the file) that read:

    ``` { .txt .no-copy}
    # Install Python executables
    install(PROGRAMS
      scripts/minimal_node.py
      DESTINATION lib/${PROJECT_NAME}
    )
    ```

    Replace `minimal_node.py` with `publisher.py` to define this as a Python executable in your package:

    ``` { .txt .no-copy }
    # Install Python executables
    install(PROGRAMS
      scripts/publisher.py
      DESTINATION lib/${PROJECT_NAME}
    )
    ```

1. Now, use `colcon` to build your package.
    
    1. You **MUST** run this from the **root** of your Colcon Workspace (i.e.: `~/ros2_ws/`), **NOT** the `src` directory (`~/ros2_ws/src/`), so navigate there now using `cd`:

        ```bash
        cd ~/ros2_ws/
        ```

    1. Then, use the following `colcon` command to build your package:

        ```bash
        colcon build --packages-select part1_pubsub --symlink-install
        ```

        !!! info "What do the additional arguments above do?"

            * `--packages-select`: Build *only* the `part1_pubsub` package, nothing else (without this `colcon` would attempt to build *every* package in the workspace).
            * `--symlink-install`: Ensures that you don't have to re-run `colcon build` every time you make a change to your package's executables (i.e. your Python files in the `scripts` directory).
    
    1. Finally, "re-source" your `bashrc`[^source-bashrc]:

        [^source-bashrc]: What does `source ~/.bashrc` do? [See here for an explanation](https://devconnected.com/source-command-on-linux-explained/#Source_to_update_your_current_shell_environment_bashrc).

        ```bash
        source ~/.bashrc
        ```

1. We should now be able to run this node using the `ros2 run` command.
    
    Remember: `ros2 run {package name} {script name}`, so:

    ***
    **TERMINAL 1:**
    ```bash
    ros2 run part1_pubsub publisher.py
    ```
    ***

    ... Hmm, something not quite right? If you typed the command exactly as above and then tried to run it, you probably just received the following error:

    ``` { .txt .no-copy }
    $ ros2 run part1_pubsub publisher.py
    No executable found
    ``` 

    When we create a file using `touch` it is given certain *permissions* by default. Run `ls -l` again (making sure that your terminal is in the right location: `~/ros2_ws/src/part1_pubsub/scripts/`).
        
    The first bit tells us about the permissions that are currently set: `-rw-r--r--`. This tells us *who* has permission to do *what* with this file and (currently) the first bit: `-rw-`, tells us that we have permission to **r**ead or **w**rite to it. There is a *third* option we can set too though, which is the *execute* permission, and we can set this using the `chmod` **Linux command**...

1. Run the `chmod` command as follows:

    ***
    **TERMINAL 1:**
    ```bash
    chmod +x publisher.py
    ```
    ***

1. Now, run `ls -l` again to see what has changed:
    
    ***
    **TERMINAL 1:**
    ```bash
    ls -l
    ```
    ***

    We have now granted permission for the file to be e**x**ecuted too:
    
    ``` { .txt .no-copy }
    -rwxr-xr-x 1 student student 1125 MMM DD HH:MM publisher.py
    ```

1. OK, now use `ros2 run` again to (*hopefully!*) run the `publisher.py` node (remember: `ros2 run {package name} {script name}`).
    
    If you see a message in the terminal similar to the following then the node has been launched successfully:
        
    ``` { .txt .no-copy }
    [INFO] [#####] [simple_publisher]: The 'simple_publisher' node is inisialised.
    ```

    Phew!

1. We can further verify that our publisher node is running using a number of different tools. Try running the following commands in **TERMINAL 2**:

    1. `ros2 node list`: This will provide a list of all the *nodes* that are currently active on the system. Verify that the name of our publisher node is visible in this list (it's probably the only item in the list at the moment!)
    1. `ros2 topic list`: This will provide a list of the *topics* that are currently being used by nodes on the system. Verify that the name of the topic that our publisher is publishing messages to (`/my_topic`) is present within this list.

#### Interrogating ROS Topics {#rostopic}

So far we have used the `ros2 topic` ROS command with two additional arguments: [TODO: check this!]

* `list`: to provide us with a *list* of all the topics that are active on our ROS system, and
* `info`: to provide us with *information* on a particular topic of interest.

We can use the *autocomplete functionality* of the Linux terminal to provide us with a list of *all* the available options that we can use with the `ros2 topic` command.  To do this type `ros2 topic` followed by a ++space++ and then press the ++tab++ key twice:

``` { .bash .no-copy }
rostopic[SPACE][TAB][TAB]
```

You should then be presented with a list of all options:

[TODO: a gif]

<!-- <figure markdown>
  ![](../../images/ros-cli/rostopic_autocomplete.gif)
</figure> -->

* `ros2 topic hz {topic name}` provides information on the frequency (in Hz) at which messages are being published to a topic:

    ```bash
    ros2 topic hz /my_topic
    ```

    This should tell us that our publisher node is publishing messages to the `/my_topic` topic at (or close to) 1 Hz, which is exactly what we ask for in the `publisher.py` file (in the `__init__` part of our `Publisher` class). Enter ++ctrl+c++ to stop this command.

* `ros2 topic echo {topic name}` shows the messages being published to a topic:

    ```bash
    ros2 topic echo /my_topic
    ```

    This will provide a live stream of the messages that our `publisher.py` node is publishing to the `/my_topic` topic. Enter ++ctrl+c++ to stop this.

* We can see some additional options for the `echo` command by viewing the help documentation for this too:

    ```bash
    ros2 topic echo --help
    ```

    From here, for instance, we can learn that if we just wanted to print the first message that was received we could use the `-once` option, for example:

    ```bash
    ros2 topic echo /my_topic --once
    ```

#### :material-pen: Exercise Z: Creating a subscriber node {#exZ}

To illustrate how information can be passed from one node to another (via topics and messages) we'll now create another node to *subscribe* to the topic that our publisher node is broadcasting messages to.

1. In **TERMINAL 2** use the filesystem commands that were introduced earlier (`cd`, `ls`, etc.) to navigate to the `scripts` folder of your `part1_pubsub` package.

1. Use the same procedure as before to create a new empty Python file called `subscriber.py` and remember to make it executable! <a name="sub_ret"></a>

1. Then, open the newly created `subscriber.py` file in VS Code, paste in [the code here](./part1/subscriber.md) and save it. 
    
    Once again, it's important that you understand how this code works, so **make sure you read the code annotations**! 

1. Next, we need to add this as an additional executable for our package. 

    Open up the `CMakeLists.txt` file at the root of your `part1_pubsub` package directory again, head back to the `# Install Python executables` section and add the `subscriber.py` file:

    ``` { .txt .no-copy }
    # Install Python executables
    install(PROGRAMS
      scripts/publisher.py
      scripts/subscriber.py
      DESTINATION lib/${PROJECT_NAME}
    )
    ```

1. Now we need to `colcon build` again.
    
    1. Make sure you're at the **root** of the Colcon Workspace:

        ```bash
        cd ~/ros2_ws/
        ```

    1. Run `colcon build` on *only* the `part1_pubsub` package:

        ```bash
        colcon build --packages-select part1_pubsub --symlink-install
        ```

    1. And then re-source the `bashrc`:

        ```bash
        source ~/.bashrc
        ```

1. Use `ros2 run` to execute your newly created `subscriber.py` node (remember: `ros2 run {package name} {script name}`). If your publisher and subscriber nodes are working correctly you should see an output like this:
    
    [TODO: another gif]

    <!-- <figure markdown>
      ![](part1/subscriber_output.gif)
    </figure> -->

1. As before, we can find out what nodes are running on our system by using the `ros2 node list` command. Run this in **TERMINAL 3**, you should see both your publisher *and* subscriber nodes listed there.

1. Finally, close down your publisher and subscriber nodes by entering ++ctrl+c++ in the terminals where they are running (should be 1 & 2).

**Advanced**: <a name="pubsub_plus"></a> 

You've now created a publisher and subscriber, both of which were able to communicate with one another over the `/chatter` topic, using the `String` *standard* ROS message type. This message is provided, by ROS, as part of the `std_msgs` package, but there are other simple message types within this package that we can use too to pass data around a ROS network too, one of which is `Float64`.

* How could you adapt your publisher and subscriber nodes to use the `Float64` message type, instead of `String`?