---
title: "Launch File Arguments"
---

## Introduction

As we know from the work we've done in Assignment #1, ROS applications can be executed in two different ways:  

1. Using the `ros2 run` command:

    ``` { .bash .no-copy }
    ros2 run {Package name} {Node name}
    ```

1. Using the `ros2 launch` command:

    ``` { .bash .no-copy }
    ros2 launch {Package name} {Launch file}
    ```

The `ros2 launch` command, used in combination with *launch files*, offers a few advantages over `ros2 run`, for example:

1. **Multiple nodes** can be executed **simultaneously**.
1. From within one launch file, we can call *other* launch files.
1. We can pass in **additional arguments** to launch things conditionally, or to change the behaviour of our ROS applications *dynamically*.

Points 1 and 2 above are explored in [Assignment #1 Part 3](../assignment1/part3.md) (Exercises 1 & 2). In this section we'll explore Point 3 further[^more].

[^more]: For more advanced launch file features, [have a look at this guide](https://github.com/MetroRobots/rosetta_launch){target="_blank"}.

!!! note
    Make sure you [check for updates to the course repo](../extras/course-repo.md#updating) before moving on!

## Identifying Launch Arguments

We can use the `-s` option with `ros2 launch` to discover the additional arguments that can be supplied to any given launch file. Take the `waffle.launch.py` launch file from `tuos_simulations`, for example:

```bash
ros2 launch tuos_simulations waffle.launch.py -s
```

You should be presented with a range of arguments here, starting with:

``` { .txt .no-copy }
$ ros2 launch tuos_simulations waffle.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):
urdf_file_name : turtlebot3_waffle.urdf

    'with_gui':
        Select whether to launch Gazebo with or without Gazebo Client (i.e. the GUI).
        (default: 'true')
```

Scroll to the *bottom* of the list, and you should see the following:

``` { .txt .no-copy }
    'x_pose':
        Starting X-position of the robot
        (default: '0.0')

    'y_pose':
        Starting Y-position of the robot
        (default: '0.0')

    'yaw':
        Starting orientation of the robot (radians)
        (default: '0.0')
```

Using these arguments, we can control the position and orientation of the Waffle when it is spawned into the simulated world. Try this:

```txt
ros2 launch tuos_simulations waffle.launch.py x_pose:=1 y_pose:=0.5
```

The robot should spawn into an empty world, but at coordinate position $x=1.0$, $y=0.5$, rather than $x=0$, $y=0$, as would normally be the case.

## Launching Launch Files from Within Launch Files!

This was covered in [Assignment #1 Part 3 Exercise 2](../assignment1/part3.md#ex2), where we learnt how to launch an "Empty World" simulation from within our own launch file (and also launch a velocity control node from one of our own packages alongside this).

## Passing Launch Arguments

How do we pass an argument to a launch file (`tuos_simulations/waffle.launch.py`, for example) that we are executing from within *another* launch file? 

As per [Assignment #1 Part 3 Exercise 2](../assignment1/part3.md#ex2), a basic launch file would look like this (in this case configured to launch `tuos_simulations/waffle.launch.py`):

```py title="launch_args_example.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tuos_simulations"), 
                    "launch", "waffle.launch.py"
                )
            )
        )
    ])
```

To launch this *and* supply the `x_pose` and `y_pose` launch arguments to it as well, we need to add the following:

```py title="launch_args_example.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tuos_simulations"), 
                    "launch", "waffle.launch.py"
                )
            ),
            launch_arguments={ # (1)!
                'x_pose': '1.0',
                'y_pose': '0.5' # (2)!
            }.items() 
        )
    ])
```

1. Arguments are passed to the launch file via the `launch_arguments` option of `IncludeLaunchDescription()`.
2. Arguments are passed as a dictionary, which can contain multiple key value pairs separated by commas: `#!py dict = {key1:value1, key2:value2, ... }`. 
    
    In this case, *keys* are the names of the launch arguments to be passed to the `waffle.launch.py` launch file, and **values** are the actual values we want to assign to those arguments (and which can be changed as required).

## A Very Brief Introduction to ROS Parameters

To create arguments for our own launch files and to be able to pass these arguments into our own Nodes, we need to use **the ROS parameter System**.

There's a node in the `tuos_examples` package called `param_publisher.py`. This node looks for a parameter on the ROS network called `word`, reads its value and then publishes this to a topic called `/chatter`.

Parameters are used as a way to configure nodes, and can be further used to change their behaviour dynamically during run time. [You can read more about ROS Parameters here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html){target="_blank"}, and you can also **[take a look at ^^the code for the `param_publisher.py` file^^ here](https://github.com/tom-howard/tuos_ros/blob/humble/tuos_examples/scripts/param_publisher.py){target="_blank"}**, where you'll see that it's fairly straightforward to declare and read parameter values from within a node. 

If we run this node using `ros2 run` a default value is used for the `word` parameter:

``` { .txt .no-copy }
$ ros2 run tuos_examples param_publisher.py
[INFO] [####] [param_publisher]: The 'param_publisher' node is running...
[INFO] [####] [param_publisher]: Publishing the word 'hello'.
```

In a different terminal, we can read the values that are being published to the `/chatter` topic using `ros2 topic echo`:

``` { .txt .no-copy }
$ ros2 topic echo /chatter
data: hello
---
data: hello
---
```

## Defining Command Line Arguments for Launch Files

We can create a basic launch file to launch the `param_publisher.py` file as shown below:

```py
from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='tuos_examples', 
            executable='param_publisher.py', 
            name='the_param_publisher_node' 
        )
    ])
```

In order to follow along here, you'll need to create this launch file within one of your own packages. Refer back to [Assignment #1 Part 3](../assignment1/part3.md) for a reminder on how all this works.

To define an argument for this launch file, we use the `DeclareLaunchArgument` action, which must be included as an item in the `LaunchDescription`:

```py
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument # (1)!

def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='some_word', 
            description="A word, any word.",
            default_value='Hi'
        ), # (2)!
        Node( 
            package='tuos_examples', 
            executable='param_publisher.py', 
            name='the_param_publisher_node' 
        )
    ])
```

1. We need to import `DeclareLaunchArgument` so that we can use it in the launch file.
2. Don't forget the comma to separate the two launch description items: `DeclareLaunchArgument()` and `Node()`! 

We're defining **three things** when declaring the launch argument:

1. `name`: The **name** of the argument.
2. `description`: A description of what this argument is used for.
3. `default_value`: A value that will be assigned to `name` if we don't provide one when executing the launch file.

## Passing Launch File Arguments to Python Nodes (via Parameters)

We defined a launch argument in the step above, but (currently) this argument isn't actually being used anywhere. We want to pass this value to our `param_publisher.py` node via a parameter (as discussed earlier).

To do this, we can add an argument to the `Node()` launch description item:

```py
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration # (1)!

def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='some_word', 
            description="A word, any word.",
            default_value='Hi'
        ),
        Node( 
            package='tuos_examples', 
            executable='param_publisher.py', 
            name='param_publisher_node',
            parameters=[{'word': LaunchConfiguration('some_word')}] 
        )
    ])
```

1. Another new import here!!

Remember that our `param_publisher.py` node is looking for a parameter called `word`, and we are passing this in using the value supplied by the *launch file* argument `some_word`. So, if we call this launch file *without* supplying the `some_word` argument, a default value of `Hi` will be set (instead of the default value of `Hello` set by the node itself). You can test this out by running the pre-made `cli_example.launch.py` launch file also available in the `tuos_examples` package:

``` { .txt .no-copy }
$ ros2 launch tuos_examples cli_example.launch.py
[INFO] [launch]: All log files can be found below xxx
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [param_publisher.py-1]: process started with pid [####]
[param_publisher.py-1] [INFO] [####] [param_publisher_node]: The 'param_publisher_node' node is running...
[param_publisher.py-1] [INFO] [####] [param_publisher_node]: Publishing the word 'Hi'.
[param_publisher.py-1] [INFO] [####] [param_publisher_node]: Publishing the word 'Hi'.
```

Now, do this again but this time supplying the `some_word` command line argument to the launch file:

``` { .txt .no-copy }
$ ros2 launch tuos_examples cli_example.launch.py some_word:=goodbye
[INFO] [launch]: All log files can be found below xxx
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [param_publisher.py-1]: process started with pid [####]
[param_publisher.py-1] [INFO] [####] [param_publisher_node]: The 'param_publisher_node' node is running...
[param_publisher.py-1] [INFO] [####] [param_publisher_node]: Publishing the word 'goodbye'.
[param_publisher.py-1] [INFO] [####] [param_publisher_node]: Publishing the word 'goodbye'.
```

Here, we've seen how we can build a launch file that accepts a command line argument, and how we can pass the *value* of that command line argument into a ROS node.