---  
title: "Part 5: ROS2 Actions"  
description: Building on what you learnt about ROS2 Services in Part 4, here you will look at ROS Actions, which are similar to Services, but with a few key differences.
---

## Introduction

:material-pen: **Exercises**: X essential 
:material-timer: **Estimated Completion Time**: Y

### Aims

In this part of the course you will learn about a third (and final) communication method available within ROS2: *Actions*.  Actions are essentially an advanced version of ROS 2 Services, and you will learn about exactly how these two differ and why you might choose to employ an action over a service for certain robotic tasks. 

### Intended Learning Outcomes

By the end of this session you will be able to:

1. Recognise how ROS Actions differ from ROS Services and explain where this method might be useful in robot applications.
1. Explain the structure of Action messages and identify the relevant information within them, enabling you to build Action Servers and Clients.
1. Implement Python Action *Client* nodes that utilise *concurrency* and *preemption*.
1. Develop Action Server & Client nodes that could be used as the basis for a robotic search strategy.

### Quick Links

* [Exercise 1: Launching an Action Server and calling it from the command-line](#ex1)
* [Exercise 2: Building a Python Action Client Node with concurrency](#ex2)
* [Exercise 3: Building a Preemptive Python Action Client Node](#ex3)
* [Exercise 4: Developing an "Obstacle Avoidance" behaviour using an Action Server](#ex4)
* **Advanced (optional) exercises**:
    * [Advanced Exercise 1: Implementing a Search strategy](#adv_ex1)
    * [Advanced Exercise 2: Autonomous Navigation using waypoint markers](#adv_ex2)

### Additional Resources

* [The Action Client Code (for Exercise 2)](./part5/action_client.md)
* [The Preemptive Action Client Code (for Exercise 3)](./part5/preemptive_action_client.md)

## Getting Started

**Step 1: Launch your ROS Environment**

**Step 2: Launch VS Code**  

**Step 3: Make Sure The Course Repo is Up-To-Date**

Check that the Course Repo is up-to-date before you start on these exercises. [See here for how to install and/or update](../assignment1/part2.md#getting-started).


## What is a ROS Action?

As you will have observed from the above exercise, a ROS Action actually seems to work a lot like a ROS Service.  We've seen that we have a **feedback** message associated with an Action though, which is indeed different, but this isn't the main differentiating feature. The key difference is that when a node calls a ROS Action (i.e. an action *"Caller"* or *"Client"*), it *doesn't* need to wait until the action is complete before it can move on to something else: it can continue to do other tasks at the same time. Unlike ROS Services then, ROS Actions are *Asynchronous*, which makes them useful when implementing robotic behaviours that take a longer time to execute, and which an Action *Client* might need to be updated on throughout the process.

Recall the five messages associated with the action server [from the exercise above](#action_msgs), the messages had the following names:

``` { .txt .no-copy }
/cancel
/feedback
/goal
/result
/status
```

The top item there hints at the most important feature of ROS Actions: they can be cancelled (or *"preempted"*), which we'll learn more about later.  

The other thing to note is that - where we used the `ros2 service` command to interrogate the ROS Services that were active on our ROS network previously - Actions use ROS Topics, so we use `ros2 interface` commands to interrogate action servers.

### The Format of Action Messages



``` { .txt .no-copy }
ros2 interface list
[some topics...]
/camera_sweep_action_server/cancel
/camera_sweep_action_server/feedback
/camera_sweep_action_server/goal
/camera_sweep_action_server/result
/camera_sweep_action_server/status
[some more topics...]
```

#### "Cancel" and "Status"

Every ROS Action has both a **cancel** and **status** message associated with them. These are standardised, so the format of these two messages will always be the same, regardless of the type of Action Server we use. We won't worry about these too much for now, but we'll make use of them in some ROS Nodes that we'll build in a short while.

The **feedback**, **goal** and **result** messages will be different for any given action server though, and so we need to know about the format of all of these before we attempt to make a call to the action server.