---
title: "Assignment #2: Team Robotics Project"
--- 

## Overview

In Assignment #2 you will put into practice everything that you are learning about ROS in Assignment #1, and explore the capabilities of the framework further.

You will attend a 2-hour lab session per week in Diamond Computer Room 5 for the full 12-week semester. You will work **in teams** to develop ROS Nodes for our TurtleBot3 Waffles that allow them to successfully complete a number of robotics tasks in a *real-world environment*. 

Assignment #2 is split into two parts: **Part A** and **Part B**. You will complete Part A in the first half of the semester (Weeks 1-6) and then move on to Part B in the second half of the semester (Weeks 7-12). 

## The Tasks

<center>

| Part | Tasks | Marks<br />(/100) | Submission |
| :---: | :---  | :---: | :---: |
| **A** | **Task 1**: [Velocity Control]()<br />**Task 2**: [Avoiding Obstacles]() | 20<br />20 | Week 6 |
| **B** | **Tasks 3 & 4**:<br />More Details Coming Soon... | 60 | Week 12 |

</center>

As shown above, there are **four tasks** in total that you must complete for Assignment #2, worth a total of **100 marks** overall. Exact submission deadlines will be stated on Blackboard.

Tasks **1**, **2** & **3** are *programming tasks*, requiring you to develop ROS nodes for the Waffles in order for them to complete certain real-world objectives in the robot arena in Computer Room 5. All three of these tasks will be marked based on how well the robot completes each of the objectives. Task **4** will require you to produce some *documentation* to describe your approach to Task 3.

## Assessment

This assignment is worth 30% of the overall mark for COM2009. As a team you will be assessed on a ROS package that you develop to satisfy the above tasks.

**For Tasks 1, 2 & 3**:  
Each submission will be assessed by deploying your ROS package on one of the *robotics laptops* used extensively throughout the lab sessions. Nodes within your package will then be executed on the laptop to control a real robot in the Diamond Computer Room 5 Robot Arena.

**For Task 4**:  
Your package must contain a `README.md` file in the root of the package directory. Further details on the structure of this, and the marking criteria will be made available in due course... 

### Submissions

Before you get started on any of the tasks, **as a team** you'll need to create a single ROS package and host this on GitHub (further details on the next page). You can then add all the necessary functionality for each task as you go along. On each of the submission deadlines (as summarised above and detailed on Blackboard) we will clone your teams GitHub repo. 

!!! note
    You should work on each task **as a team**, and create only **one** ROS package/GitHub repo **per team** for this assignment.

## Your ROS Package

### Launching Your Code

In order to launch the necessary functionality within your package for a given task you will need to include correctly named *launch files*: `task1.launch.py`, `task2.launch.py`, etc. This will allow you to ensure that all the required functionality is executed when your submission is assessed, and also ensures that we know exactly how to launch this functionality in order to assess it. Full details of the requirements for each launch file are provided on the associated task page.

!!! warning 
    It's up to **you** to ensure that your code launches as intended for a given task. If it doesn't, then you'll be awarded zero marks, so **make sure you test it all out prior to submission**!

For more information on how to create `.launch.py` files, refer to the following resources:

1. [Assignment #1 Part 3](../assignment1/part3.md)
2. The [**Launch Files** *Extras*](../extras/launch-files.md) page 

### Key Requirements (Penalties...)

In order to be awarded any marks for any task outlined in the table above, you **must** ensure that the following key requirements are met in regard to the ROS package that you create:

TODO

<!-- submit (as well as any additional requirements specific to a given task):

1. Your package must be submitted to Blackboard as a `.tar` file with the following naming convention:

        com2009_team{}.tar
  
    Where the `{}` is replaced with your own team number. [See here for how to create a `.tar` archive of your package](submission.md).
  
1. Your ROS package directory, when extracted, must be named:

        com2009_team{}/

    Again, replacing the `{}` with your own team number!

1. Your ROS **package name** must also be the same, so that the following would work (for example):

    ```bash
    roslaunch com2009_team100 task1.launch
    ```

    (assuming you are Team 100!)

1. Finally (and most importantly), your ROS package must work *"out-of-the-box,"* i.e. the Teaching Team won't make any modifications or fix any errors for you!  -->

!!! warning
    Failure to follow these requirements could result in you being awarded **zero marks**!