---
title: "Assignment #2: Team Robotics Project"
description: Apply your ROS2 knowledge to real TurtleBot3 Waffles in the lab.
--- 

## Overview

In Assignment #2 you will put into practice everything that you are learning about ROS in Assignment #1, and explore the capabilities of the framework further.

You will attend a 2-hour lab session per week in Diamond Computer Room 5 for the full 12-week semester. You will work **in teams** to develop ROS Nodes for our TurtleBot3 Waffles that allow them to successfully complete a number of robotics tasks in a *real-world environment*. 

Assignment #2 is split into two parts: **Part A** and **Part B**. You will complete Part A in the first half of the semester (Weeks 1-6) and then move on to Part B in the second half of the semester (Weeks 7-12). 

## The Tasks

<center>

| Part | Tasks | Marks<br />(/100) | Submission |
| :---: | :---  | :---: | :---: |
| **A** | **Task 1**: [Velocity Control](./part-a/task1.md)<br />**Task 2**: [Avoiding Obstacles](./part-a/task2.md) | 20<br />20 | Friday of Week 6 at 10pm (GMT) |
| **B** | **Task 3**: [Exploration & Search](./part-b/task3.md)<br />**Task 4**: [Documentation](./part-b/task4.md) | 40<br />20 | Friday of Week 12 at 10pm (BST) |

</center>

As shown above, there are **four tasks** in total that you must complete for Assignment #2, worth a total of **100 marks** overall. Exact submission deadlines will be stated on Blackboard.

Tasks **1**, **2** & **3** are *programming tasks*, requiring you to develop ROS nodes for the Waffles in order for them to complete certain real-world objectives in the robot arena in Computer Room 5. All three of these tasks will be marked based on how well the robot completes each of the objectives. Task **4** will require you to produce some *documentation* to describe your approach to Task 3.

## Assessment

This assignment is **worth 30%** of the overall mark for COM2009. As a team you will be assessed on a ROS package that you develop to satisfy the above tasks.

Your ROS package will be assessed by the teaching team *off-line* in the weeks following the submission deadlines (as above). You will receive your marks, plus video recordings of the assessment within 3 weeks of submission[^holiday].

[^holiday]: If this falls over a holiday period (i.e. Easter), then the deadline for marking and the return of marks extends to 5 weeks (as per standard University policy).

### Tasks 1, 2 & 3

Each submission will be assessed by deploying your ROS package on one of the *robotics laptops* used extensively throughout the lab sessions. Nodes within your package will then be executed on the laptop to control a real robot in the Diamond Computer Room 5 Robot Arena.

### Task 4 

Your package must contain a `README.md` file in the root of your package directory. This README file must document the application that you have developed for Task 3. [Further details can be found on the Task 4 page](./part-b/task4.md). 

### Submissions

Before you get started on any of the tasks, **as a team** you'll need to create a single ROS package and host this on GitHub (which you'll do in [the Week 1 Lab Session](./getting-started.md)). You can then add all the necessary functionality for each task as you go along. On each of the submission deadlines (as summarised above and detailed on Blackboard) we will pull your team's work from your GitHub repo. [See here for further details](./assessment.md).

!!! note
    You should work on each task **as a team**, and create only **one** ROS package/GitHub repo **per team** for this assignment.

## Your ROS Package

### Launching Your Code

In order to launch the necessary functionality within your package for a given task you will need to include correctly named *launch files*: `task1.launch.py`, `task2.launch.py`, etc. This will allow you to ensure that all the required functionality is executed when your submission is assessed, and also ensures that we know exactly how to launch this functionality in order to assess it. Full details of the requirements for each launch file are provided on the associated task page.

For more information on how to create `.launch.py` files, refer to the following resources:

1. [Assignment #1 Part 3](../assignment1/part3.md)
2. [Launch File Arguments](../extras/launch-file-args.md) (Additional Resources) 

### Preparing for the Deadlines

You can find all the Key Information regarding assessment of the *programming* tasks on [this page](./assessment.md). It's extremely important that you follow all the *Key Requirements* outlined here regarding the structure, content and configuration of your ROS package, so please be sure to **read this page in full** at your earliest convenience!

