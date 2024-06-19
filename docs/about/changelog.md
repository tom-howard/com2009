---
title: "Version History"
---

# Version History

## Iteration 4

**Academic Year**: 2023-24 

* Overhaul of both COM2009 Assignments due to changes to Diamond Teaching Spaces...
* Assignment #1 is now completed asynchronously, and the course notes have been changed to "parts" rather than "weeks."
* Content has been moved around slightly but is broadly the same except for an addition of more formal [PID section in Part 6](../com2009/assignment1/part6.md#pid-control-and-line-following-pid) (formerly "Week 6") to tie in with lecture material more closely.
* Overall, assumptions that everyone is working within WSL-ROS are no longer correct (again, due to Diamond PC room changes), so references to this have been removed and made more generic.
* [Assignment #2](../com2009/assignment2/README.md) is now only 4 tasks rather than 5, but all are now completed with real robots, where previously 3/5 were assessed in simulation instead.
* Students now have 12 weeks of labs to work on Assignment #2, where previously it was only 6.

## Iteration 3

**Academic Year**: 2022-23 

* Moved everything (from [the Wiki](https://github.com/tom-howard/COM2009/wiki)) across to this new site and made lots of tweaks and improvements along the way.
* Two new labs for AMR31001 have been added.

## Iteration 2

**Academic Year**: 2021-22 

* Updated for a new release of the WSL-ROS Environment: now running Ubuntu 20.04 and ROS Noetic.
* All code templates now updated for Python 3, including nice things like [f-strings for all string formatting](https://realpython.com/python-f-strings/).
* [Week 3 Exercise 4 (Autonomous Navigation)](https://github.com/tom-howard/COM2009/wiki/Week-3#ex4) has been revised to use command-line calls to the `/move_base_simple` action server to make the robot move to navigation goals (rather than using the GUI tools in RViz), in the hope that this will make it easier to see how the same thing could be achieved programmatically instead.
* Removed an exercise in [Week 5](https://github.com/tom-howard/COM2009/wiki/Week-5) to make it shorter (because it was a bit of a long one originally), but introduced [a couple of optional ones](https://github.com/tom-howard/COM2009/wiki/Week-5#advanced) instead for those who wish to delve further.

## Iteration 1

**Academic Year**: 2020-21 

* Initial release of the COM2009 practical ROS course and [the COM2009 Wiki](https://github.com/tom-howard/COM2009/wiki).
* Based on the brand new WSL-ROS environment (running Ubuntu 18.04 and ROS Melodic).
