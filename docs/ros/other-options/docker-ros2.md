---
title: Installing "docker-ros2" for Linux, Mac (and Windows)
---

**Applicable to**: Linux, Mac (and Windows) Users

!!! warning "Windows Users"
    While this *should* work on Windows machines, it's currently untested so proceed with caution! (and remember that there's always [the WSL-ROS2 option](../wsl-ros/install.md))

## Acknowledgments

This solution has been put together by Atri Hegde, a COM2009 student from 2024 (and a course demonstrator in 2025!)

**Thanks Atri**!!

## Setup

See here for the `docker-ros2` repo and the instructions on how to install it: 

<center>[https://github.com/hegde-atri/ros2-docker](https://github.com/hegde-atri/ros2-docker){target="_blank"}</center>

## Launching the ROS Environment {#launch}

### Mac & Linux

Instructions on how to launch the ROS2 docker container are provided [in the README](https://github.com/hegde-atri/ros2-docker?tab=readme-ov-file#ros2-humble-development-container){target="_blank"}, so please consult this for all the details. Essentially though (*if you're on a Mac or Linux machine*) then you first need to fire up the docker container using:

```bash
ros_start
```

... and once that's done, you need to run the following command to enter the ROS2 environment:

```bash
ros_shell
```

The above assumes that you have already set up your shell appropriately (again [see the README](https://github.com/hegde-atri/ros2-docker?tab=readme-ov-file#ros2-humble-development-container){target="_blank"}).

### Windows

[The process for Windows users is slightly different](https://github.com/hegde-atri/ros2-docker?tab=readme-ov-file#windows){target="_blank"}. 
