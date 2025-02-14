---
title: Key Assessment Info & Requirements
---

!!! warning
    Failure to follow all the requirements listed on this page could result in you receiving **zero marks** for a submission point and/or assignment task!

Your ROS package should be hosted on GitHub, it should be setup as a private repository, and you should have added `tom-howard` as a maintainer. 

In addition to this, you should have registered your package with the teaching team (via the Google Form), so that we know where to find it on the submission deadlines.

All the above was covered in [the Getting Started section](./getting-started.md), which you should have completed in the Week 1 Lab.

Having completed all of this *successfully*, we'll be able to pull your package on each of the submission dates so that your team's Assignment #2 work can be assessed. If you *haven't* completed all this, then you could receive zero marks!

!!! note
    At some point within the first few weeks of the course a `hello.md` file will be pushed to your repo (by Tom) to confirm that it has been registered correctly.

## Submission Points

[As discussed here](./README.md#the-tasks), there are **two submission points** for Assignment #2 and **four tasks** to complete overall: 

<center>

| Part | Tasks | Marks<br />(/100) | Submission |
| :---: | :---  | :---: | :---: |
| **A** | **Tasks 1 & 2** | 40 | Week 6 |
| **B** | **Tasks 3 & 4** | 60 | Week 11/12 |

</center>

See the task pages for full details on each of the four tasks.

##  Key Requirements

In addition to registering your package correctly (as above), you **must** also ensure that the following *Key Requirements* are met for each of the submission points (**A** and **B**): 

* [ ] The name of your ROS package must be:

    ``` { .txt .no-copy }
    com2009_teamXX_2025
    ```

    ... where `XX` should be replaced with your team number.

* [ ] Your package must contain **no build files** (`build/`, `install/`, `logs/`) that would be generated as a result of running `colcon build` from inside your package.

    !!! warning "Remember"
        **Always** run `colcon build` from the **root** of the ROS workspace (e.g. `~/ros2_ws/`), to ensure that all build files are generated in the right location in the filesystem (`~/ros2_ws/build/`, `~/ros2_ws/install/`, `~/ros2_ws/logs/`).

* [ ] On each of the deadlines, we will pull your work from the `main` branch of your package repository. We will **ONLY** assess work on your `main` branch!

* [ ] Your package's `package.xml` file must contain a `#!xml <maintainer>` tag for each member of your team. Add these as necessary, e.g.:

    ```xml
    <maintainer email="member.1@sheffield.ac.uk">Member 1's Name</maintainer>
    <maintainer email="member.2@sheffield.ac.uk">Member 2's Name</maintainer>
    ...
    ```

!!! warning
    Failure to follow **ALL** the Key Requirements will result in penalties being applied to your team's Part A and/or B mark. 

### Ensuring Successful Deployment 

For the assessment of each Assignment #2 Task, your package will be built and deployed on one of the Robotics Laptops that you'll have been working with extensively during the lab sessions. We will use the standard `student` user account, and your package will be downloaded to the `~/ros2_ws/src/` directory. 

* [ ] It must be possible to build your package by running the following command from the root of the local ROS2 Workspace, and this must build without errors:
    
    ``` { .bash .no-copy }
    colcon build --packages-select com2009_teamXX_2025
    ```

* [ ] You must ensure that a launch file exists for each of the *programming* tasks (Tasks 1, 2 & 3) and that these are executable (after having run the above `colcon build` command) so that we are able to launch your work as follows[^launch-files]:
    
    [^launch-files]: Make sure you have [defined an appropriate `install` directory in your package's `CMakeLists.txt`](../assignment1/part3.md#ex1) 

    ``` { .bash .no-copy }
    ros2 launch com2009_teamXX_2025 taskY.launch.py
    ```

    ... where `XX` will be replaced by your team number, and `Y` will be replaced by the appropriate task number.

* [ ] Any nodes within your package that are executed by the above launch files **must** have been correctly defined as package executables (i.e. in your `CMakeLists.txt`) and must **also** have been assigned the appropriate execute permission (i.e. with `chmod`).  

    !!! warning 
        It's up to **you** to ensure that your code launches as intended for a given task. If it doesn't, then you'll be awarded zero marks, so **make sure you test it all out prior to submission**!

## Other Important Information 

* The [`tuos_ros` Course Repo](../extras/course-repo.md) will be installed and up-to-date on the Robotics Laptop that we use to assess your work with.

* The Robotics Laptop that we use for the assessment will be selected at random.

* This laptop will have been paired with a robot prior to us attempting to run your submission.

* The robot will also be selected at random.

* We will have already [launched the *bringup* on the robot](../../waffles/launching-ros.md#step-3-launching-ros), so ROS will be up and running, and the robot will be ready to go in the arena.

* [A bridge between the robot and laptop will have already been established](../../waffles/basics.md#exBridge), and communications will be tested, prior to us attempting to launch your work for each task.
