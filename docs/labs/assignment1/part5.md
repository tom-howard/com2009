---  
title: "Part 5: ROS2 Actions"  
description: Building on what you learnt about ROS2 Services in Part 4, here you will look at ROS Actions, which are similar to Services, but with a few key differences.
---

## Introduction

:material-pen: **Exercises**: 4 essential (plus 2 *advanced* exercises)  
:material-timer: **Estimated Completion Time**: 3 hours (for the essential exercises only)

### Aims

In this part of the course you will learn about a third (and final) communication method available within ROS: *Actions*.  Actions are essentially an advanced version of ROS Services, and you will learn about exactly how these two differ and why you might choose to employ an action over a service for certain robotic tasks. 

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

The other thing to note is that - where we used the `rosservice` command to interrogate the ROS Services that were active on our ROS network previously - Actions use ROS Topics, so we use `rostopic` commands to interrogate action servers:<a name="rostopic_for_actions"></a>

1. `rostopic list`: to identify the action servers that are available on the network.
1. `rostopic echo`: to view the messages being published by a given action server.
1. `rostopic pub`: to call an action from the command-line. 

### The Format of Action Messages

Like Services, Action Messages have multiple parts to them, and we need to know what format these action messages take in order to be able to call them. We don't have a tool like `rossrv` to do this for Actions though, instead we have to use `rosmsg`, or look for the message definition inside the *Action Message Package*.

We ran `rostopic list` to identify our action server in the previous exercise, which told us that there was an action server running called `/camera_sweep_action_server`:

``` { .txt .no-copy }
rostopic list
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

We can run `rostopic info` on any of these to find out more about them...

#### "Goal"

Let's look at the **goal** to start with:

***
**TERMINAL 1:**
```bash
rostopic info /camera_sweep_action_server/goal
```
From which we obtain the usual information:

``` { .txt .no-copy }    
Type: tuos_msgs/CameraSweepActionGoal

Publishers: None

Subscribers:
  * /camera_sweep_action_server (http://localhost:#####/)
```
***

The `Type` field tells us that the action message belongs to the `tuos_msgs` package, and we can find out more about the `goal` message by using `rosmsg info`. You'll be familiar with how this works by now:

``` { .txt .no-copy }
rosmsg info {messageType}
```

Where `{messageType}` is established from the output of the `rostopic info` command above: 

``` { .txt .no-copy }
Type: tuos_msgs/CameraSweepActionGoal
```

When working with ROS Actions and the `rosmsg` command though, we can actually drop the word "`Action`" in the message Type, so our `rosmsg` command becomes:

***
**TERMINAL 1:**
```bash
rosmsg info tuos_msgs/CameraSweepGoal
```
Which will output:

``` { .txt .no-copy }
float32 sweep_angle
int32 image_count
```
***

??? info "Further Info"
    `rosmsg info tuos_msgs/CameraSweepActionGoal` will work as well, but we get a lot of other information in the output that we're not all that interested in. Give it a go and see the difference, if you want to!

In order to call this action server, we need to send a **goal**, and `rosmsg info` has just told us that there are **two** goal parameters that we must provide:

1. `sweep_angle`: a 32-bit floating-point value
1. `image_count`: a 32-bit integer

So we know more about our Action Server's *Goal* now, but there are two other parameters we still know nothing about: *Result* and *Feedback*. It's important to know about all three things in order to be able to work with the Action Server effectively, and we can use an alternative approach to interrogate all three at the same time...

#### "Goal," "Feedback" and "Result"

We know, from above, that the `/camera_sweep_action_server` messages are part of the `tuos_msgs` package, so we can navigate to the package directory (using `roscd`) and look at the actual message definition. 

***
**TERMINAL 1:**
```bash
roscd tuos_msgs/
```
***

Actions are always contained within an `action` folder inside the package directory, so we can then navigate into this folder using `cd`:

***
**TERMINAL 1:**
```bash
cd action/
```
***

Use the `ll` command again here to view all the action messages within the package. Here you should see the `CameraSweep.action` message listed. Run `cat` on this file to view the full message definition:<a name="camera_sweep_msg_params" ></a>

***
**TERMINAL 1:**
```bash
cat CameraSweep.action
```
``` { .txt .no-copy }
#goal
float32 sweep_angle    # the angular sweep over which to capture images (degrees)
int32 image_count      # the number of images to capture during the sweep
---
#result
string image_path      # The filesystem location of the captured images
---
#feedback
int32 current_image    # the number of images taken
float32 current_angle  # the current angular position of the robot (degrees)
```

!!! question "Questions"
    * What are the names of the **result** and **feedback** message parameters? (There are three parameters in total.)
    * What datatypes do these parameters use?

You'll learn how we use this information to develop Python Action Server & Client nodes in the following exercises.

### Concurrent Activity

An Action Server provides **feedback** messages at regular intervals whilst performing an action and working towards its **goal**.  This is one way that an Action Client can monitor the progress of the action that it has requested.  Another way it can do this is by monitoring the **status** of an action.  Both of these features enable *concurrency*, allowing an action client to work on other things whilst waiting for the requested behaviour to be completed by the action server.

<figure markdown>
  ![](part5/action_msgs.png)
</figure>

#### :material-pen: Exercise 2: Building a Python Action Client Node with Concurrency {#ex2}

1. You should only have one Terminal application instance open now, with three terminal tabs in it. **TERMINAL 3** should already be idle (i.e. not running any commands), and (if you haven't done so already) enter ++ctrl+c++ in **TERMINAL 1** and **TERMINAL 2** to stop the headless Gazebo simulation processes and the Camera Sweep Action Server respectively. 

1. In **TERMINAL 1** create a new package called `part5_actions` using the `catkin_create_pkg` tool [as you have done previously](./part4.md#ex1). This time, define `rospy`, `actionlib` and `tuos_msgs` as dependencies.
    
    !!! tip "Remember"
        Make sure you're in your `~/catkin_ws/src/` folder when you run the `catkin_create_pkg` command!
    
1. Once again, run `catkin build` on this and then re-source your environment:

    ***
    **TERMINAL 1:**  
    First:
    ```bash
    catkin build part5_actions
    ```
    Then:
    ```bash
    source ~/.bashrc
    ```
    ***

1. Navigate to the `src` folder of this package, create a file called `action_client.py` (using `touch`) and set this to be executable (using `chmod`).        

1. Review [the code provided here](./part5/action_client.md), and the annotations, then copy and paste the code into your newly created `action_client.py` file. <a name="ex2_ret"></a>

1. Then, in **TERMINAL 2**, execute the same launch file as before but this time with a couple of additional arguments:

    ***
    **TERMINAL 2:**
    ```bash
    ros2 launch tuos_simulations mystery_world.launch gui:=true camera_search:=true
    ```
    
    ... which will launch the Gazebo simulation in GUI mode this time, as well as the `/camera_sweep_action_server` too.

    ***

1. In **TERMINAL 1**, use `ros2 run` to call the action server with the `action_client.py` node that you have just created...

    !!! warning "Something not right?"
        You may need to change the values that have been assigned to the goal parameters, in order for the client to successfully make a call to the server!

    The node we have just created, in its current form, uses a *feedback callback function* to perform some operations while the action server is working. In this case, it simply prints the feedback data that is coming from the Action Server.  That's it though, and the `client.wait_for_result()` line still essentially just makes the client node wait until the action server has finished doing its job before it can do anything else. This still therefore looks a lot like a service, so let's modify this now to really build *concurrency* into the client node.

1. First, create a copy of your `action_client.py` node and call it `concurrent_action_client.py` (you will need to make sure you are still in the `src` directory of your `part5_actions` package before you run this command):

    ***
    **TERMINAL 1:**
    ```bash
    cp action_client.py concurrent_action_client.py
    ```
    ***

1. We want to use the **status** message from the action server now, and we can find out a bit more about this as follows:
    
    1. Use `rostopic info camera_sweep_action_server/status` to find the message type.
    1. Then, use `rosmsg info` (using the message type you have just identified) to tell you all the status codes that could be returned by the action server.

    You should have identified the following states, listed in the `status_list` portion of the message:

    ``` { .txt .no-copy }
    PENDING=0
    ACTIVE=1
    PREEMPTED=2
    SUCCEEDED=3
    ABORTED=4
    REJECTED=5
    ...
    ```

    We can set up our action client to monitor these status codes in a `while` loop, and then perform other operations inside this loop until the action has completed (or has been stopped for another reason).

1. To do this, replace the `client.wait_for_result()` line in the `concurrent_action_client.py` file with the following code:

    ```python
    rate = rospy.Rate(1)
    i = 1
    print("While we're waiting, let's do our seven-times tables...")
    while client.get_state() < 2:
        print(f"STATE: Current state code is {client.get_state()}")
        print(f"TIMES TABLES: {i} times 7 is {i*7}")
        i += 1
        rate.sleep()
    ```

1. Run the `concurrent_action_client.py` node and see what happens this time.  Essentially, we know that we can carry on doing other things as long as the status code is less than 2 (either `PENDING` or `ACTIVE`), otherwise either our goal has been achieved, or something else has happened...

### Cancelling (or *Preempting*) an Action {#preemptive_client}

Actions are extremely useful for controlling robotic tasks or processes that might take a while to complete, but what if something goes wrong, or if we just change our mind and want to stop an action before the goal has been reached? The ability to *preempt* an action is one of the things that makes them so useful.

#### :material-pen: Exercise 3: Building a Preemptive Python Action Client Node {#ex3}

1. In **TERMINAL 1** you should still be located within the `src` folder of your `part5_actions` package. If not, then go back there now! Create a new file called `preemptive_action_client.py` and make this executable.
1. Have a look at the code [here](./part5/preemptive_action_client.md), then copy and paste it into the `preemptive_action_client.py` node that you have just created.<a name="ex3_ret"></a>

    Here, we've built an action client that will cancel the call to the action server if we enter ++ctrl+c++ into the terminal.  This is useful, because otherwise the action server would continue to run, even when we terminate the client.  A lot of the code is similar to the Action Client from the previous exercise, but we've built a class structure around this now for more flexibility.  Have a look at [the code annotations](./part5/preemptive_action_client.md) and make sure that you understand how it all works.

1. Run this using `ros2 run`, let the server take a couple of images and then enter ++ctrl+c++ to observe the goal cancelling in action.

    !!! warning
        You'll need to set some values for the goal parameters again!
    
1. We can also cancel a goal conditionally, which may also be useful if, say, too much time has elapsed since the call was made, or the caller has been made aware of something else that has happened in the meantime (perhaps we're running out of storage space on the robot and can't save any more images!) This is all achieved using the `cancel_goal()` method.

    * Have a go now at introducing a conditional call to the `cancel_goal()` method once a total of **5 images** have been captured.
    * You could use the `captured_images` attribute from the `CameraSweepFeedback` message to trigger this.

### A Summary of ROS Actions

ROS Actions work a lot like ROS Services, but they have the following key differences:

1. They are **asynchronous**: a client can do other things while it waits for an action to complete.
1. They can be **cancelled** (or *preempted*): If something is taking too long, or if something else has happened, then an Action Client can cancel an Action whenever it needs to.
1. They provide **feedback**: so that a client can monitor what is happening and act accordingly (i.e. preempt an action, if necessary).

<figure markdown>
  ![](part5/action_msgs.png)
</figure>

This mechanism is therefore useful for robotic operations that may take a long time to execute, or where intervention might be necessary.

## Creating Action Servers in Python {#cam_swp_act_srv}

!!! info "Important"
    Cancel *all* active processes that you may have running before moving on.

So far we have looked at how to call an action server, but what about if we actually want to set up our own? We've been working with a pre-made action server in the previous exercises, but so far we haven't really considered how it actually works. First, let's do some detective work... We launched the Action Server using `ros2 launch` in Exercise 1:

```bash
ros2 launch tuos_examples camera_sweep.launch
```

!!! question "Questions"
    * What does this tell us about the *package* that the action server node belongs to?
    * Where, in the package directory, is this node likely to be located?
    * How might we find out the name of the Python node from the `camera_sweep.launch` file?

Once you've identified the name and the location of the source code, open it up in VS Code and have a look through it to see how it all works.

Don't worry too much about all the content associated with obtaining and manipulating camera images in there, we'll learn more about this in the next session. Instead, focus on the general overall structure of the code and the way that the action server is implemented.

1. As a starting point, consider the way in which the action server is initialised and the way a callback function is defined to encapsulate all the code that will be executed when the action is called:

    ``` { .python .no-copy }
    self.actionserver = actionlib.SimpleActionServer(self.server_name, 
        CameraSweepAction, self.action_server_launcher, auto_start=False)
    self.actionserver.start()
    ```

1. Look at how a `/cmd_vel` publisher and an `/odom` subscriber are defined in external classes:<a name="tb3_module" ></a>

    ``` { .python .no-copy }
    self.robot_controller = Tb3Move()
    self.robot_odom = Tb3Odometry()
    ```

    These are imported (at the start of the code) from an external `tb3.py` module that also lives in the same directory as the action server itself:

    ``` { .python .no-copy }
    from tb3 import Tb3Move, Tb3Odometry
    ```

    We do this to simplify the process of obtaining odometry data and controlling the robot, whilst keeping the actual action server code itself more concise. Have a look at the `tb3.py` module to discover exactly how these Python classes work.

1. Look inside the action server callback function to see how the camera sweep operation is performed once the action has been called:

    ``` { .python .no-copy }
    def action_server_launcher(self, goal):
        ...
    ```

    1. Consider the error checking that is performed on the `goal` input variables, and how the call to the action server is aborted should any of these goal requests be invalid:

        ``` { .python .no-copy }
        success = True
        if goal.sweep_angle <= 0 or goal.sweep_angle > 180:
            print("Invalid sweep_angle! Select a value between 1 and 180 degrees.")
            success = False
            ...

        if not success:
            self.result.image_path = "None [ABORTED]"
            self.actionserver.set_aborted(self.result)
            return
        ```

    1. Consider how preemption is implemented in the server, and how the Action is stopped on receipt of a preempt request:

        ``` { .python .no-copy }    
        if self.actionserver.is_preempt_requested():
            ...
        ```

    1. Also have a look at the way a `feedback` message is constructed and published by the server:

        ``` { .python .no-copy }
        self.feedback.current_image = i
        self.feedback.current_angle = abs(self.robot_odom.yaw)
        self.actionserver.publish_feedback(self.feedback)
        ```

    1. Finally, consider how we tell the server that the action has been completed successfully, how the `result` message is published to the caller, and how we make the robot stop moving:

        ``` { .python .no-copy }
        if success:
            rospy.loginfo("Camera sweep completed successfully.")
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
        ```

#### :material-pen: Exercise 4: Developing an "Obstacle Avoidance" behaviour using an Action Server {#ex4}

Knowing what you now do about ROS Actions, do you think the Service Server/Client systems that we developed in Part 4 were actually appropriate use cases for ROS Services?  Probably not!  In fact, *Action* Server/Client methods would have probably been more appropriate! 

You are now going to construct your own Action Server and Client nodes to implement a more effective obstacle avoidance behaviour that could form the basis of an effective search strategy. For this, you're going to need to build your own Search Server and Client.

**Step 1: Launch a simulation**

There's a simulation environment that you can use as you're developing your action server/client nodes for this exercise. Launch the simulation in **TERMINAL 1**, with the following `ros2 launch` command: 

***
**TERMINAL 1:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_stage_4.launch
```
***

**Step 2: Build the Action Server**

1. In **TERMINAL 2** navigate to the `src` folder of your `part5_actions` package, create a Python script called `search_server.py`, and make it executable.

1. The job of the Action Server node is as follows:

    * The action server should make the robot move forwards until it detects an obstacle up ahead.
    * Similarly to the *Service* Server that you created last part, your *Action* Server here should be configured to accept two **goal** parameters:
        1. The speed (in m/s) at which the robot should move forwards when the action server is called. Consider doing some error checking on this to make sure a velocity request is less than the maximum speed that the robot can actually achieve (0.26 m/s)!
        1. The distance (in meters) at which the robot should stop ahead of any objects or boundary walls that are in front of it. To do this you'll need to subscribe to the `/scan` topic. Be aware that an object won't necessarily be directly in front of the robot, so you may need to monitor a range of `LaserScan` data points (within the `ranges` array) to make the collision avoidance effective (recall the [LaserScan callback example](./part4/scan_callback.md) and also have a look at the `Tb3LaserScan` class within the `tuos_examples/tb3.py` module that might help you with this).
    * Whilst your server performs its task it should provide the following **feedback** to the Action Caller:
        1. The distance travelled (in meters) since the current action was initiated.

            To do this you'll need to subscribe to the `/odom` topic. Remember that there's a `Tb3Odometry` class within [the `tuos_examples/tb3.py` module](#tb3_module) that might help you with obtaining this data.
            
            Remember also that your robot's orientation shouldn't change over the course of a single action call, only its `linear.x` and `linear.y` positions should vary.  Bear in mind however that the robot won't necessarily be moving along the `X` or `Y` axis, so you will need to consider the total distance travelled in the `X-Y` plane.  You should have done this in the [Part 2 `move_square` exercise](./part2.md#ex5), so refer to this if you need a reminder.

    * Finally, on completion of the action, your server should provide the following *three* **result** parameters:
        1. The *total* distance travelled (in meters) over the course of the action.
        1. The distance to the obstacle that made the robot stop (this should match, or very close to, the distance that was provided by the Action Client in the **goal**).
        1. The angle (in degrees) at which this obstacle is located in front of the robot (`Tb3LaserScan` class within the `tuos_examples/tb3.py` module, which may already provide this).

1. An action message has been created for you to use for this exercise: `tuos_msgs/Search.action`.  Navigate to the `action` folder of the `tuos_msgs` package directory (or use `rosmsg info ...` in the terminal) to find out everything you need to know about this action message in order to develop your Action Server (and Client) nodes appropriately.

1. We've put together [some template code](./part5/search_server.md) to help you with this. For further guidance though, you should also refer to the code for `/camera_sweep_action_server` node, which [we talked about earlier](#cam_swp_act_srv): a lot of the techniques used by `/camera_sweep_action_server` node will be similar to what you'll need to do in this exercise. <a name="ex4_ret"></a>

1. Whenever you're ready you can launch your action server from **TERMINAL 2**, using `ros2 run`, as below:

    ***
    **TERMINAL 2:**
    ```bash
    ros2 run part5_actions search_server.py
    ```
    ***

**Step 3: Build the Action Client**

1. In **TERMINAL 3** navigate to the `src` folder of your `part5_actions` package, create a Python script called `search_client.py`, and make it executable.

1. The job of the Action Client node is as follows:

    * The client needs to issue a correctly formatted **goal** to the server.
    * The client should be programmed to monitor the **feedback** data from the Server.  If it detects (from the feedback) that the robot has travelled a distance *greater than 2 meters* without detecting an obstacle, then it should cancel the current action call using the `cancel_goal()` `actionlib` method.

1. Use the techniques that we used in the Client node from [Exercise 3](#ex3) as a guide to help you with this. There's also [a code template here](./part5/search_client.md) to help you get started. <a name="ex4c_ret"></a>

1. Once you have everything in place launch the action client with `rosrun` as below:

    ***
    **TERMINAL 3:**
    ```bash
    ros2 run part5_actions search_client.py
    ```
    ***

    If all is good, then this client node should call the action server, which will - in turn - make the robot move forwards until it reaches a certain distance from an obstacle up ahead, at which point the robot will stop, and your client node will stop too. Once this happens, reorient your robot (using the `turtlebot3_teleop` node) and launch the client node again to make sure that it is robustly stopping in front of obstacles repeatedly, and when approaching them from a range of different angles. 

    !!! info "Important"
        Make sure that your preemption functionality works correctly too, so that the robot never moves any further than 2 meters during a given action call!

## Some advanced exercises (if you're feeling adventurous!) {#advanced}

Want to do more with the ROS skills that you have now developed?! Consider the following advanced exercises that you could try out now that you know how to use ROS Actions!

!!! note
    We've covered a lot already in this session, and the next exercises are really just suggestions for more advanced things that you may want to explore to push your knowledge further (it may also help with the further work that you will do in Assignment #2...)

#### :material-pen: Advanced Exercise 1: Implementing a Search strategy {#adv_ex1}

What you developed in [the previous exercise](#ex4) could be used as the basis for an effective robot search strategy.  Up to now, your Action Client node should have the capability to call your `Search.action` server to make the robot move forwards by 2 meters, or until it reaches an obstacle (whichever occurs first), but you could enhance this further:

* Between action calls, your *client* node could make the robot turn on the spot to face a different direction and then issue a further action call to make the robot move forwards once again.
* The turning process could be done at random, or it could be informed by the **result** of the last action call, i.e.: if (on completion) the server has informed the client that it detected an object at an angle of, say, 10&deg; *anti-clockwise* from the front of the robot, then the client might then decide to turn the robot *clockwise* in an attempt to turn away from the object before issuing its next action call to make the robot move forwards again.
* By programming your client node to repeat this process over and over again, the robot would (somewhat randomly) travel around its environment safely, stopping before it crashes into any obstacles and reorienting itself every time it stops moving forwards. *This is effectively an implementation of a basic robotic search strategy!* 

    !!! tip "Enhancing this further..."
        Imagine SLAM was running at the same time too... your robot could be building up a map of its environment in the background as it slowly explored every part of it!

!!! success "Assignment #2 Checkpoint"
    Having completed Assignment #1 up to this point, you should have everything you need to tackle [Assignment #2 Task 2](../assignment2/parta/task2.md).

#### :material-pen: Advanced Exercise 2: Autonomous Navigation using waypoint markers {#adv_ex2}

In Part 3 you used SLAM to construct a map of an environment ([Exercise 2](./part3.md#ex2)) and then issued navigation requests to the `move_base` action server, via the command-line, ([Exercise 3](./part3.md#ex3)) to make your robot move to a zone marker, based on coordinates that you had established beforehand. Now that you know how to build Action Client Nodes in Python you could return to your `part2_navigation` package and build a new node that makes the robot move sequentially between each zone marker programmatically.

* Your node could cycle through the coordinates of all four of the zone markers (or "waypoints") that you established whilst using SLAM to build a map of the environment ([as per Exercise 2](./part3.md#ex2)).
* Your node could monitor the status of the `move_base_simple` action call to know when the robot has reached a zone marker, so that it knows when to issue a further action call to move on to the next one.
* You could refer to [the launch file that you created in Part 3](./part3.md#launch_file) to launch all the navigation processes that need to be running in order to enable and configure the ROS Navigation Stack appropriately for the TurtleBot3 robot.

## Wrapping Up

In Part 5 of this course you've learnt:

* How ROS Actions work and why they might be useful.
* How to develop Action Client Nodes in Python which can perform other tasks *concurrently* to the action they have requested, and which can also *cancel* the requested action, if required.
* How to use standard ROS tools to interrogate the topic messages used by an action server, allowing you to build clients to call them, and to also allow you to build standalone action servers yourself using bespoke Action messages.
* How to harness this communication method to implement a behaviour that could be used as the basis for a genuine robotic *search strategy*. 

### Topics, Services or Actions: *Which to Choose?*

You should now have developed a good understanding of the three communication methods that are available within ROS to facilitate communication between ROS Nodes:

1. Topic-based messaging.
1. ROS Services.
1. ROS Actions.

Through this course you've gained some practical experience using all three of these, but you may still be wondering how to select the appropriate one for a certain robot task... 

[This ROS.org webpage](https://wiki.ros.org/ROS/Patterns/Communication#Communication_via_Topics_vs_Services_vs_X) summarises all of this very nicely (and briefly), so you should have a read through this to make sure you know what's what. In summary though:

* **Topics**: Are most appropriate for broadcasting continuous data-streams such as sensor data and robot state information, and for publishing data that is likely to be required by a range of Nodes across a ROS network.
* **Services**: Are most appropriate for very short procedures like *quick* calculations (inverse kinematics etc.) and performing short discrete actions that are unlikely to go wrong or will not need intervention (e.g. turning on a warning LED when a battery is low).
* **Actions**: Are most appropriate for longer running tasks (like moving a robot), for longer processing calculations (processing the data from a camera stream) or for operations where we *might* need to change our mind and do something different or cancel an invoked behaviour part way through.