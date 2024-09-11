---  
title: "Part 4: ROS 2 Services"  
description: Learn how ROS Services can be combined with standard publisher/subscriber principles to enhance robot control for specific operations and gain the ability to create custom msgs and srv for tailored communication
---

## Introduction 

:material-pen: **Exercises**: X

:material-timer: **Estimated Completion Time**: Y hours

### Aims
In this part you will learn about ROS Services, a communication method that facilitates request-response interactions between nodes. You will understand how to use ROS services in combination with standard publisher/subscriber principles to enhance control for specific operations. Additionally, you'll create custom messages and services for tailored communication.

### Intended Learning Outcomes 
By the end of this session you will be able to:

1. Recognise how ROS Services differ from the standard topic-based publisher-subscriber approach, and identify appropriate use-cases for this type of messaging system.
1. Implement Python node pairs to observe services in action, and understand how they work.
1. Invoke different services using a range of service message types.
1. Develop Python Service nodes of your own to perform specific robotic tasks.
1. Harness Services, in combination with LiDAR data, to implement a basic obstacle avoidance behaviour 
1. Develop custom ROS messages and services (still need to think about the task for this) 
1. Demonstrate your understanding of ROS2 so far by developing a Python node which incorporates elements from this and previous parts of this course.


### Quick Links
* [Exercise 1: ](#ex1)

### Additional Resources
### Prerequisites
Before we begin, ensure that you have the following:

1. ROS2 Humble installed on your system
1. Cloned the tuos package from github
1. Basic understanding of ROS2 concepts like nodes and topics

### Getting Started
**Step 1: Launch your ROS Environment**

If you haven't done so already, launch your ROS environment now:

**Step 2: Restore your work (todo)**

**Step 3: Launch VS Code (todo)**  

**Step 4: Make Sure The Course Repo is Up-To-Date**

Once again, it's worth quickly checking that the Course Repo is up-to-date before you start on the Part 4 exercises. Go back to [Part 1](./part1.md#course-repo) if you haven't installed it yet (really?!). For the rest, run the following commands:
```bash
cd ~/ros2_ws/src/tuos_ros/ && git pull
```

Now build with colcon:
```bash
cd ~/ros2_ws/ && colcon build 
```
Finally, re-source the environment 
```bash
source ~/.bashrc
```

## Step 5: Launch the Robot Simulation 
From **TERMINAL 1**, launch the TurtleBot3 Waffle *"Empty World"* 
simulation:

***
**TERMINAL 1:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch
```
...and then wait for the Gazebo window to open:

<figure markdown>
</figure>

## An Introduction to Services

So far, we've learnt about ROS *topics* and *messages*, and how individual nodes can access data on a robot by simply *subscribing* to topics that are being published by any other node on the system.  In addition to this, we also learnt how any node can *publish* messages to any topic: this essentially broadcasts the data contained in the message across the ROS Network, making it available to any other node on the network that may wish to access it.

ROS2 uses interface as a communication structure that allow different nodes to exchange data. These interfaces are broadly categorized into three types:

1. Messages
2. Services
3. Actions

We have already learned how to use `Messages` in [part 2](./part2.md#ros-velocity-commands) and now we will learn in detail about `ROS2 Services`.   

These are different to messages in that *"Service calls"* (that is, the process of requesting a service) occur *only* between one node and another:

* One node (a Service **Client**) sends a **Request** to another node.
* Another node (a Service **Server**) processes that request, performs an action and then sends back a **Response**.

<figure markdown>
  ![The difference between topic-based messaging and the ROS Service protocol](../../images/part4/Service-SingleServiceClient.gif)
</figure>

Services are *Synchronous* (or *sequential*): When a ROS node sends a request to a service (as a Service *Client*) it can't do anything else until the service has been completed and the Service *Server* has sent a response back. This can be useful for a few reasons:

* There can be multiple service clients using the same service but only one service server for a service. 

1. **Discrete, short-duration actions**: A robot might need to do something before it can move on to something else, e.g.:
    
    * A robot needs to see something before it can move towards it.
    * High definition cameras generate large amounts of data and consume battery power, so you may wish to turn a camera on for a specific amount of time (e.g. until an image has been captured) and then turn it off again.

2. **Computations**: Remember that ROS is *network-based*, so you might want to offload some computations to a remote computer or a different device on a robot, e.g.:
    
    * A client might send some data and then wait for another process (the server) to process it and send back the result.

It's also worth noting that any number of ROS Client nodes can call a service, but you can only have a *single* Server providing that particular service at any one time.

<figure markdown>
  ![](../../images/part4/Service-MultipleServiceClient.gif)
</figure>

!!! question
    Can you think of any other scenarios where this type of communication protocol might be useful?

#### :material-pen: Exercise 1: Creating a Service *Server* in Python and calling it from the command-line {#ex1}

To start with, let's set up a service and learn how to make a call to it from the command-line to give you an idea of how this all works and why it might be useful.

1. First open up a new terminal instance (**TERMINAL 2**) and source your ROS2 environment as you did in [part 1](./part1.md#first-steps): 

    1. Now navigate into the Course Repo `ros2_ws/src/tuos_ros` and run the helper script agian as you did in [part 1](./part1.md#first-steps) to create a new package called `part4_services`:
    ***    
    **TERMINAL 2:**
        ```bash
        ./create_pkg.sh part4_services        
        ```
    ***

    Your terminal will return a message verifying the creation of your package.

1. Navigate into the new package directory using cd:

    ***
    **TERMINAL 2:**

    ```bash
    cd ../part4_services/
    ```
    ***

1. Then navigate into the `scripts` folder in the package directory (using cd) and create an empty file called `move_server.py` using `touch` command.
    ```bash
    touch move_server.py
    ```
    
1.  Open the file in VS Code, copy and paste [this code](./part4/move_server.md) and then save it. <a name="ex1_ret"></a> (todo: need to add the template)

    !!! note
        It's really important that you understand how the code above works, so that you know how to build your own service *Servers* in Python. Also, make sure that the `move_server` file is executable using `chmod +x` command. 

1. Next, we need to add our `move_server.py` file as an executable to our package's `CMakeLists.txt`. This will ensure that it then gets built when we run `colcon build` (in the next step):

    In VS Code, open the `CMakeLists.txt` file that is at the root of your `part4_services` package directory (`ros2_ws/src/part4_services/CMakeLists.txt`). Locate the lines (near the bottom of the file) that read:

    ``` { .txt .no-copy}
    # Install Python executables
    install(PROGRAMS
      scripts/minimal_node.py
      DESTINATION lib/${PROJECT_NAME}
    )
    ```

    Replace `minimal_node.py` with `move_server.py` to define this as a Python executable in your package:

    ``` { .txt .no-copy }
    # Install Python executables
    install(PROGRAMS
      scripts/publisher.py
      DESTINATION lib/${PROJECT_NAME}
    )
    ```
    i.  It's a good practice to run `rosdep` in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    ```

1. Then, use Colcon to build your new package and its contents :

    ***
    **TERMINAL 2:**
    ```bash
    cd ~/ros2_ws/ && colcon build --packages-select part4_services --symlink-install
    ```
    i. Finally re-source your `bashrc`: 
    ```bash
    source ~/.bashrc
    ```
    ***
    
1. Now, we are ready to run the node. Use `ros2 run` and observe what is displayed on the terminal 
   
    ***
   **TERMINAL 2:**
    ```bash
    ros2 run part4_services move_server.py
    ```
    You should see this message: 
    ```{ .txt .no-copy}
    The 'move_service' server is ready to be called...
    ```
   ***

1. Open a new terminal window (**TERMINAL 3**) 

1. While the node is running, use `ros2 service` command to view all the currently active services on the system:

    ***
    **TERMINAL 3:**
    ```bash
    ros2 service list
    ```
    You should see the `/move_service` service that we defined in the Python code:
    ``` { .txt .no-copy}
    service_name = "move_service"
    ```
    ***

1. We can find out more about this using the `ros2 service type` command:
    
    ***
    **TERMINAL 3:**
    ```bash
    ros2 service type /move_service 
    ```
    Which should provide the following output:
    
    ``` { .txt .no-copy}
    std_srvs/srv/SetBool 
    ```

    This shows that the `move_service_server` node is using `SetBool` service (or interface) type defined in the `std_srv` package

    !!! tip 
        You can also view the type of all services at the same time by adding `-t` to the `ros2 service list` command.

1. We can also call this service from the command line using `ros2 service call`.<a name="cl_call"></a> 
    ```bash
    ros2 service call <service_name> <service_type> <arguments> 
    ```
    
    In this case, 
    ```bash
    ros2 service call /move_service std_srvs/srv/SetBool "data: false"
    ```
    
1. Press ++enter++ to issue the command and make a call to the service. 
    You should see the following response:
    ``` { .txt .no-copy }
    requester: making request: std_srvs.srv.SetBool_Request(data=False)

    response:
    std_srvs.srv.SetBool_Response(success = False, message="Nothing happened, set request_signal to 'true' next time.")
    ```

1. Arrange the terminals 1 and 3 so that you can see both the Gazebo simulation and the terminal that you just issued the `ros2 service call` command in.

1. In **TERMINAL 3** enter the `ros2 service call` command again, but this time set the `data` input to `true`. You should be able to see the response of the robot in Gazebo simulation. Switch back to **TERMINAL 2** and observe the terminal output there as well. 

**Summary**

In the section above, you learned how to create a Service Server node. This node sits idle and keeps waiting for its service to be called. Then you called the service through command line which prompted the Server to carry out the following tasks defined in the Python code,
    1. Start a timer
    1. Issue a velocity commnad to the robot to move it forward
    1. Wait for 5 seconds
    1. Issue a velocity command to stop the robot
    1. Get the service **Response** and issue it as an output to the terminal in which the service is called

***
###Understanding key features of **ros2 service** 

In Part 2, you learned how to find out more about a particular message type, using the `ros2 interface show` command. You can do the same to find out the details of service type as follow: 

***
**Terminal 3**
```bash
ros2 interface show std_srvs/srv/SetBool
```
which will give the following output: 

``` { .txt .no-copy }
bool data   # e.g. for hardware enabling / disabling 
---
bool success     # indicate successful run of triggered service
string message   # informational, e.g. for error messages
```
### The Format of SetBool Service 
The service above is structured in two parts separated by three hyphens (`---`). The part above the hyphens is called the Service **Request** while the part below is Service **Response**:

``` { .txt .no-copy }
bool data        <-- Request 
---
bool success     <-- Response (Parameter 1 of 2)
string message   <-- Response (Parameter 2 of 2)
```
***

In order to *Call* a service, we need to provide data to it in the format specified in the **Request** section. A service *Server* (like the [Python node we created above](./part4/move_server.md#code)) will then send data back to the caller in the format specified in the **Response** section.

The `std_srvs/srv/SetBool` service that we're working with here has **one** request parameter:

1. A *boolean* input called `data`  
    ...which is the only thing we need to send to the Service Server in 
order to call the service.

There are then **two** response parameters:

1. A *boolean* flag called `success`
1. A text *string* called `message`  
    ...both of these will be returned to the client, by the server, once 
the Service has completed.



#### :material-pen: Exercise 2: Creating a Python Service *Client* Node {#ex2}

Instead of calling a service from command-line we can also build Python Service *Client* Nodes to do the same. In this exercise you will learn how this is done.

1. **TERMINAL 3** should be idle, so from here navigate to the `scripts` folder within the `part4_services` package that we created earlier:

    ***
    **TERMINAL 3:**
    ```bash
    cd ~/ros2_ws/src/part4_services/scripts
    ```
    ***

1. Create a new file called `move_client.py`
1. Now as you did in the previous exercise, open the VS Code, copy and paste [this code](./part4/move_client.md) and then save it. <a name="ex2_ret"></a>

    !!! note
        Once again, be sure to read the code and understand how this Python Service Client Node works too!

1.  Return to **TERMINAL 3** and launch the node using `ros2 run`:

    ```bash
    ros2 run part4_services move_client.py 
    ```

***
The response should be exactly the same when we called the service from the command line. 

#### :material-pen: Exercise 3: Learn to create custom services {#ex3}
In previous exercises you learned about messages, topics and services by using the predefined definitions of them. While using predefined interfaces is considered a good practice, it is also important to know how you can custom define these interfaces based on your own need. This exercise will teach you, step-by-step, how to create custom service definition and use it to move the robot to the requested position (providing x and y coordinates).

**Procedure**  

1. Close down the Service Server that is currently running in **TERMINAL 2**
1. Navigate to your `part4_services` package 
    
    ***
    **TERMINAL 2:**
    ```bash
    cd ~ros2_ws/src/part4_services
    ```
and make a new directory `srv` by running the following command:
    ```bash
    mkdir srv
    ```
    ***

1. Now navigate into the newly created directory `srv` and create new file called `MoveToPosition.srv`

    !!!note 
        It is important that your file name should end with `.srv` extension as this identifies the file as a ROS service.

1. As we learned earlier, a service file consists of two parts: `Request` and `Response`. Here we will provide our own definition for each one of these parts as follow:

    ```bash
    float32 goal_x      <-- request parameter 1 of 2
    float32 goal_y      <-- request parameter 2 of 2
    ---
    bool success        <-- response 
    ```
 The service takes in two user inputs `goal_x` and `goal_y` of type `float` for the `x` and `y` coordinates to where the robot needs to move.
***

1. Open the VS Code, copy and paste the above lines and save the file. 
1. We need to add a few lines in the `CMakeList.txt` to convert the defined service into language-specific code (C++ and Python) and make it usable:

    ```bash 
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/MoveToPosition.srv"
    )
    ```

1. Now open the `Package.xml` file and add the following lines:
    ```bash 
    <build_depend>rosidl_default_generators></build_depend>
    <exec_depend>rosidl_default_runtime></exec_depend>
    <member_of_group>rosidl_interface_packages></member_of_group>
    ```
    These lines specify the dependencies required to run the custom service. 
    
    ***

1. Next, navigate to the `scripts` folder of the `part4_services` package and create an empty file called `MoveToPosition.py`. 

1. Open the newly created file in VS Code. Copy and paste the code provided [here](./part4/MoveToPosition.md). 
1. Now modify the code as follow:

    1. Change the imports to utilise the service we just created 
    1. Develop the callback_function() to: 
        1. Process the **two** parameters that will be provided to the server via the `service request
        1. Retrieve the current position and calculate the difference to goal
        1. Generate movement command to the specific coordinates
        1. Return a correctly formatted service response message to the service caller
    1. Launch the server node using `ros2 run` command from **TERMINAL 2** and `call` the service from the command-line using `ros2 service call` in **TERMINAL 3** [as you did earlier](#cl_call)

1. Make sure you build the package in the root directory of ros2 workspace using `colcon` and source the environment: 

    ??? tip 
        For convenience, you can use a handy alias `src` instead of writing the whole `source ~/.bashrc`

***

## A recap on everything you've learnt so far...

You should now hopefully understand how to use the ROS2 Service architecture and understand why, and in what context, it might be useful to use this type of communication method in a robot application.

!!! tip "Remember"
    Services are **synchronous** and are useful for one-off, quick actions or for offloading jobs or computations that might need to be done before something else can happen. (Think of it as a transaction that you might make in a shop: You hand over some money, and in return you get a chocolate bar, for example!)

#### :material-pen: Exercise 4: Creating your own Service {#ex4}

In this exercise you will create your own service Server to make the Waffle perform a specific movement for a given amount of time and then stop.

