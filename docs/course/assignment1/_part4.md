---  
title: "Part 4: Services"  
description: Learn about an alternative way that ROS nodes can communicate across a ROS network, and the situations where this might be useful.  
---

## Introduction

:material-pen: **Exercises**: X  
:material-timer: **Estimated Completion Time**: Y hours

### Aims

In this part you'll learn about *Services*: an alternative communication method that can be used to transmit data/information or invoke actions on a ROS Network. You'll learn how this works, and why it might be useful. You'll also look at some practical applications of this.

### Intended Learning Outcomes

By the end of this session you will be able to:

1. Recognise how ROS Services differ from the standard topic-based publisher-subscriber approach, and identify appropriate use-cases for this type of messaging system.
1. Identify the services that are available on a ROS network, and use ROS command-line tools to interrogate and *call* them.
1. Develop Python Service Client Nodes.
1. Invoke different services using multiple service-type interfaces.

### Quick Links

* [Exercise 1: Calling a Service from the Command-line](#ex1)
* [Exercise 2: ](#ex2)
* [Exercise 3: ](#ex3)
* [Exercise 4: ](#ex4)

### Additional Resources

* [TODO]()

## Getting Started

**Step 1: Launch your ROS2 Environment**

If you haven't done so already, launch your ROS environment now. Having done this, you should now have access to a Linux terminal instance (aka **TERMINAL 1**).

**Step 2: Restore your work (WSL-ROS2 Managed Desktop Users ONLY)**

Remember that any work you do within the WSL-ROS2 Environment will not be preserved between sessions or across different University computers, so you should be backing up your work to your `U:\` drive regularly. When prompted (on first launch of WSL-ROS2 in **TERMINAL 1**) enter ++y+enter++ to restore your data[^1].

``` { .txt .no-copy }
It looks like you already have a backup from a previous session:
  U:\wsl-ros\ros2-backup-XXX.tar.gz
Do you want to restore this now? [y/n]
```

[^1]: Remember: you can also use the `wsl_ros restore` command at any time.

**Step 3: Launch VS Code**  

It's also worth launching VS Code now. *WSL users* remember to check for this:

<figure markdown>
  ![](../../ros/figures/code-wsl-ext-on.png){width=400px}
</figure>

**Step 4: Make Sure The Course Repo is Up-To-Date**

Once again, it's worth quickly checking that the Course Repo is up-to-date before you start on the Part 4 exercises. Go back to [Part 1](./part1.md#course-repo) if you haven't installed it yet (really?!) or - alternatively - [see here for how to update](../extras/course-repo.md#updating).

## An Introduction to Services

So far, we've learnt about ROS *topics* and the *message*-type interfaces that we use to transmit data on them. We've also learnt how individual nodes can access data on a robot by simply *subscribing* to topics that are being published by any other node on the system. In addition to this, we also know that any node can *publish* messages to any topic: this essentially broadcasts the data across the ROS Network, making it available to any other node on the network that may wish to access it.

Another way to pass data between ROS Nodes is by using *Services*. These are based on a *call and response* type of communication:

* A Service **Client** sends a **Request** to a Service **Server**.
* The Service **Server** processes that request and sends back a **Response**.

<figure markdown>
  ![The difference between topic-based messaging and the ROS Service protocol](part4/topic_vs_service.png)
</figure>

This is a bit like a transaction: one node requests something, and another node fulfils that request and responds, and this is good for **quick, short duration tasks**, e.g.:

1. Turning a device on or off.
1. Grabbing some data and saving it to a file.
1. Performing a calculation and returning a result.
1. Making a sound[^tb3_sound].

[^tb3_sound]: On the real Waffles, there's a service called `/sound`. Have a look at this next time you're in the lab... Once you've worked through the whole of Part 4 you'll know exactly how to interrogate this service and leverage the functionality that it provides!

A single service can have many clients, but you can only have a *single* Server providing that particular service at any one time.

<figure markdown>
  ![](part4/service_clients.png)
  <figcaption>Multiple Clients to a single Service Server</figcaption>
</figure>

Let's see how this all works in practice now, by playing a number game! We don't need a simulation up and running for this one, so in **TERMINAL 1** use the following command to launch the *Guess the Number* Service: 

***
**TERMINAL 1:**
```bash
ros2 run tuos_examples number_game.py
```

Having launched the service successfully, you should be presented with the following:

``` { .txt .no-copy }
[INFO] [#####] [number_game_service]: The '/guess_the_number' service is active.
[INFO] [#####] [number_game_service]: A magic number has been set... Game on!
```
***

We need to interrogate this now, in order to work out how to play the game...

### Interrogating a Service

#### :material-pen: Exercise 1: Using Command-line Tools to Interrogate a Service and its Interface {#ex1}

1.  Open up a new terminal instance (**TERMINAL 2**) and use the `ros2 service` command to list all active ROS services:

    ***
    **TERMINAL 2:**
    ```bash
    ros2 service list
    ```
    ***

    There'll be a few items in this list, most of them with the prefix: `/number_game_service`. This is the name of the *node* that is providing the service (i.e. the **Server**) and these items are all automatically generated by ROS. What we're really interested in is the service itself, which should be listed as: `/guess_the_number`. 

1. Next, we need to find the interface type used by this service, which we can do a couple of ways:

    ***
    **TERMINAL 2:**
    
    1. Use the `type` sub-command:

        ```bash
        ros2 service type /guess_the_number
        ```

    1. Use the `list` sub-command again, but with the `-t` flag:

        ```bash
        ros2 service list -t
        ```

        The latter will provide the same list of services as before, but each one will now have its interface type provided too.

    ***

1. Regardless of the method that you used, you should have identified that the interface type used by the `/guess_the_number` service is:
    
    ``` { .txt .no-copy }
    tuos_interfaces/srv/NumberGame
    ```

    Notice how ([much like with the interface types used by *topics*](./part1.md#msg-interface-struct)), there are three fields to this again:
        
    1. `tuos_interfaces`: the name of the ROS package that this interface belongs to.
    1. `srv`: that this is a *service* message (the second interface type we've covered now, and we'll learn about the third and final in Part 5).
    1. `NumberGame`: the message data structure.

    We need to know the data structure in order to make a call to this service, so let's identify this next.

1. We can use the `ros2 interface list` command to list *all* interface types available to us on our ROS system, but this will provide us with a long list!

    ***
    **TERMINAL 2:**

    1. We can use the `-m` flag to filter for *message* interfaces, or the `-s` flag to filter for *service* interfaces. Try the latter:

        ```bash
        ros2 interface list -s
        ```
    
    1. Still quite a lot there, right!? Let's filter this further with `grep` to identify *only* interfaces from the `tuos_interfaces` package:

        ```bash
        ros2 interface list -s | grep tuos_interfaces
        ```

        Hopefully, the `srv/NumberGame` interface is now listed.

    1. Use `ros2 interface show` sub-command to *show* the message structure:

        ```bash
        ros2 interface show tuos_interfaces/srv/NumberGame
        ``` 
        
    ***

    The service message structure should be shown as follows:

    ``` { .txt .no-copy }
    int32 guess
    ---
    int32 guesses
    string hint
    bool success
    ```

### The Format of a Service Message

Service interfaces have two parts to them, separated by three hyphens (`---`). Above the separator is the Service **Request**, and below it is the Service **Response**:

``` { .txt .no-copy }
int32 guess      <-- Request
---
int32 guesses    <-- Response (1 of 3)
string hint      <-- Response (2 of 3)
bool success     <-- Response (3 of 3)
```

In order to *Call* a service, we need to provide data to it in the format specified in the **Request** section of the interface. A service *Server* will then send data back to the caller in the format specified in the **Response** section of the interface.

The `tuos_interfaces/srv/NumberGame` service interface has only **one** request parameter:

1. An `int32` (32-bit integer) called `guess`  
    ...which is the only thing we need to send to the `/number_game_service` Service Server in order to call it.

There are then **three** response parameters:

1. A *32-bit integer* called `guesses`
1. A text *string* called `hint`  
1. A *boolean* flag called `success`

    ...all of which will be returned by the server, once it has processed our request.

#### :material-pen: Exercise 2: Playing the Number Game (from the Command-line) {#ex2}

We're now ready to make a call to the service, and we can do this using the `ros2 service` sub-command (from **TERMINAL 2**):

1. To start, let's send an initial guess of `0` and see what happens:

    ```bash
    ros2 service call /guess_the_number tuos_interfaces/srv/NumberGame "guess: 0"
    ```

    The request will be echoed back to us, followed by a response, which will likely look something like this and which shows us the valuer of the three response parameters that we identified above:

    ``` { .txt .no-copy }
    response:
    tuos_interfaces.srv.NumberGame_Response(guesses=1, hint='Higher', success=False)
    ```

    1. `guesses`: tells us how many times we've tried to guess the number, in total (just once so far)
    1. `hint`: tells us if our next guess should be "higher" or "lower" in order to get us to the magic number
    1. `success`: indicates if we guessed the right number or not (unlikely on the first attempt!)

1. Make another service call, this time changing the value of your `guess`, e.g.:
    
    ```bash
    ros2 service call /guess_the_number tuos_interfaces/srv/NumberGame "guess: 10"
    ```

1. Try making a guess of 500 next.

    The service should respond with the hint `'Error'`. Have a look back in **TERMINAL 1** (where the Server is running) to get more information on this.

1. Keep going until you guess the magic number, how many guesses does it take you?!

    ??? tip "Hint"
        There are also a couple of *cheat codes* that you can send as guesses, that will make the server to tell you what the magic number actually is...[^cheat-codes] 

    [^cheat-codes]: Check the `tuos_examples/number_game.py` Python code to find out what the cheat codes are!! 
    
## Wrapping Up

In Part 4 you have learnt about ROS Services and why they might be useful for robot applications:

* Services differ from standard topic-based communication methods in ROS in that they are a direct form of communication between one node and another.  
* The communication between the two nodes is sequential or *synchronous*: once a service *Caller* has *called* a service, it must wait until it has received a *response*.
* This is useful for controlling *quick*, *short-duration* tasks or for *offloading computations* (which could perhaps also be considered *decision-making*).

Having completed all the exercises above, you should now be able to:

* Create and execute Python Service *Servers*.
* Create and execute Python Service *Callers*, as well as call services from the command-line.
* Implement these principles with a range of different service message types to perform a number of different robot tasks.
* Use LiDAR data effectively for basic closed-loop robot control.
* Develop Python nodes which *also* incorporate principles from Parts 1, 2 & 3 of this course:
    * Publishing and subscribing to topics.
    * Controlling the velocity and position of a robot.
    * Using the Python Class architecture.
    * Harnessing ROS and Linux command-line tools.
    
### WSL-ROS2 Managed Desktop Users: Save your work! {#backup}

Remember, to save the work you have done in WSL-ROS2 during this session so that you can restore it on a different machine at a later date. Run the following script in any idle WSL-ROS2 Terminal Instance now:

```bash
wsl_ros backup
```

You'll then be able to restore it to a fresh WSL-ROS2 environment next time you fire one up (`wsl_ros restore`).  