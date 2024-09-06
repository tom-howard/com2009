---  
title: "A Simple Publisher Node"
---

## The Code

Copy **all** the code below into your `publisher.py` file and **review the annotations** to understand how it all works.

!!! tip
    **Don't forget the Shebang!** See [below](#shebang) for further details...

```python title="publisher.py"
--8<-- "code_templates/publisher.py"
```

1. `rclpy` is the *ROS Client Library for Python*. 
    
    This is a vital import that allows us to create ROS nodes and initialise them on the ROS network.
    
    We also import the `Node` class from the `rclpy.node` library. This is a ready-made Python Class that contains all the necessary functionality that a Python ROS Node might need, so we'll use this as the basis for our own node (which we'll create shortly).

2. We also need to import the `String` message type from the `example_interfaces.msg` library for publishing our messages.

3. We create a *Python class* called `#!python SimplePublisher()`, which we'll use to encapsulate all the functionality of our node.
    
    The vast majority of the functionality of this node is inherited from the `rclpy.node`, `Node()` Class which we imported above. 

4. Using the `#!python super()` method we call the `#!python __init__()` method from the parent Node class that our `SimplePublisher` class is derived from.
    
    We provide a *name* here, which is the name that will be used to register our node on the ROS network (we can call the node anything that we want, but it's a good idea to call it something meaningful).

5. We then use the `#!python create_publisher()` method (inherited from the `Node` class) in order to provide our node with the ability to publish messages to a ROS Topic. When calling this we provide 3 key bits of information:

    1. `msg_type`: The **type** of message that we want to publish.
        
        In our case, a `String` message from the `example_interfaces.msg` module.
    
    1. `topic`: The **name of the topic** that we want to publish these messages to.
        
        This could be an existing topic (in which case, we'd need to make sure we used the correct message type), or a new topic (in which case, the name can be anything we want it to be).
        
        In our case, we want to create a new topic on the ROS network called `"my_topic"`.
    
    1. `qos_profile`: A **queue size**, which is a *"Quality of Service"* (QoS) setting which limits the amount of messages that are *queued* in a buffer. 
    
        In our case, we're setting this to `10`, which is generally appropriate for most of the applications that we'll be working on.

6. Here, we're calling the `#!python create_timer()` method, which we'll use to control the rate at which messages are published to our topic. Here we define 2 things:

    1. `timer_period_sec`: The rate at which we want the timer to run. This must be provided as a *period*, in seconds. In the line above, we have specified a publishing *frequency* (in Hz):
    
        <center>`#!python publish_rate = 1 # Hz`</center>
        
        So the associated time *period* (in seconds) is: 

        <center>$T = \frac{1}{f}$</center>

    1. `callback`: This is a function that will be executed every time the timer elapses at the desired rate (1 Hz). We're specifying a function called `timer_callback`, which we'll define later on in the code...

7. Finally, we use the `#!python get_logger().info()` method to send a *Log* message to the terminal to inform us that the initialisation of our node is complete.

8. Here we define the timer callback function. Anything in here will execute at the rate that we specified when we created the `#!python create_timer()` instance before. In our case:

    1. Use the `#!python get_clock()` method to get the current *ROS Time*.
    1. Instantiate a `String()` message (defined as `topic_msg`).
    1. Populate this message with *data*. In our case, a statement that includes the ROS Time, as obtained above.
    1. Call the `#!python publish()` method of our `my_publisher` object, to actually publish this message to the `"my_topic"` topic.
    1. Send the message data to the terminal as a log message as well, so that we can see what it is when our Node is actually running.

9. With the functionality of our `SimplePublisher` class now established, we define a `#!python main()` function for the Node. This will be fairly common to most Python Nodes that we create, with the following 5 key processes:

    1. Initialise the `rclpy` library.
    1. Create an instance of our `#!python SimplePublisher()` node.
    1. "Spin" the node to keep it alive so that any callbacks can execute as required (in our case here, just the `#!python timer_callback()`). 
    1. Destroy the node once termination is requested (triggered by entering ++ctrl+c++ in the terminal).
    1. Shutdown the `rclpy` library.

10. Finally, we call the `#!python main()` function to set everything going. We do this inside an `#!python if` statement, to ensure that our node is the *main executable* (i.e. it has been executed directly (via `ros2 run`), and hasn't been called by another script)

## Defining Package Dependencies

We're importing a couple of Python libraries into our node here, which means that our package has two *dependencies*: `rclpy` and `example_interfaces`:

```py
import rclpy 
from rclpy.node import Node

from example_interfaces.msg import String
```

Its good practice to add these dependencies to your `package.xml` file. Locate this file (`ros2_ws/src/part1_pubsub/package.xml`), open it up and find the following line:

```xml
<exec_depend>rclpy</exec_depend>
```

`rclpy` is therefore already defined as an *execution dependency* (which means that our package needs this library in order to execute our code), but we need to add `example_interfaces` as well, so add the following additional line underneath:

```xml
<exec_depend>example_interfaces</exec_depend>
```

Job done. Save the file and close it.

## The Shebang {#shebang}

The very first line of code looks like a comment, but it is actually a very crucial part of the script:

```python
#!/usr/bin/env python3
```

This is called the *Shebang*, and it tells the operating system which interpreter to use to execute the code. In our case here, it tells the operating system where to find the right *Python interpreter* that should be used to actually run the code.
    
<p align="center">
  <a href="../../part1#pub_ret">&#8592; Back to Part 1</a>
</p>