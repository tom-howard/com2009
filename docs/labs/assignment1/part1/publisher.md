---  
title: A Simple Publisher Node (Part 1)
---

## The Code

Copy **all** the code below into your `publisher.py` file and *review the annotations* to understand how it all works.

!!! tip
    **Don't forget the Shebang!** See [below](#shebang) for further details...

```python title="publisher.py"
--8<-- "code_templates/publisher.py"
```



<!-- 

1. `rospy` is the *Python client library for ROS*, and we need to import this in order to create ROS Nodes in Python.

2. We also need to import the `String` message type from the `std_msgs.msg` library for publishing our messages.

3. We create a *Python class* called `Publisher()` to encapsulate all the functionality of our node.

4. The `__init__()` method is called as soon as an instance of the `Publisher()` class is created.

5. We define a name for this node and assign it to `self.node_name`. We can call the node anything that we want, but it's good to give it a meaningful name as this is the name that will be used to register the node on the ROS Network.

6. We also define the name of a topic that we want to publish messages to (`"chatter"` in this case). If this is the name of a topic that already exists on the ROS Network, then we need to ensure that we use the correct message type, in order to be able to publish to it. In this case however, a topic called "chatter" shouldn't currently exist, so it will be created for us, and we can choose whatever type of message we want to use (a `String` message from the `std_msgs` library in our case).

7. We then create an instance of a `#!python rospy.Publisher()` object within our class: this creates the topic on the ROS Network (if it doesn't already exist). We therefore need to tell it the name of the topic that we want to create, and we also need to specify that we will be publishing `String` type messages.

8. Then, we initialise our Python node, using the name that we defined earlier (`"simple_publisher"`), setting the `anonymous` flag to `True` to ensure that the node name is unique, by appending random numbers to it (we'll observe this when we run the node shortly).

9. Then, we instantiate a `rospy.Rate()` object and set the frequency to 10 Hz, so that our publisher will publish messages at this frequency.

10. This is used to shut down a ROS node effectively:

    1. First, we create a `ctrl_c` variable within the parent class and initialise it to `False`.
    1. Then, we use the `rospy.on_shutdown()` method to register a shutdown *hook* (in this case a function called `shutdownhook`). This will be called when rospy detects that the node has been asked to stop (i.e. by a user entering `Ctrl+C` in the terminal, for example). **The shutdown hook function must take no arguments**.

11. Finally, we issue a message to indicate that our node is active (this will appear in the terminal that we run the node in):

12. This method is called by the `rospy.on_shutdown()` method when the node is stopped. 

    Here, we can include any important shutdown processes (making a robot stop moving, for instance). In this case, we just print a message to the terminal and then set the `ctrl_c` variable to `True`, which will stop the `main()` method...

13. The `while` loop here makes sure that the `main()` runs continuously, until the node is shut down (via the `self.ctrl_c` flag):

    Inside the `while` loop we create a publisher message (a simple string in this case), publish it using the `pub` object we created in the initialisation stage, and then use the `rate` object (also created earlier) to then make the node "sleep" for as long as required to satisfy the frequency that we defined earlier.

14. This `__name__` check, ensures that our node is the main executable (i.e. it has been executed directly (via `rosrun`), and hasn't been called by another script):

15. We create an instance of the `Publisher` class that we created above (which executes the `__init__` method automatically). 

16. We call the `main()` of our `Publisher()` class to execute the core functionality of the node. 

    We wrap this inside a `try-except-pass` statement to look for a `rospy.ROSInterruptException` error, which can be output by rospy when the user presses `Ctrl+C` or the node is shutdown in some other way. -->

## The Shebang {#shebang}

The very first line of code looks like a comment, but it is actually a very crucial part of the script:

```python
#!/usr/bin/env python3
```

This is called the *Shebang*, and it tells the operating system which interpreter to use to execute the code. In our case here, it tells the operating system where to find the right *Python interpreter* that should be used to actually run the code.
    
<p align="center">
  <a href="../../part1#ex5_ret">&#8592; Back to Part 1 - Exercise 5</a>
</p>