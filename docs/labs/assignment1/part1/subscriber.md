---  
title: "A Simple Subscriber Node"  
---

## The Code

Copy **all** the code below into your `subscriber.py` file and (again) *make sure you read the annotations to understand how it all works!*

```python title="subscriber.py"
--8<-- "code_templates/subscriber.py"
```

1. As with our publisher node, we need to import the `rclpy` client library and the `String` message type from the `example_interfaces.msg` library in order to write a Python ROS Node and use the relevant ROS messages:

2. This time, we create a Python Class called `SimpleSubscriber()` instead, but which still inherits the `Node` class from `rclpy` as we did with the Publisher before.

3. Once again, using the `#!python super()` method we call the `#!python __init__()` method from the parent Node class that our `SimpleSubscriber` class is derived from, and provide a name to use to register in on the network.

4. We're now using the `#!python create_subscription()` method here, which will allow this node to *subscribe* to messages on a ROS Topic. When calling this we provide 4 key bits of information:

    1. `msg_type`: The **type** of message that the topic uses (which we could obtain by running the `ros2 topic info` command).
        
        We know (having just created the publisher), that our topic uses `String` messages (from `example_interfaces`).
    
    1. `topic`: The **name of the topic** that we want to listen (or subscribe) to.
        
        !!! warning "Fill in the Blank!"
            Replace the `{BLANK}` in the code above with the name of the topic that our [`publisher.py` node](./publisher.md) was set up to publish to!
    
    1. `callback`: When building a subscriber, we need a *callback function*, which is a function that will execute every time a new message is received from the topic.

        At this stage, we define what this callback function is called (`self.msg_callback`), and we'll actually define the function itself further down within the Class.
    
    1. `qos_profile`: As before, a **queue size** to limit the amount of messages that are *queued* in a buffer. 

5. Print a Log message to the terminal to indicate that the initialisation process has taken place.

6. Here we're defining what will happen each time our subscriber receives a new message. This callback function must have only one argument (other than `self`), which will contain the message data that has been received:

    We're also using [a Python Type Annotation](https://docs.python.org/3/library/typing.html) here too, which informs the interpreter that the `topic_message` that is received by the `msg_callback` function will be of the `String` data type.
    
    (All this really does is allow autocomplete functionality to work within our text editor, so that whenever we want to pull an attribute from the `toic_message` object it will tell us what attributes actually exist within the object.)

7. In this simple example, all we're going to do on receipt of a message is to print a couple of log messages to the terminal, to include: 

    1. The name of this node (using the `self.get_name()` method)

    1. The message that has been received (`topic_mesage.data`)

8. The rest of this is exactly the same as before with our publisher.

## Don't Forget the Shebang! {#dfts}

Remember: **don't forget the shebang**, it's very important!

```python
#!/usr/bin/env python3
```

<p align="center">
  <a href="../../part1#sub_ret">&#8592; Back to Part 1</a>
</p>