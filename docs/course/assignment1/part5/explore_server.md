---
title: The Part 5 Explore Server Template
---

# The Part 5 Explore Server Template

```py title="explore_server.py"
--8<-- "code_templates/explore_server.py"
```

1. Some variables to store data from the `/odom` (`posx` and `posy`) and `/scan` (`lidar_reading`) subscribers, and share this data across the class.
2. A shutdown flag (you've seen this before)
3. Some flags to determine when we've received data from the `/odom` and `/scan` subscribers, to ensure that our action server's main execution callback doesn't begin until we have some valid data to work with (see usage below).
4. Here we're creating an object that we can use in our action server's main execution callback to control the rate of execution inside a `#!py while` loop (see usage below).
5. Refer back to the [Part 2 Odometry Subscriber](../part2/odom_subscriber.md#modifying-the-message-callback) for help with this (if you need it).
6. Refer back to the [Part 3 Lidar Subscriber](../part3/lidar_subscriber.md) for help with this (if you need it).
7. Calling this here will block any further code execution until enough time has elapsed. 
    
    This time is dictated by the `frequency` parameter that we defined when we set this up earlier:

    ```py
    self.loop_rate = self.create_rate(
        frequency=5, 
        clock=self.get_clock()
    )
    ```
    

<p align="center">
  <a href="../../part5#explore_srv_ret">&#8592; Back to Part 5</a>
</p>