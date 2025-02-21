---  
title: Launching ROS (and Pairing the Laptop)
---  

The first step is to launch ROS on the Waffle.

!!! info "Important"
    This ensures that all the core ROS functionality is executed on the robot, without this the robot won't be able to do *anything*!

## Step 1: Identify your Waffle

Robots are named as follows:

    dia-waffleX

... where `X` is the *'Robot Number'* (a number between 1 and 50). Make sure you know which robot you are working with by checking the label printed on top of it!

## Step 2: Pairing your Waffle to a Laptop

[As discussed earlier](./intro.md#laptops), you'll be provided with one of our Robotics Laptops to work with in the lab, and the robot needs to be paired with this in order for the two to work together.  

1. Open up a terminal instance on the laptop, either by using the ++ctrl+alt+t++ keyboard shortcut, or by clicking the Terminal App icon in the favourites bar on the left-hand side of the desktop:
    
    <figure markdown>
      ![](../images/laptops/terminal_icon.svg){width=60px}
    </figure>

1. We'll use our purpose-built `waffle` CLI to handle the pairing process. Run this in the terminal by entering the following command to *pair* the laptop and robot:

    ***
    ```bash
    waffle X pair
    ```
    Replacing `X` with the number of the robot that you are working with.
    
    ***

1. You may see a message like this early on in the pairing process:

    <figure markdown>
      ![](../images/laptops/ssh_auth.svg){width=600px}
    </figure>

    If so, just type `yes` and then hit ++enter++ to confirm that you want to continue.

1. Enter the password for the robot when requested (we'll tell you what this is in the lab!)

    !!! note
        You won't see anything change on the screen when you are entering the password. This is normal, just keep typing!!
    
1. The pairing process will take a minute, but once it's finished you should see a message saying `pairing complete`, displayed in blue in the terminal. 

1. Then, in the same terminal, enter the following command: <a name="tmux"></a>

    ***
    ```bash
    waffle X term
    ```
    (again, replacing `X` with the number of *your* robot).
    
    ***

    A green banner should appear across the bottom of the terminal window:
    
    <figure markdown>
      ![](../images/laptops/tmux.svg){width=500px}
    </figure>

    This is a terminal instance running **on the robot**, and any commands that you enter here will be executed *on the robot* (not the laptop!)

## Step 3: Launching ROS

Launch ROS on the robot by entering the following command:

```bash
tb3_bringup
```

If all is well then the robot will play a nice *"do-re-me"* sound and a message like this should appear (amongst all the other text):

``` { .txt .no-copy }
[tb3_status.py-#] ######################################
[tb3_status.py-#] ### dia-waffleX is up and running! ###
[tb3_status.py-#] ######################################
```

You shouldn't need to interact with this terminal instance any more now, but the screen will provide you with some regular real-time info related to the status of the robot. As such, keep this terminal open in the background and check on the `Battery` indicator every now and then:

``` { .txt .no-copy } 
Battery: 12.40V [100%]
```

!!! info "Low Battery :material-battery-low:"

    **The robot's battery won't last a full 2-hour lab session!!**

    When the capacity indicator reaches around 15% then it will start to beep, and when it reaches ~10% it will stop working all together.  Let a member of the teaching team know when the battery is running low and we'll replace it for you. (It's easier to do this when it reaches 15%, rather than waiting until it runs below 10%!)


## Step 4: Robot-Laptop 'Bridging'

The Waffle and laptop both communicate over the University network via [a Zenoh Bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds){target="_blank"}. The bridge should already be running on the robot after having run the `tb3_bringup` command in [Step 3 above](#step-3-launching-ros). 

The next **crucial** step is to establish a connection to this bridge from the laptop, so that all ROS nodes, topics etc. can flow between the two devices as necessary. 

!!! warning "This is Essential!"
    You **always** need to have the bridge running on the laptop in order to be able to communicate with your robot!

Open up **a new terminal instance** on the laptop (either by using the ++ctrl+alt+t++ keyboard shortcut, or by clicking the Terminal App icon) and enter the following command:

```bash
waffle X bridge
```
You should now have two terminals active: 

1. The *robot* terminal where you ran `tb3_bringup` to launch ROS in [Step 3](#step-3-launching-ros)[^term_recover]
1. The *laptop* terminal where you just ran the `bridge` command

[^term_recover]: If you happen to have closed down the *robot* terminal, you can return to it by entering `waffle X term` from a new terminal instance on the laptop.

Leave both of these terminals alone, but **keep them running in the background at all times** while working with your robot.

## Shutting Down (at the end of a Lab Session)

When you've finished working with a robot it's really important to **shut it down properly** before turning off the power switch. Please refer to the [safe shutdown procedures](./shutdown.md) for more info.