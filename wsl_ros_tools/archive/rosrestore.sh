#!/bin/bash

echo "$(date): Restore to $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> /mnt/u/wsl-ros/wsl-ros-log.txt

tar -xvjf /mnt/u/wsl-ros/ros-backup.tar.gz -C /
