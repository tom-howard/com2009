#!/bin/bash

echo "Restoring your home directory from 'U:\wsl-ros\ros-backup.tar.gz', please wait..."

echo "$(date): Restore to $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> /mnt/u/wsl-ros/wsl-ros-log.txt

tar -xjf /mnt/u/wsl-ros/ros-backup.tar.gz -C /

echo "Restore complete."
