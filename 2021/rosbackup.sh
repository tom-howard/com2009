#!/bin/bash

echo "Backing up your WSL-ROS environment to 'U:\wsl-ros\ros-backup.tar.gz', please wait..."

mkdir -p /mnt/u/wsl-ros

touch /mnt/u/wsl-ros/wsl-ros-log.txt

echo "$(date): Backup from $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> /mnt/u/wsl-ros/wsl-ros-log.txt

tar --exclude='home/student/.local'    \
    --exclude='home/student/.cache'    \
    --exclude='home/student/.config'   \
    --exclude='home/student/pkgs'      \
    --exclude='home/student/.wsl-ros'  \
    -cjf /mnt/u/wsl-ros/ros-backup.tar.gz -C / home/student/
    
echo "Backup complete."
