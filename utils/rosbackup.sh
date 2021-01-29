#!/bin/bash
mkdir -p /mnt/u/wsl-ros

touch /mnt/u/wsl-ros/wsl-ros-log.txt
echo "$(date): Backup from $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> /mnt/u/wsl-ros/wsl-ros-log.txt

tar --exclude='/home/student/.local' \
    --exclude='/home/student/.cache' \
    --exclude='/home/student/.config'\
    --exclude='/home/student/.atom'  \
    -cvjf /mnt/u/wsl-ros/ros-backup.tar.gz /home/student
