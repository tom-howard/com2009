#!/bin/bash
mkdir -p /mnt/u/wsl-ros

touch /mnt/u/wsl-ros/wsl-ros-log.txt

echo "$(date): Backup from $(hostname)" >> /mnt/u/wsl-ros/wsl-ros-log.txt

tar --exclude='/home/student/.local' \
    --exclude='/home/student/.cache' \
    --exclude='/home/student/.config'\
    -cvjf /mnt/u/wsl-ros/ros-backup.tar.gz /home/student
