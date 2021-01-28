#!/bin/bash

echo "$(date): Restore from $(hostname)" >> /mnt/u/wsl-ros/wsl-ros-log.txt

tar -xvjf /mnt/u/wsl-ros/ros-backup.tar.gz -C /
