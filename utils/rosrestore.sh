#!/bin/bash

echo "$(date): Restore from $(hostname)" >> ~/wsl-ros-log.txt

tar -xvjf /mnt/u/wsl-ros/ros-backup.tar.gz -C /

cp ~/wsl-ros-log.txt /mnt/u/wsl-ros/wsl-ros-log.txt
