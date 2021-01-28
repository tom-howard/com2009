#!/bin/bash

tar -xvjf /mnt/u/wsl-ros/ros-backup.tar.gz -C /

echo "$(date): Restore from $(hostname)" >> ~/wsl-ros-log.txt

cp ~/wsl-ros-log.txt /mnt/u/wsl-ros/wsl-ros-log.txt
