#!/bin/bash

rbpth=$rbname'.tar.gz'
rbwpth=$(sed "s/\/mnt\/u/U\:/;s/\//\\\/g" <<< $rbpth)

echo "Restoring your home directory from '"$rbwpth"', please wait..."

rblog=$rbname'-log.txt'

echo "$(date): Restore to $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> $rblog

tar -xjf $rbpth -C /

echo "Restore complete."
