#!/bin/bash

rbpth=$rbname'.tar.gz'
rbwpth=$(sed "s/\/mnt\/u/U\:/;s/\//\\\/g" <<< $rbpth)

echo "Backing up your WSL-ROS environment to '"$rbwpth"', please wait..."

rbdir=$(sed "s/\/[^/]*$//" <<< $rbname)

mkdir -p $rbdir

rblog=$rbname'-log.txt'

touch $rblog

echo "$(date): Backup from $(hostname) [wsl-ros version: $WSL_ROS_VER]" >> $rblog

tar --exclude='home/student/.local'    \
    --exclude='home/student/.cache'    \
    --exclude='home/student/.config'   \
    --exclude='home/student/pkgs'      \
    --exclude='home/student/.wsl-ros'  \
    --exclude='home/student/.dbus'     \
    --exclude='home/student/.gazebo'   \
    --exclude='home/student/.miro2'    \
    --exclude='home/student/.ros'      \
    --exclude='home/student/.rviz'     \
    --exclude='home/student/.ignition' \
    --exclude='home/student/.sdformat' \
    --exclude='home/student/.vscode-server' \
    --checkpoint=.200 -cjf $rbpth -C / home/student/

echo "."
echo "Backup complete."
