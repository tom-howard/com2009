#!/bin/bash

cd ~/catkin_ws/src/
git clone https://github.com/tom-howard/com2009_actions.git
git clone https://github.com/tom-howard/com2009_vision.git
git clone https://github.com/tom-howard/com2009_simulations.git

cd ~/catkin_ws/src/com2009_actions/src/
chmod +x camera_sweep.py

cd ~/catkin_ws/src/com2009_vision/src/
chmod +x image_colours.py colour_search.py
mkdir -p ~/.gazebo/models/
cd ~/catkin_ws/src/com2009_vision/models/
cp -r ./line_following_track ~/.gazebo/models/

cd ~/catkin_ws/ && catkin_make
