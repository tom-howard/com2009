#!/bin/bash

cd /mnt/c/ && cmd.exe /c "echo %USERNAME%" > ~/.wsl-ros/tuos_user
dos2unix -q ~/.wsl-ros/tuos_user

read -r first_line < ~/.wsl-ros/tuos_user
export TUOS_USER=$first_line
