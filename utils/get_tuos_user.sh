#!/bin/bash

cd /mnt/c/

cmd.exe /c "echo %USERNAME%" > ~/tuos_user
dos2unix -q ~/tuos_user
export TUOS_USER=$(cat ~/tuos_user)
