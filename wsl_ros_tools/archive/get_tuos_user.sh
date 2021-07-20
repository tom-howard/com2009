#!/bin/bash

# this doesn't currently work! (something to do with a trailing LF in the ~/tuos_user file maybe?)

cd /mnt/c/

cmd.exe /c "echo %USERNAME%" > ~/tuos_user
dos2unix -q ~/tuos_user
export TUOS_USER=$(cat ~/tuos_user)
