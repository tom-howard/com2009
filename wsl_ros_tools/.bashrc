# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
#HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
#shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
#HISTSIZE=1000
#HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

### WSL-ROS Mods:
export WSL_ROS_VER=$(cat ~/.wsl-ros/wsl_ros_ver)

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@WSL-ROS($WSL_ROS_VER)\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@WSL-ROS($WSL_ROS_VER):\w\$ '
fi
###
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

################### Custom ###################

# WSL-ROS Configs:
## GUI:
source ~/.wsl-ros/set_display.sh
export LIBGL_ALWAYS_INDIRECT=

## Other:
source ~/.wsl-ros/get_tuos_user.sh
# display a rosrestore prompt to the user
# if this is the first launch of WSL-ROS:
if [ ! -f ~/.wsl-ros/no_welcome ]; then
  touch ~/.wsl-ros/no_welcome
  wsl_ros first-launch
fi

# ROS Settings:
## General:
export GAZEBO_IP=127.0.0.1
export EDITOR='nano -w' # Used in rosed

## robot_switch routine is used to switch between the two robot profiles, MiRo and Turtlebot3
# '~/.current_robot' text file is checked for one of the following strings: 'miro', 'turtlebot3'
# This is saved to the $CURRENT_ROBOT environment variable 
# If the file is not present or the string is not valid, the file is reset to Turtlebot3
###############################################################################
# Check '~/.current_robot' contents for 'MiRo' or 'Turtlebot3':
touch ~/.current_robot
if grep -qi "miro" ~/.current_robot; then
  export CURRENT_ROBOT=miro
elif grep -qi "turtlebot" ~/.current_robot; then
  export CURRENT_ROBOT=turtlebot3	
else
  # In all other cases set robot mode to 'Turtlebot3':
  rm -f ~/.current_robot
  echo "turtlebot" > ~/.current_robot
  export CURRENT_ROBOT=turtlebot3
fi

###############################
if [ "$CURRENT_ROBOT" == "miro" ]; then
  # Source MiRo robot profile from .bashrc_miro:
  . ~/.wsl-ros/bashrc_miro
elif [ "$CURRENT_ROBOT" == "turtlebot3" ]; then
  # Source Turtlebot3 robot profile from .bashrc_turtlebot3:
  . ~/.wsl-ros/bashrc_turtlebot3
fi

# Remove duplicates from the terminal command history
HISTSIZE=100000
HISTFILESIZE=200000
shopt -s histappend
export HISTCONTROL=ignoreboth:erasedups
export PROMPT_COMMAND="history -n; history -w; history -c; history -r"
tac "$HISTFILE" | awk '!x[$0]++' > /tmp/tmpfile  && tac /tmp/tmpfile > "$HISTFILE"
rm /tmp/tmpfile
