# Bash script (for WSL2)
# Obtains the IPv4 address of the host Windows machine
# and assigns this to the DISPLAY variable for GUI apps

# Grabs all the IPv4 addresses from the output of ipconfig.exe
# by filtering for lines containing 'IPv4'
# awk then grabs the actual IP address from each line
# and dumps these to a file:
ipconfig.exe | grep 'IPv4' | awk {'print $NF'} > ~/.wsl-ros/ipv4s.txt
# ipconfig.exe is a windows command, so we need to get rid of CRLFs in the output
dos2unix -q ~/.wsl-ros/ipv4s.txt
# read will just parse the first line of this file:
read -r line < ~/.wsl-ros/ipv4s.txt
export DISPLAY=$line:0.0
