# Bash script (for WSL)
# Obtains the IPv4 address of the host Windows machine
# and assigns this to the DISPLAY variable for GUI apps

ipconfig.exe | grep -A 2 'shef.ac.uk' | grep 'IPv4' | awk {'print $NF'} > ~/ipv4.txt
dos2unix -q ~/ipv4.txt
export DISPLAY=$(cat ipv4.txt):0.0
