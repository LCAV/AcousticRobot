#!/usr/bin/expect -f
# connect to robot
spawn ssh pi@172.16.156.139
expect "password:"
sleep 1
send "raspberry" 
send "sudo reboot"



