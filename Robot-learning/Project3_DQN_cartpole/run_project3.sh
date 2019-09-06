#!/bin/bash

echo "Checking your solution ..."
echo "[NOTE: We do not recommend you using this tool for debugging purposes. It might be easier to run things manually for debugging. You should change the sleep values in this script for your own testing]"

sleep 3

roslaunch robot_sim project3.launch &
sleep 1
rosrun robot_sim learn_dqn.py &

#Change this value if you need more time for training
sleep 120

rosrun robot_sim cartpole_gui.py &

sleep 1

xterm -hold -e "rosrun robot_sim executive.py" 

sleep 1

killall -9 rosmaster
killall python



