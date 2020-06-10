#!/bin/sh
#comment 
#All required things for ardupilot

echo "	welcome "

xterm -title "detection0" -hold -e "cd; cd catkin_ws; cd src; cd football;cd src; python3 detection0.py" &
xterm -title "detection1" -hold -e "cd; cd catkin_ws; cd src; cd football;cd src; python3 detection1.py" &
xterm -title "player0" -hold -e "cd; cd catkin_ws; cd src; cd football;cd src; python3 player0.py" &
xterm -title "player1" -hold -e "cd; cd catkin_ws; cd src; cd football;cd src; python3 player1.py" &