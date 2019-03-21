#/bin/bash

byobu new-session -d -s openptrack
byobu send-keys "roscore"
byobu send-keys Enter
sleep 3
byobu split-window -v
byobu send-keys "roslaunch detection detection_node_kinect01.launch enable_pose:=true enable_object:=true enable_people_tracking:=true"
byobu send-keys Enter
byobu split-window -h
byobu send-keys "roslaunch rosbridge_server rosbridge_websocket.launch" 
byobu send-key Enter
byobu select-pane -U
byobu split -h
byobu send-keys "roslaunch tracking tracking_node.launch enable_pose:=true enable_object:=true enable_people_tracking:=true"
byobu send-keys Enter
byobu attach-session -t openptrack
