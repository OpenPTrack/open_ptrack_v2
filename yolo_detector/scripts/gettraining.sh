
#!/bin/bash

read -p "Training set name (e.g., iSTEP-CH_180219A): "  TRAINING_SET
echo "Using name $TRAINING_SET"
echo "Remember to supply URLs that provide the actual data,"
echo " not a login/info page, i.e., use Dropbox URLs ending in dl=1"

echo " "
read -p "URL for $TRAINING_SET.data: " data_URL
read -p "URL for $TRAINING_SET.names: " names_URL
read -p "URL for $TRAINING_SET.weights: " weights_URL
read -p "URL for $TRAINING_SET-test.cfg: " cfg_URL

echo " "
echo "DOWNLOADING..."

echo "-------------------------"
wget -O ~/workspace/ros/src/open_ptrack/yolo_detector/darknet_opt/cfg/$TRAINING_SET.data $data_URL

echo "-------------------------"
wget -O ~/workspace/ros/src/open_ptrack/yolo_detector/data/$TRAINING_SET.names $names_URL
echo " "

echo "-------------------------"

wget -O ~/workspace/ros/src/open_ptrack/yolo_detector/darknet_opt/$TRAINING_SET.weights $weights_URL
echo " "

echo "-------------------------"

wget -O ~/workspace/ros/src/open_ptrack/yolo_detector/darknet_opt/cfg/$TRAINING_SET-test.cfg $cfg_URL

echo "-------------------------"

echo " "
echo "Files downloaded:"
ls -l ~/workspace/ros/src/open_ptrack/yolo_detector/darknet_opt/cfg/$TRAINING_SET.data
ls -l ~/workspace/ros/src/open_ptrack/yolo_detector/data/$TRAINING_SET.names
ls -l ~/workspace/ros/src/open_ptrack/yolo_detector/darknet_opt/$TRAINING_SET.weights
ls -l ~/workspace/ros/src/open_ptrack/yolo_detector/darknet_opt/cfg/$TRAINING_SET-test.cfg 


echo "-------------------------"
echo "If all files have downloaded correctly, "
echo "now execute the following command on each" 
echo "node to set the training set any time you need it:"
echo " "
echo "export OPT_OBJECT_TRAINING=\"$TRAINING_SET\""
echo " "
