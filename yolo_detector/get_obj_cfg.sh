#!/bin/bash

# TODO Should also fetch cfgs
# TODO Why two copies
wget https://www.dropbox.com/s/3mcjldc7ddykfza/open_ptrack_object.names?dl=1 -O data/open_ptrack_object.names
wget https://www.dropbox.com/s/3mcjldc7ddykfza/open_ptrack_object.data?dl=1 -O darknet_opt/cfg/open_ptrack_object.data
wget https://www.dropbox.com/s/3mcjldc7ddykfza/yolo-opt-object.weights?dl=1 -O darknet_opt/yolo-opt-object.weights
  
