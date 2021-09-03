#!/bin/bash

# Define folder paths
start_path=/home/ana/nbvp_ws/src/nbvplanner/startup/kopterworx_one_flying

for max_range in {0..1..9}
  do
    rosbag record -o our-maze /red/octomap_volume /red/comp_times __name:=my_bag &
    ./start.sh 
    rosnode kill /my_bag
    killall gzserver
    done
done
