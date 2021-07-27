#!/bin/bash

## declare an array variable
declare -a arr=("roslaunch xplane_ros default.launch" "roslaunch rosplane_sim xplane_fw.launch")

for cmd in "${arr[@]}"; do {
  echo "Process \"$cmd\" started";
  $cmd & pid=$!
  PID_LIST+=" $pid";
  sleep 1
} done

trap "kill $PID_LIST" SIGINT

echo "Parallel processes have started";

wait $PID_LIST

echo
echo "All processes have completed";