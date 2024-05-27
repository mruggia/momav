#!/bin/sh

echo ''
echo ''
echo '##########################################################'
echo '### BUILDING GROUND PC ###################################'
echo '##########################################################'
echo ''
rsync -rvh --delete --checksum momav/ momav@ground:~/catkin_ws/src/momav/
ssh -t momav@ground 'cd ~/catkin_ws && catkin build momav --no-deps'

echo ''
echo ''
echo '##########################################################'
echo '### BUILDING MOMAV #######################################'
echo '##########################################################'
echo ''
rsync -rvh --delete --checksum -e 'ssh -A -J momav@ground' momav/ momav@momav:~/catkin_ws/src/momav/
ssh -t momav@momav -J momav@ground 'cd ~/catkin_ws && catkin build momav --no-deps'