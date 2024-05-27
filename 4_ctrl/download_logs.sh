#!/bin/sh

scp -oProxyJump=momav@ground momav@momav:~/bags/* temp_logs
scp -oProxyJump=momav@ground momav@momav:~/catkin_ws/src/momav/launch/core.launch temp_logs