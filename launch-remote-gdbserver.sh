#!/bin/bash

echo "Deploying to target"

ssh root@${TARGET_IP} "sh -c '/usr/bin/killall -q gdbserver; rm -rf /home/root/kalman; exit 0'"

echo "Transfer binary to target"

scp ${PROGRAM_PATH} root@${TARGET_IP}:/home/root

echo "Starting GDB Server"

ssh -t root@${TARGET_IP} "sh -c 'cd /home/root; export DISPLAY=:0; gdbserver :${DEBUG_PORT} ./kalman MOT15/test yolov3_adas_pruned_0_9'"
