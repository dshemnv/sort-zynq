#!/bin/bash

echo "Deploying to target"

ssh root@${TARGET_IP} "sh -c '/usr/bin/killall -q gdbserver; rm -rf /mnt/sd-mmcblk0p1/kalman; exit 0'"

echo "Transfer binary to target"

scp ${PROGRAM_PATH} root@${TARGET_IP}:/mnt/sd-mmcblk0p1

echo "Starting GDB Server"

ssh -t root@${TARGET_IP} "sh -c 'cd /mnt/sd-mmcblk0p1; gdbserver :${DEBUG_PORT} ./kalman'"
