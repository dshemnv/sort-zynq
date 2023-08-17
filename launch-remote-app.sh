#!/bin/bash

BIN="$1"

echo "Transfer binary to target"
scp ${PROGRAM_PATH} root@${TARGET_IP}:/mnt/sd-mmcblk0p1

echo "Launching target"
ssh -t root@${TARGET_IP} "sh -c './${BIN} /home/root/MOT15/test/Venice-1'"