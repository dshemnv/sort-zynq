#!/bin/bash

[ ! -f sortzynq ] && echo "SORT-ZYNQ binary is not here. Change directory." && exit 1

if [ ! -d "MOT15" ]; then
    echo "Downloading MOT15"
    curl -OL https://motchallenge.net/data/MOT15.zip
    echo "Extracting MOT15"
    unzip MOT15.zip
    echo "Extraction complete"
fi
echo $'Available sequences:\n'
find MOT15/train MOT15/test -maxdepth 1 -mindepth 1 -type d
echo
echo $'Available models:\n'
echo $'yolov3_adas_pruned_0_9\nyolov3_voc\nyolov4_leaky_spp_m\nyolov4_leaky_spp_m_pruned_0_36'
echo
echo $'Usage:\n\t./sortzynq <path/to/seqence> <yolomodel>'