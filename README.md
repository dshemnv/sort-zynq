# SORT-ZYNQ

SORT-ZYNQ is a real-time multiple object tracking solution for AMD Zynq UltraScale+ devices. It allows to perform object detection on the PL (FPGA) and the tracking on the PS (ARM CPU). The Auction algorithm is used as a LSAP solver. It was succesfully tested on a KV260 board.

## Requirements

* Petalinux v2021.2 with Vitis AI Libraries
* A Zynq UltraScale+ device
  
## Instructions

Use `make` to build the projet.

```bash
HOST_ARCH=aarch64 make yolo-app
```