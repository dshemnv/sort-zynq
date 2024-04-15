# SORT-ZYNQ

SORT-ZYNQ is a real-time multiple object tracking solution for AMD Zynq UltraScale+ devices. It allows to perform object detection on the PL (FPGA) and the tracking on the PS (ARM CPU). The Auction algorithm is used as a LSAP solver. It was succesfully tested on a KV260 board.

## Requirements

* Petalinux v2021.2 with Vitis AI Libraries (might work on other versions).
* A Zynq UltraScale+ device.
  
## Instructions

Clone the project
```bash
git clone --recurse-submodules https://github.com/dshemnv/sort-zynq.git
```

Create a folder `sysroot` and extract the petalinux SDK to it.

Use `make` to build the projet.

```bash
HOST_ARCH=aarch65 make yolo-app
```
The binary is then available under the `build` directory. Copy the binary to the Zynq device and run it there.

```bash
./sortzynq <path-to-image-files|camera-device> <yolo-model-file>
```

