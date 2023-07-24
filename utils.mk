# Copyright (C) 2019-2022, Xilinx, Inc.
# Copyright (C) 2022-2023, Advanced Micro Devices, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# vitis makefile-generator v2.0.9
#
#+-------------------------------------------------------------------------------
# The following parameters are assigned with default values. These parameters can
# be overridden through the make command line
#+-------------------------------------------------------------------------------

REPORT := no
PROFILE := no
DEBUG := no


# checking Vivado
ifeq ($(wildcard $(XILINX_VIVADO)/bin/vivado),)
$(error [ERROR]: Cannot locate Vivado installation. Please set XILINX_VIVADO variable. Have you sourced settings?)
endif

# checking Vitis
ifeq ($(wildcard $(XILINX_VITIS)/bin/v++),)
$(error [ERROR]: Cannot locate Vitis installation in $(XILINX_VITIS). Please set XILINX_VITIS variable. Have you sourced settings?)
endif

#'estimate' for estimate report generation
#'system' for system report generation
ifneq ($(REPORT), no)
VPP_LDFLAGS += --report estimate
VPP_LDFLAGS += --report system
endif

#Generates profile summary report
ifeq ($(PROFILE), yes)
VPP_LDFLAGS += --profile_kernel data:all:all:all
endif

#Generates debug summary report
ifeq ($(DEBUG), yes)
VPP_LDFLAGS += --dk protocol:all:all:all
endif

#Check vitis setup
ifndef XILINX_VITIS
  XILINX_VITIS = /opt/Xilinx/Vitis/$(TOOL_VERSION)
  export XILINX_VITIS
endif

# Special processing for tool version/platform type
# VITIS_VER = $(shell v++ --version | grep 'v++' | sed 's/^[[:space:]]*//' | sed -e 's/^[*]* v++ v//g' | cut -d " " -f1) 
VITIS_VER = $(shell v++ --version | sed -n "2 s/^.*v\([0-9\.]*\).*/\1/p")

#check if need sd_card
ifeq ($(HOST_ARCH), aarch32)
SD_CARD_NEEDED := on
endif
ifeq ($(HOST_ARCH), aarch64)
SD_CARD_NEEDED := on
endif

#Set/Check SYSROOT/K_IMAGE/ROOTFS
ifneq ($(HOST_ARCH), x86)
K_IMAGE ?= $(SYSROOT)/../../Image
ROOTFS ?= $(SYSROOT)/../../rootfs.ext4
endif

# checking sysroot
ifneq ($(HOST_ARCH), x86)
ifeq ($(wildcard $(SYSROOT)),)
$(error [ERROR]: SYSROOT variable is not set, please set SYSROOT variable correctly and rerun)
endif
endif

CXX := g++
ifeq ($(HOST_ARCH), x86)
ifeq ($(shell expr $(VITIS_VER) \>= 2022.1), 1)
CXX_VER := 8.3.0
else
CXX_VER := 6.2.0
endif
CXX_V := $(shell echo $(CXX_VER) | awk -F. '{print tolower($$1)}')
ifneq ($(shell expr $(shell echo "__GNUG__" | g++ -E -x c++ - | tail -1) \>= $(CXX_V)), 1)
ifndef XILINX_VIVADO
$(error [ERROR]: g++ version too old. Please use $(CXX_VER) or above)
else
CXX := $(XILINX_VIVADO)/tps/lnx64/gcc-$(CXX_VER)/bin/g++
ifeq ($(LD_LIBRARY_PATH),)
export LD_LIBRARY_PATH := $(XILINX_VIVADO)/tps/lnx64/gcc-$(CXX_VER)/lib64
else
export LD_LIBRARY_PATH := $(XILINX_VIVADO)/tps/lnx64/gcc-$(CXX_VER)/lib64:$(LD_LIBRARY_PATH)
endif
$(warning [WARNING]: g++ version too old. Using g++ provided by the tool: $(CXX))
endif
endif
else 
ifeq ($(HOST_ARCH), aarch64)
CXX := $(XILINX_VITIS)/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-g++
else ifeq ($(HOST_ARCH), aarch32)
CXX := $(XILINX_VITIS)/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/arm-linux-gnueabihf-g++
endif
endif

#Check OS and setting env for xrt c++ api
OSDIST = $(shell lsb_release -i |awk -F: '{print tolower($$2)}' | tr -d ' \t' )
OSREL = $(shell lsb_release -r |awk -F: '{print tolower($$2)}' |tr -d ' \t')

# for centos and redhat
ifneq ($(findstring centos,$(OSDIST)),)
ifeq (7,$(shell echo $(OSREL) | awk -F. '{print tolower($$1)}' ))
ifeq ($(HOST_ARCH), x86)
XRT_CXXFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0
endif
endif
else ifneq ($(findstring redhat,$(OSDIST)),)
ifeq (7,$(shell echo $(OSREL) | awk -F. '{print tolower($$1)}' ))
ifeq ($(HOST_ARCH), x86)
XRT_CXXFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0
endif
endif
endif

#Setting VPP
VPP := v++

#Cheks for aiecompiler
AIECXX := aiecompiler
AIESIMULATOR := aiesimulator
X86SIMULATOR := x86simulator



export PATH := $(XILINX_VITIS)/bin:$(XILINX_XRT)/bin:$(PATH)
ifeq ($(HOST_ARCH), x86)
	ifeq (,$(LD_LIBRARY_PATH))
		LD_LIBRARY_PATH := $(XILINX_XRT)/lib
	else
		LD_LIBRARY_PATH := $(XILINX_XRT)/lib:$(LD_LIBRARY_PATH)
	endif
endif

# Cleaning stuff
RM = rm -f
RMDIR = rm -rf

MV = mv -f
CP = cp -rf
ECHO:= @echo
