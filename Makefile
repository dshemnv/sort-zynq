# defaults, if not set
TARGET ?= sw_emu
HOST_ARCH ?= x86

APP_NAME := kalman
BUILD_DIR := build

SRC_DIR := src
KERNEL_DIR := hls
VITIS_VISION_LIB_DIR := extern/vitis_lib/vision

# sets various useful variables
include ./utils.mk

IDIRS := src 
ifeq ($(HOST_ARCH), aarch64)
IDIRS += $(SYSROOT)/usr/include/xrt $(XILINX_HLS)/include $(VITIS_VISION_LIB_DIR)/ext/xcl2 $(VITIS_VISION_LIB_DIR)/L1/include $(SYSROOT)/usr/include/opencv4 
endif

# generate include flags
IFLAGS := $(addprefix -I, $(IDIRS))

# generate srcs 
HOST_SRCS = $(wildcard $(SRC_DIR)/*.cpp)

ifeq ($(HOST_ARCH), aarch32)
HOST_SRCS += $(wildcard $(KERNEL_DIR)/*.cpp)
HOST_SRCS += $(VITIS_VISION_LIB_DIR)/ext/xcl2/xcl2.cpp
endif

OBJS := $(subst $(SRC_DIR),$(BUILD_DIR),$(HOST_SRCS:.cpp=.o))
DEP_FILES := $(OBJS:%.o=%.d)

# common compile flags
CXXFLAGS += $(IFLAGS) -std=c++14 -O3 -Wall -MMD -MP -Wno-unknown-pragmas -Wno-unused-label -fmessage-length=0 
LDFLAGS += -pthread -Wl,--as-needed
VPP_FLAGS += -t $(TARGET) --platform $(XPLATFORM) --save-temps
VPP_LDFLAGS += --optimize 2 -R 2

# setup host dependant cxx flags
ifeq ($(HOST_ARCH), x86)
CXXFLAGS += $(shell pkg-config --cflags opencv4)
LDFLAGS += $(shell pkg-config --libs opencv4)
else ifeq ($(HOST_ARCH), aarch64)
CXXFLAGS += -DKALMAN_ACCEL --sysroot=$(SYSROOT)
LDFLAGS += -L$(SYSROOT)/usr/lib -L$(SYSROOT)/lib -libxilinxopencl -lxrt_coreutil
LDFLAGS += -lopencv_videoio -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_flann -lopencv_video -lopencv_calib3d -lopencv_highgui
endif

# Host app building
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/$(APP_NAME): $(OBJS)
	mkdir -p $(BUILD_DIR)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@


.PHONY: app
app: $(BUILD_DIR)/$(APP_NAME)

.PHONY: clean
clean:
	@rm $(BUILD_DIR)/*

-include $(wildcard $(DEP_FILES))
