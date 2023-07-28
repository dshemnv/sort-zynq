ifeq ($(filter clean clean-pkg clean-app, $(MAKECMDGOALS)),)
ifeq ($(filter aarch64 x86, $(HOST_ARCH)),)
$(error [ERROR]: HOST_ARCH variable is not correctly set. Set it to x86 or aarch64)
endif
endif

# defaults, if not set
TARGET ?= sw_emu
PLATFORM ?= /opt/xilinx/xilinx_zcu102_base_202120_1/xilinx_zcu102_base_202120_1.xpfm
PLATFORM_NAME = $(strip $(patsubst %.xpfm, % , $(shell basename $(PLATFORM))))

# ifeq ($(TARGET), sw_emu)
# $(info Target is software emulation, setting HOST_ARCH to x86)
# HOST_ARCH := x86
# endif

APP_NAME := kalman
BUILD_DIR := build/$(HOST_ARCH)/$(TARGET)
$(info $(BUILD_DIR))

SRC_DIR := src
KERNEL_DIR := hls
VITIS_VISION_LIB_DIR := extern/vitis_lib/vision

# sets various useful variables, if using v++
ifneq ($(filter accel package app debug-all debug-app, $(MAKECMDGOALS)),)
include utils.mk
endif

# ------------------------- Setting correct compiler ------------------------- #

ifeq ($(HOST_ARCH), aarch64)
CXX := $(XILINX_VITIS)/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-g++
else ifeq ($(HOST_ARCH), x86)
CXX := g++
endif

# ------------------------------- Temp folders ------------------------------- #

TEMP_DIR := tmp/$(TARGET)_$(PLATFORM_NAME)
TEMP_REPORT_DIR := $(TEMP_DIR)/report
TEMP_LOG_DIR := $(TEMP_DIR)/logs

# ----------------------------- Host app section ----------------------------- #

IDIRS := src 
XF_VIDEO_IDIR := $(VITIS_VISION_LIB_DIR)/L1/include/video
ifeq ($(HOST_ARCH), aarch64)
IDIRS += $(SYSROOT)/usr/include/xrt $(XILINX_HLS)/include $(VITIS_VISION_LIB_DIR)/ext/xcl2 $(VITIS_VISION_LIB_DIR)/L1/include $(SYSROOT)/usr/include/opencv4 $(XF_VIDEO_IDIR) $(KERNEL_DIR)
endif

# generate include flags
IFLAGS := $(addprefix -I, $(IDIRS))

# generate srcs 
HOST_SRCS = $(wildcard $(SRC_DIR)/*.cpp)

ifeq ($(HOST_ARCH), aarch64)
# HOST_SRCS += $(wildcard $(KERNEL_DIR)/*.cpp)
HOST_SRCS += $(VITIS_VISION_LIB_DIR)/ext/xcl2/xcl2.cpp
endif

HOST_SRCS_FILES_ONLY := $(notdir $(HOST_SRCS))
OBJS := $(addprefix $(BUILD_DIR)/, $(HOST_SRCS_FILES_ONLY:.cpp=.o))
DEP_FILES := $(OBJS:%.o=%.d)

# common compile flags
CXXFLAGS += $(IFLAGS) -std=c++14 -O3 -Wall -MMD -MP -Wno-unknown-pragmas -Wno-unused-label -fmessage-length=0 
LDFLAGS += -pthread -Wl,--as-needed
VPP_FLAGS += -t $(TARGET) --platform $(PLATFORM) --save-temps
VPP_LDFLAGS += --optimize 2 -R 2

# setup host dependant cxx flags
ifeq ($(HOST_ARCH), x86)
CXXFLAGS += $(shell pkg-config --cflags opencv4)
LDFLAGS += $(shell pkg-config --libs opencv4)
else ifeq ($(HOST_ARCH), aarch64)
CXXFLAGS += -I$(SYSROOT)/usr/include/opencv4 -I$(SYSROOT)/usr/include -I$(SYSROOT)/usr/include/xrt
CXXFLAGS += -DKALMAN_ACCEL --sysroot=$(SYSROOT) 
LDFLAGS += -L$(SYSROOT)/usr/lib -L$(SYSROOT)/lib -lxrt_coreutil -lxilinxopencl --sysroot=$(SYSROOT)
LDFLAGS += -lopencv_videoio -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_flann -lopencv_video -lopencv_calib3d -lopencv_highgui
LIBRARY_PATH := $(OPENCV_LIB):$(LD_LIBRARY_PATH):$(XILINX_XRT)/lib
endif

# Host app building
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/xcl2.o: $(VITIS_VISION_LIB_DIR)/ext/xcl2/xcl2.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/$(APP_NAME): $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

# ---------------------------- HLS Kernel section ---------------------------- #

VPP_FLAGS += -I$(VITIS_VISION_LIB_DIR)/L1/include -I$(KERNEL_DIR)
VPP_FLAGS += --hls.clock 300000000:kalmanfilter_accel
VPP_LDFLAGS += --clock.defaultFreqHz 300000000

XCLBIN_FILES += $(BUILD_DIR)/xclbins/krnl_kalmanfilter.xclbin
XCLBIN_FILES_PKG += $(BUILD_DIR)/xclbins/krnl_kalmanfilter_pkg.xclbin

$(TEMP_DIR)/kalmanfilter_accel.xo: $(KERNEL_DIR)/kalman_hls_accel.cpp
	@echo "Compiling Kalman Filter HLS Kernel"
	@mkdir -p $(TEMP_DIR)
	@mkdir -p $(TEMP_REPORT_DIR)
	@mkdir -p $(TEMP_LOG_DIR)
	$(VPP) -c $(VPP_FLAGS) -k kalmanfilter_accel -I'$(<D)' --temp_dir $(TEMP_DIR) --report_dir $(TEMP_REPORT_DIR) --log_dir $(TEMP_LOG_DIR) -o $@ $^

XCLBIN_OBJS += $(TEMP_DIR)/kalmanfilter_accel.xo
XCLBIN_DEPS += $(XCLBIN_OBJS)
SD_FILES += $(BUILD_DIR)/$(APP_NAME)
SD_FILES += xrt.ini

SD_FILES_PREFIXED = $(addprefix --package.sd_file ,$(SD_FILES))

$(XCLBIN_FILES): $(XCLBIN_DEPS)
	@mkdir -p $(BUILD_DIR)
	$(VPP) -l $(VPP_FLAGS) --temp_dir $(TEMP_DIR) --report_dir $(TEMP_REPORT_DIR)/krnl_kalmanfilter $(VPP_LDFLAGS) -o $@ $^

# ----------------------------- Emulation config ----------------------------- #
EMCONFIG := $(BUILD_DIR)/emconfig.json

$(EMCONFIG):
	emconfig --platform $(PLATFORM) --od $(BUILD_DIR)

# ---------------------------- SD Image generation --------------------------- #
$(BUILD_DIR)/sd_card/sd_card.img: $(XCLBIN_FILES) $(BUILD_DIR)/$(APP_NAME)
	$(VPP) -t $(TARGET) --platform $(PLATFORM) -o $(XCLBIN_FILES_PKG) -p $(XCLBIN_FILES) --package.out_dir $(@D) --package.rootfs $(ROOTFS) --package.kernel_image $(K_IMAGE) $(SD_FILES_PREFIXED)
	@echo "*** SD Card Image successfully generated ***"

# ------------------------------- Named targets ------------------------------ #
.PHONY: app
app: $(BUILD_DIR)/$(APP_NAME)

.PHONY: accel
accel: $(XCLBIN_FILES)

.PHONY: debug-accel
debug-accel: VPP_FLAGS += -g
debug-accel: VPP_LDFLAGS += --advanced.param compiler.fsanitize=address,memory --debug.chipscope kalmanfilter_accel_1
debug-accel: accel 

.PHONY: debug-app
debug-app: CXXFLAGS += -g3 -O0
debug-app: app 

.PHONY: debug-all
debug-all: debug-accel debug-app

.PHONY: all
all: accel app
	@echo "Pakaging outputs"

.PHONY: package
package: accel app $(BUILD_DIR)/sd_card/sd_card.img

.PHOMY: clean-app
clean-app:
	@rm -f $(BUILD_DIR)/*.d
	@rm -f $(BUILD_DIR)/*.o
	@rm -f $(BUILD_DIR)/$(APP_NAME)

.PHONY: clean-pkg
clean-pkg:
	@rm -rf $(BUILD_DIR)/sd_card
	@rm -rf $(TEMP_DIR)

.PHONY: clean
clean:
	@rm -rf $(BUILD_DIR)/*
	@rm -rf $(TEMP_DIR)
	@rm -rf .Xil

-include $(wildcard $(DEP_FILES))