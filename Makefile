ifeq ($(filter clean clean-pkg clean-app, $(MAKECMDGOALS)),)
ifeq ($(filter aarch64 x86, $(HOST_ARCH)),)
$(error [ERROR]: HOST_ARCH variable is not correctly set. Set it to x86 or aarch64)
endif
endif

# --------------------------- Defaults, if not set --------------------------- #
TARGET ?= sw_emu
PLATFORM ?= /opt/xilinx/xilinx_zcu102_base_202120_1/xilinx_zcu102_base_202120_1.xpfm
PLATFORM_NAME = $(strip $(patsubst %.xpfm, % , $(shell basename $(PLATFORM))))
HLS ?= off

# ifeq ($(TARGET), sw_emu)
# $(info Target is software emulation, setting HOST_ARCH to x86)
# HOST_ARCH := x86
# endif

APP_NAME := kalman
BUILD_DIR := build/$(HOST_ARCH)/$(TARGET)

SRC_DIR := src
KERNEL_DIR := hls
VITIS_VISION_LIB_DIR := extern/vitis_lib/vision

# sets various useful variables, if using v++
ifeq ($(filter sw, $(TARGET)),)
ifneq ($(filter accel package app debug-all debug-app vivado, $(MAKECMDGOALS)),)
include utils.mk
endif
endif

# ------------------------- Setting correct compiler ------------------------- #

ifeq ($(HOST_ARCH), aarch64)
CXX := $(XILINX_VITIS)/gnu/aarch64/lin/aarch64-linux/bin/aarch64-linux-gnu-g++
else ifeq ($(HOST_ARCH), x86)
CXX := g++
endif

ifndef DEBUG
ifeq ($(HOST_ARCH), aarch64)
OPT_FLAGS := -mcpu=cortex-a72.cortex-a53 -march=armv8-a+crc -fstack-protector-strong -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security -Ofast
else
OPT_FLAGS := -march=native -mtune=native
endif
else
$(info debug is $(DEBUG))
OPT_FLAGS := -g3 -O0
$(info $(OPT_FLAGS))
endif

# ------------------------------- Temp folders ------------------------------- #

TEMP_DIR := tmp/$(TARGET)_$(PLATFORM_NAME)
TEMP_REPORT_DIR := $(TEMP_DIR)/report
TEMP_LOG_DIR := $(TEMP_DIR)/logs

# ----------------------------- Host app section ----------------------------- #

IDIRS := src
IDIRS += extern/eigen
IDIRS += /home/dshem/Documents/these/ressources/papers/rapido2023/experiments/auction_c/include
XF_VIDEO_IDIR := $(VITIS_VISION_LIB_DIR)/L1/include/video

ifeq ($(HOST_ARCH), aarch64)
IDIRS +=  $(SYSROOT)/usr/include/opencv4 
# IDIRS += $(SYSROOT)/usr/include
endif
ifeq ($(HLS), on)
IDIRS += $(SYSROOT)/usr/include/xrt $(XILINX_HLS)/include $(VITIS_VISION_LIB_DIR)/ext/xcl2 $(VITIS_VISION_LIB_DIR)/L1/include $(XF_VIDEO_IDIR) $(KERNEL_DIR)
endif

# generate include flags
IFLAGS := $(addprefix -I, $(IDIRS))

# generate srcs and filter out the main files, those will be added based on target
ALL_MAINS := $(wildcard $(SRC_DIR)/main*.cpp)
HOST_SRCS := $(filter-out $(ALL_MAINS), $(wildcard $(SRC_DIR)/*.cpp))
# HOST_SRCS := $(wildcard $(SRC_DIR)/*.cpp)

ifeq ($(HOST_ARCH), aarch64)
ifeq ($(HLS), on)
HOST_SRCS += $(wildcard $(KERNEL_DIR)/*.cpp)
HOST_SRCS += $(VITIS_VISION_LIB_DIR)/ext/xcl2/xcl2.cpp
endif
endif

HOST_SRCS_FILES_ONLY := $(notdir $(HOST_SRCS))
OBJS := $(addprefix $(BUILD_DIR)/, $(HOST_SRCS_FILES_ONLY:.cpp=.o))
DEP_FILES := $(OBJS:%.o=%.d)

# common compile flags
CXXFLAGS += $(IFLAGS) -std=c++17 -Wall -MMD -MP -Wno-unknown-pragmas -Wno-unused-label -fmessage-length=0 
LDFLAGS += -pthread -Wl,--as-needed
VPP_FLAGS += -t $(TARGET) --platform $(PLATFORM) --save-temps
VPP_LDFLAGS += --optimize 2 -R 2

# setup host dependant cxx flags
ifeq ($(HOST_ARCH), x86)
CXXFLAGS += $(shell pkg-config --cflags opencv4)
LDFLAGS += $(shell pkg-config --libs opencv4)
else ifeq ($(HOST_ARCH), aarch64)
CXXFLAGS += --sysroot=$(SYSROOT)
ifeq ($(HLS), on)
CXXFLAGS += -DKALMAN_ACCEL 
endif
LDFLAGS += -L$(SYSROOT)/usr/lib -L$(SYSROOT)/lib
LDFLAGS += --sysroot=$(SYSROOT)
ifeq ($(HLS), on)
LDFLAGS += -lxrt_coreutil -lxilinxopencl
endif
LDFLAGS += -lopencv_videoio -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_flann -lopencv_video -lopencv_calib3d -lopencv_highgui
LIBRARY_PATH := $(OPENCV_LIB):$(LD_LIBRARY_PATH)
ifeq ($(HLS), on)
LIBRARY_PATH := $(LD_LIBRARY_PATH):$(XILINX_XRT)/lib
endif
endif

# Host app building, when compiler changes
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp Makefile
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJS): |$(BUILD_DIR)

$(BUILD_DIR)/mainyolodpu.o: $(SRC_DIR)/mainyolodpu.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/mainsort.o: $(SRC_DIR)/mainsort.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/mainboxdemo.o: $(SRC_DIR)/mainboxdemo.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/xcl2.o: $(VITIS_VISION_LIB_DIR)/ext/xcl2/xcl2.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Only links the selected application
$(BUILD_DIR)/$(APP_NAME): $(OBJS) $(BUILD_DIR)/$(MAIN_FILE:%.cpp=%.o)
	$(CXX) $(filter-out $(subst $(SRC_DIR)/,$(BUILD_DIR)/,$(ALL_MAINS:%.cpp=%.o)),$(OBJS)) $(BUILD_DIR)/$(MAIN_FILE:%.cpp=%.o) $(LDFLAGS) -o $@

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
app: CXXFLAGS += $(OPT_FLAGS)
app: $(BUILD_DIR)/$(APP_NAME)

.PHONY: sort-app
sort-app: MAIN_FILE := mainsort.cpp
sort-app: app

.PHONY: yolo-app
yolo-app: CXXFLAGS += -DDPUYOLO
yolo-app: MAIN_FILE := mainyolodpu.cpp
yolo-app: LDFLAGS += -lvitis_ai_library-yolov3 -lvitis_ai_library-dpu_task -lvitis_ai_library-xnnpp -lvitis_ai_library-model_config -lvitis_ai_library-math -lvart-util -lxir -pthread -ljson-c -lglog -lturbojpeg 
yolo-app: app

.PHONY: accel
accel: $(XCLBIN_FILES)

.PHONY: vivado
vivado: accel
	$(VPP) -t hw --link --platform $(PLATFORM) -o $(XCLBIN_FILES_PKG) $(XCLBIN_OBJS) --temp_dir $(TEMP_DIR) --report_dir $(TEMP_REPORT_DIR) --log_dir $(TEMP_LOG_DIR) --save-temps --advanced.misc solution_name=link --connectivity.nk kalmanfilter_accel:1:kalman_filter_accel_1 --interactive impl

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