ifeq ($(filter clean clean-pkg clean-app docs, $(MAKECMDGOALS)),)
ifeq ($(filter aarch64 x86, $(HOST_ARCH)),)
$(error [ERROR]: HOST_ARCH variable is not correctly set. Set it to x86 or aarch64)
endif
endif

BUILD_DIR := build/$(HOST_ARCH)

SRC_DIR := src
SYSROOT := sysroot/sysroots/cortexa72-cortexa53-xilinx-linux

# ------------------------- Setting correct compiler ------------------------- #

ifeq ($(HOST_ARCH), aarch64)
CXX := sysroot/sysroots/x86_64-petalinux-linux/usr/bin/aarch64-xilinx-linux/aarch64-xilinx-linux-g++
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
OPT_FLAGS := -ggdb -O0
endif

# ----------------------------- Host app section ----------------------------- #
APPTARGET := $(filter yolo-app,$(MAKECMDGOALS))

ifeq ($(APPTARGET), yolo-app)
MAIN_FILE := mainyolodpu.cpp
APP_NAME := sortzynq
endif

ifeq ($(DEBUG), 1)
APP_NAME := $(APP_NAME)_dbg
endif

IDIRS := src
IDIRS += extern/eigen
IDIRS += extern/auction/include
ifeq ($(HOST_ARCH), aarch64)
IDIRS += sysroot/sysroots/cortexa72-cortexa53-xilinx-linux/usr/include
endif

ifeq ($(HOST_ARCH), aarch64)
IDIRS +=  $(SYSROOT)/usr/include/opencv4 
endif

# generate include flags
IFLAGS := $(addprefix -I, $(IDIRS))

# generate srcs and filter out the main files, those will be added based on target
ALL_MAINS := $(wildcard $(SRC_DIR)/main*.cpp)
HOST_SRCS := $(filter-out $(ALL_MAINS), $(wildcard $(SRC_DIR)/*.cpp))

HOST_SRCS_FILES_ONLY := $(notdir $(HOST_SRCS))
OBJS := $(addprefix $(BUILD_DIR)/, $(HOST_SRCS_FILES_ONLY:.cpp=.o))
DEP_FILES := $(OBJS:%.o=%.d)

# common compile flags
CXXFLAGS += $(IFLAGS) -std=c++17 -Wall -MMD -MP -Wno-unknown-pragmas -Wno-unused-label -fmessage-length=0 
LDFLAGS += -pthread -Wl,--as-needed

# setup host dependant cxx flags
ifeq ($(HOST_ARCH), x86)
CXXFLAGS += $(shell pkg-config --cflags opencv4)
LDFLAGS += $(shell pkg-config --libs opencv4)
else ifeq ($(HOST_ARCH), aarch64)
CXXFLAGS += --sysroot=$(SYSROOT)
CXXFLAGS += -DDPUYOLO
LDFLAGS += -lvitis_ai_library-yolov3 -lvitis_ai_library-dpu_task -lvitis_ai_library-xnnpp -lvitis_ai_library-model_config -lvitis_ai_library-math -lvart-util -lxir -pthread -ljson-c -lglog -lturbojpeg -lboost_filesystem -lboost_program_options

LDFLAGS += -L$(SYSROOT)/usr/lib -L$(SYSROOT)/lib
LDFLAGS += --sysroot=$(SYSROOT)
LDFLAGS += -lopencv_videoio -lopencv_imgcodecs -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_flann -lopencv_video -lopencv_calib3d -lopencv_highgui
LIBRARY_PATH := $(OPENCV_LIB):$(LD_LIBRARY_PATH)
endif

# Host app building, when compiler changes
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp Makefile
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJS): |$(BUILD_DIR)

$(BUILD_DIR)/mainyolodpu.o: $(SRC_DIR)/mainyolodpu.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Only links the selected application
$(BUILD_DIR)/$(APP_NAME): $(OBJS) $(BUILD_DIR)/$(MAIN_FILE:%.cpp=%.o)
	$(CXX) $(filter-out $(subst $(SRC_DIR)/,$(BUILD_DIR)/,$(ALL_MAINS:%.cpp=%.o)),$(OBJS)) $(BUILD_DIR)/$(MAIN_FILE:%.cpp=%.o) $(LDFLAGS) -o $@

# ------------------------------- Named targets ------------------------------ #
.PHONY: app
app: CXXFLAGS += $(OPT_FLAGS)
app: $(BUILD_DIR)/$(APP_NAME)

# TODO: Manage source files more properly depending on the target
.PHONY: yolo-app
yolo-app: app

.PHONY: docs
docs:
	doxygen Doxyfile
	make -C docs/doxygen/latex

.PHONY: debug-app
debug-app: CXXFLAGS += -g3 -O0
debug-app: app

.PHOMY: clean-app
clean-app:
	@rm -f $(BUILD_DIR)/$(APP_NAME)

.PHONY: clean
clean:
	@rm -rf $(BUILD_DIR)/*
	# @rm -rf $(TEMP_DIR)
	# @rm -rf .Xil

-include $(wildcard $(DEP_FILES))