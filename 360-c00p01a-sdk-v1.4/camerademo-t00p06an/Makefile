#!bin/sh
###TOF TYPE: MTP009A_9FPS  MTP009A_17FPS MTP012_9FPS MTP012_17FPS  
#LIB_TYPE ?= DYNAMIC_LIB_TYPE
COMPILATION_TOOLS_VERSION ?= BUILDROOT
#TOF_TYPE := MTP013_17FPS
TOF_SDK_TYPE := T00P06AN
TRUNK_DIR := $(shell pwd)
TOF_DIR := $(TRUNK_DIR)/TOF_SDK/$(TOF_SDK_TYPE)
TOOL_DIR := $(TRUNK_DIR)/tools/
OUT_DIR := $(TRUNK_DIR)/sunnytest/

TARGET := $(TRUNK_DIR)/bin/camerademo
TARGET += $(OUT_DIR)/camerademo


INCLUDES += -I$(TOF_DIR)/include -I ./

LIBS += -lpthread  -L $(TOF_DIR)/libs
###-ldl -lrt

ifeq ($(COMPILATION_TOOLS_VERSION), UBUNTU)
	GCC_DIR :=/usr/local/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin
	CC  :=$(GCC_DIR)/aarch64-linux-gnu-gcc 
	GCC :=$(GCC_DIR)/aarch64-linux-gnu-g++ 
	AR  :=$(GCC_DIR)/aarch64-linux-gnu-ar
	LIBS += -ltof_mod_sdk -lrga -lrgbd_registration -lsunnytof
else
ifeq ($(COMPILATION_TOOLS_VERSION), BUILDROOT)
	GCC_DIR :=/home/zhangdq/1204fs/zhangdq/360_tof/360_3566system/buildroot/output/rockchip_rk356x_robot/host/usr/bin
	CC  :=$(GCC_DIR)/aarch64-linux-gcc
	GCC :=$(GCC_DIR)/aarch64-linux-g++
	AR  :=$(GCC_DIR)/aarch64-linux-ar
	#LIBS += -ltof_mod_sdk_buildroot
	LIBS += -ltof_mod_sdk -lrga -ltof_rgbd_sdk -lsunnytof
endif
endif

	
SRCS = camerademo.cpp camera_control.cpp rgb_camera_control.cpp
OBJS = $(SRCS:.c=.o)
OBJS = $(SRCS:.cpp=.o)

.PHONY: all 

all: clean $(TARGET)

%.o: %.c
	$(CC)  $(INCLUDES)  -c -o $@ $< 
## -fpermissive

%.o: %.cpp
	$(GCC)  $(INCLUDES) -c -o $@ $<
## -fpermissive -std=c++11 
$(TARGET): $(OBJS)


	$(GCC)  -o $@ $(OBJS) $(LIBS)
##-fpermissive -std=c++11
	cp -rf $(TOOL_DIR)/* $(OUT_DIR)
	cp -rf $(TOF_DIR)/parameter/ $(OUT_DIR)
	cp -rf $(TOF_DIR)/libs/ $(OUT_DIR)

clean:
	rm -rf $(TARGET) *.o *.a *~ *.bak 
	rm -rf $(OUT_DIR)/*
	
	