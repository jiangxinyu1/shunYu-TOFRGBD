#!bin/sh
###TOF TYPE: MTP009A_9FPS  MTP009A_17FPS MTP012_9FPS MTP012_17FPS  
LIB_TYPE ?= DYNAMIC_LIB_TYPE
COMPILATION_TOOLS_VERSION ?= BUILDROOT
TOF_TYPE := MTP013_17FPS
TOF_SDK_TYPE := T00P06AN
TRUNK_DIR := $(shell pwd)
TOF_DIR := $(TRUNK_DIR)/TOF_SDK/$(TOF_SDK_TYPE)
TOOL_DIR := $(TRUNK_DIR)/tools/
OUT_DIR := $(TRUNK_DIR)/sunnytest/

TARGET := $(TRUNK_DIR)/bin/camerademo
TARGET += $(OUT_DIR)/camerademo


INCLUDES += -I$(TOF_DIR)/include

LIBS += -lpthread -ldl -lrt -L $(TOF_DIR)/libs
	
ifeq ($(COMPILATION_TOOLS_VERSION), UBUNTU)
	GCC_DIR :=/usr/local/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin
	CC  :=$(GCC_DIR)/aarch64-linux-gnu-gcc 
	GCC :=$(GCC_DIR)/aarch64-linux-gnu-g++ 
	AR  :=$(GCC_DIR)/aarch64-linux-gnu-ar
	LIBS += -ltof_mod_sdk
else
ifeq ($(COMPILATION_TOOLS_VERSION), BUILDROOT)
	GCC_DIR :=/home/xinyu/software/gcc-linaro-6.4.1-2018.05-x86_64_aarch64-linux-gnu/bin/
	CC  :=$(GCC_DIR)aarch64-linux-gnu-gcc
	GCC :=$(GCC_DIR)aarch64-linux-gnu-g++
	AR  :=$(GCC_DIR)aarch64-linux-gnu-ar
	#LIBS += -ltof_mod_sdk_buildroot
	LIBS += -ltof_mod_sdk -lsunnytof
endif
endif

	
SRCS = camerademo.cpp camera_control.cpp
OBJS = $(SRCS:.c=.o)
OBJS = $(SRCS:.cpp=.o)

.PHONY: all 

all: clean $(TARGET)

%.o: %.c
	$(CC)  $(INCLUDES) -fpermissive -c -o $@ $< 
	
%.o: %.cpp
	$(GCC) -fpermissive -std=c++11 $(INCLUDES) -c -o $@ $<
	
$(TARGET): $(OBJS)

	$(GCC) -fpermissive -std=c++11 -o $@ $(OBJS) $(LIBS)
	cp -rf $(TOOL_DIR)/* $(OUT_DIR)
	cp -rf $(TOF_DIR)/parameter/ $(OUT_DIR)
	cp -rf $(TOF_DIR)/libs/ $(OUT_DIR)

clean:
	rm -rf $(TARGET) *.o *.a *~ *.bak 
	rm -rf $(OUT_DIR)/*
	
	