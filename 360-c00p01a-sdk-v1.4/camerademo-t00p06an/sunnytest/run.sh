#!/bin/sh

mkdir -p /userdata/sunnytest/capture/
rm -rf /userdata/sunnytest/capture/*
export LD_LIBRARY_PATH="/userdata/sunnytest/libs:${LD_LIBRARY_PATH}"

v4l2-ctl -d /dev/video0 --set-fmt-video=width=224,height=2193,pixelformat=BG12 --set-crop=top=0,left=0,width=224,height=2193 --stream-mmap=4 --stream-count=1

chmod 777 /userdata/sunnytest/camerademo
cd /userdata/sunnytest/
./camerademo 

