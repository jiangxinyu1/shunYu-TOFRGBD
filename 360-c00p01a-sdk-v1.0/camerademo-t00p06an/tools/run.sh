#!/bin/sh

mkdir -p /userdata/sunnytest/capture/
rm -rf /userdata/sunnytest/capture/*
export LD_LIBRARY_PATH="/userdata/sunnytest/libs:${LD_LIBRARY_PATH}"

media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":0[fmt:SBGGR12/224x2193]' 
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":0[fmt:SBGGR12/224x2193]' --set-v4l2 '"rkisp-isp-subdev":0[crop:(0,0)/224x2408]'
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":2[fmt:SBGGR12/224x2193]' 
media-ctl -d /dev/media1 --set-v4l2 '"rkisp-isp-subdev":2[fmt:SBGGR12/224x2193]' --set-v4l2 '"rkisp-isp-subdev":2[crop:(0,0)/224x2408]'
media-ctl --set-v4l2 '"rkisp-isp-subdev":2[fmt:SRGGB12_1X12/224x2193]'

chmod 777 /userdata/sunnytest/camerademo
cd /userdata/sunnytest/
./camerademo 



