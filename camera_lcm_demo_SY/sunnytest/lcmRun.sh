###
 # @Author: jiangxinyu
 # @Date: 2022-01-22 14:39:56
 # @LastEditTime: 2022-01-22 14:39:57
 # @LastEditors: Please set LastEditors
 # @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 # @FilePath: /camera_lcm_demo_SY/sunnytest/lcmRun.sh
### 
#!/bin/sh
mkdir -p /userdata/sunnytest/capture/
rm -rf /userdata/sunnytest/capture/*
export LD_LIBRARY_PATH="/userdata/sunnytest/libs:${LD_LIBRARY_PATH}"

v4l2-ctl -d /dev/video0 --set-fmt-video=width=224,height=2193,pixelformat=BG12 --set-crop=top=0,left=0,width=224,height=2193 --stream-mmap=4 --stream-count=1

chmod 777 /userdata/sunnytest/sy_camerademo_lcm
cd /userdata/sunnytest/
./sy_camerademo_lcm

