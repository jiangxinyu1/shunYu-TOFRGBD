#!/bin/bash
###
 # @Author: your name
 # @Date: 2021-12-13 16:30:10
 # @LastEditTime: 2022-01-19 15:46:30
 # @LastEditors: Please set LastEditors
 # @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 # @FilePath: /my_rkaiq_3A_server/build.sh
### 

echo "Configuring and building  ..."
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake ..  -DCMAKE_TOOLCHAIN_FILE=./arm_toolchain_rk.cmake
make -j4
