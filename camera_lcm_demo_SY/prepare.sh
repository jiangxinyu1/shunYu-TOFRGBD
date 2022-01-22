###
 # @Author: jiangxinyu
 # @Date: 2022-01-22 14:30:04
 # @LastEditTime: 2022-01-22 14:56:17
 # @LastEditors: Please set LastEditors
 # @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 # @FilePath: /camera_lcm_demo_SY/prepare.sh
### 
#!/bin/bash

echo "Prepare sunnytest dir ... "

if [ ! -d "sunnytest" ]; then
    mkdir sunnytest
fi

if [ ! -d "sunnytest/libs" ]; then
    mkdir sunnytest/libs
fi

if [ ! -d "sunnytest/parameter" ]; then
    mkdir sunnytest/parameter
fi

outputDir="./sunnytest/"
libsDir="./TOF_SDK/T00P06AN/libs/"
toolsDir="./tools"
parameterDir="./TOF_SDK/T00P06AN/parameter/"
binDir="./bin/"

cp -rf ${libsDir} ${outputDir}
cp -rf ${parameterDir} ${outputDir}
cp -rf ${toolsDir}/* ${outputDir}
cp -rf ${binDir}/* ${outputDir}

# echo "cp -rf ${libsDir} ${outputDir}"
# echo "cp -rf ${parameterDir} ${outputDir}}"
# echo "cp -rf ${toolsDir}/* ${outputDir}"

echo "Prepare done"
date
