<!--
 * @Author: your name
 * @Date: 2022-01-24 11:22:53
 * @LastEditTime: 2022-01-24 14:23:11
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /camera_lcm_demo_SY/tools/readme.md
-->


# camera _lcm_demo_SY 工程使用说明

## 1 文件目录

TOF_SDK 为tof模组T00P06AN的算法库头文件，库，参数文件。更新版本只更新参数及动态库。

tools 存放在板子上运行程序的脚本文件。

bin 为编译出的可执行文件

sunnytest 是为运行程序准备生成的目录，通过脚本文件、将参数文件、可执行程序、动态库、运行脚本拷贝到sunnytest目录。

## 2 操作方法

### 1 编译

通过工程根目录下 arm_toolchain_rk.cmake 指定编译器。

(1) 执行

```shell
./build.sh
```
会编译生成 sy_camerademo 和 sy_camerademo_lcm

(2) 准备sunnytest目录，执行

```shell
./prepare.sh
```

(3) adb push目录到板子/userdata, 执行
```shell
adb push sunnytest /data/
```

### 2 到扫地机/data/sunnytest/目录

(1) 如果需要使用舜宇提供的原始demo，运行
```shell
./run.sh
```
输入c命令抓一帧点云，分辨率224*114,10fps，q命令退出，p命令抓10帧点云

(2) 如果需要使用LCM实时看数据，运行

```shell
./lcmRun.sh
```

q命令退出