
1 sunnytest为编译生成的目录，adb push目录到板子/userdata，运行run.sh，输入c命令抓一帧点云，分辨率224*114,10fps，q命令退出

2 TOF_SDK为tof模组T00P06AN的算法库头文件，库，参数文件

3 camerademo.cpp为demo

4 修改ifeq ($(COMPILATION_TOOLS_VERSION), BUILDROOT)里的 GCC_DIR，指定编译器。







