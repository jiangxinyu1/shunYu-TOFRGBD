CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(camera_lcm_demo_sy)

SET(CMAKE_CXX_FLAGS "-pthread")

SET(Thirdparty_DIR $ENV{HOME}/cleanRobot/thirdparty)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${Thirdparty_DIR}/CMakeModules")
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

FIND_PACKAGE(lcm REQUIRED) 
FIND_PACKAGE(Threads)

INCLUDE_DIRECTORIES(
    ${INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src/include
    ${PROJECT_SOURCE_DIR}/TOF_SDK/T00P06AN/include
    ${Thirdparty_DIR}/include
    ${Thirdparty_DIR}/include/lcm_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_std_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_sensor_msgs
    ${Thirdparty_DIR}/include/lcm_msgs/lcm_geometry_msgs
    ${LCM_INCLUDE_DIR}
)

FILE(
  GLOB
  SO_FILES
  # ${Thirdparty_DIR}/lib/*so
  ${Thirdparty_DIR}/lib/libglib-2.0.so
  ${Thirdparty_DIR}/lib/libpcre.so
  ${PROJECT_SOURCE_DIR}/TOF_SDK/T00P06AN/libs/*
)
# MESSAGE(">>>>>> Thirdparty_DIR = " ${Thirdparty_DIR})
# MESSAGE(">>>>>> SO_FILES = " ${SO_FILES})

FILE(
  GLOB
  SRC_FILES
  ${PROJECT_SOURCE_DIR}/src/camera_control.cpp
  ${PROJECT_SOURCE_DIR}/src/camerademo.cpp
  ${PROJECT_SOURCE_DIR}/src/rgb_camera_control.cpp
)

ADD_EXECUTABLE(camera_lcm_demo_sy ${SRC_FILES})
TARGET_LINK_LIBRARIES(camera_lcm_demo_sy ${SO_FILES} ${LCM_LIBRARY})