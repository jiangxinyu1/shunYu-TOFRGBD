cmake_minimum_required(VERSION 2.8.3)
project(camera_lcm_demo_sy)

set(CMAKE_CXX_FLAGS "-pthread")
set(Thirdparty_DIR ~/cleanRobot/thirdparty)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${Thirdparty_DIR}/CMakeModules")
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

FIND_PACKAGE(lcm REQUIRED) 
FIND_PACKAGE(Threads)

include_directories(
    ${INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/src/include
    ${PROJECT_SOURCE_DIR}/TOF_SDK/T00P06AN/include
    ${Thirdparty_DIR}/include
    # ${Thirdparty_DIR}/include/lcm_msgs
    # ${Thirdparty_DIR}/include/lcm_msgs/lcm_std_msgs
    # ${Thirdparty_DIR}/include/lcm_msgs/lcm_sensor_msgs
    # ${Thirdparty_DIR}/include/lcm_msgs/lcm_geometry_msgs
    # ${LCM_INCLUDE_DIR}
)

FILE(
  GLOB
  SO_FILES
  ${PROJECT_SOURCE_DIR}/TOF_SDK/T00P06AN/libs/*
  # ${Thirdparty_DIR}/lib/*
  # ${Thirdparty_DIR}/lib/libglib-2.0.so
  # ${Thirdparty_DIR}/lib/libpcre.so
)

FILE(
  GLOB
  SRC_FILES
  ${PROJECT_SOURCE_DIR}/src/camera_control.cpp
  ${PROJECT_SOURCE_DIR}/src/camerademo.cpp
  ${PROJECT_SOURCE_DIR}/src/rgb_camera_control.cpp
)

add_executable(camera_lcm_demo_sy ${SRC_FILES})
target_link_libraries(camera_lcm_demo_sy ${SO_FILES} ${PTHREAD})

# ${LCM_LIBRARY} 