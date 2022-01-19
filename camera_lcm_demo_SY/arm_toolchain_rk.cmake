if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColorReset "${Esc}[m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
endif()

# this one is important
SET (CMAKE_SYSTEM_NAME Linux)
if (DEFINED ENV{ARM64_GCC_RK_HOME})
  message(STATUS "${BoldYellow}Use compiler provided by User${ColorReset}")
  SET (CMAKE_C_COMPILER $ENV{ARM64_GCC_RK_HOME}/bin/aarch64-rockchip-linux-gnu-gcc)
  SET (CMAKE_CXX_COMPILER $ENV{ARM64_GCC_RK_HOME}/bin/aarch64-rockchip-linux-gnu-g++)
  MESSAGE("<<<<<<< CMAKE_C_COMPILER = " ${CMAKE_C_COMPILER})
elseif (DEFINED ENV{ARM64_BUILDROOT_PATH})
  message(STATUS "${BoldYellow}Use compiler provided by BuildRoot${ColorReset}")
  SET (CMAKE_C_COMPILER $ENV{ARM64_BUILDROOT_PATH}/bin/aarch64-rockchip-linux-gnu-gcc)
  SET (CMAKE_CXX_COMPILER $ENV{ARM64_BUILDROOT_PATH}/bin/aarch64-rockchip-linux-gnu-g++)
  MESSAGE(">>>>>> CMAKE_C_COMPILER = " ${CMAKE_C_COMPILER})
else () 
  message(FATAL_ERROR "${BoldYellow}Cannot find toolchain for arm64${ColorReset}")
endif ()

# where is the target environment
SET (CMAKE_FIND_ROOT_PATH "$ENV{HOME}/cleanRobot/thirdparty")

# search for programs in the build host directories
SET (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
SET (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

SET(IS_ARM TRUE)
add_definitions(-DUSER="$ENV{USER}")

message(STATUS "${BoldYellow}Target: ARM 64 bit ${ColorReset}")
