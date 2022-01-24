#ifndef __RGBD_H__
#define __RGBD_H__

#include "tof_rgbd_sdk.h"
#include <string>

#ifndef MTS001SDK_MAJOR
#define MTS001SDK_MAJOR 1
#endif

#ifndef MTS001SDK_MINOR
#define MTS001SDK_MINOR 4
#endif

unsigned long long Utils_GetTickCount(void);
void HandleTofRgbdOutputData(unsigned int frameIndex, const std::string& strSaveDir, TofRgbdOutputData* rgbdData);

const char* tof_get_sdk_version();

#endif
