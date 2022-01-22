/*
 * @Author: your name
 * @Date: 2022-01-19 12:06:18
 * @LastEditTime: 2022-01-21 18:53:12
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /camera_lcm_demo_SY/src/include/tof_depth_process.h
 */
#ifndef __TOF_DEPTH_PROCESS_H__
#define __TOF_DEPTH_PROCESS_H__

#include <string>

#include "tof_mod_sdk.h"
#include "tof_error.h"
#include "tof_typedef.h"
#include "camera_control.h"

typedef enum tof_exp_mode {
	SINGLE_FRAME_MODE,
	LONG_SHORT_FRAME_MODE,
} TOF_EXP_MODE_E;

typedef struct set_tof_exp{
  HTOFM hTofMod;
  TOF_EXP_MODE_E eExpMode;
  UINT32 expTime;
  UINT32 expTime_AEF;
  UINT32 expTime_FEF;
  UINT32 flag;
}Tof_SetExp;

void Net_SetTofFilter(const TOF_FILTER type, const SBOOL bEnable);
void Net_SetTofRemoveINS(const SBOOL bEnable);

int TofDepthSdkInit(char *pcCalibFilePath);
int TofDepthSdkUnInit();
int TofDepthProcess(TofRawData *pRawData, TofModDepthData *pstTofDepthDataInfo);
void HandleDepthData_For360(const UINT32 threadIndex, UINT32 frameIndex, std::string& strSaveDir, TofModDepthData* tofFrameData);
void HandleDepthData(const UINT32 threadIndex, UINT32 frameIndex, std::string& strSaveDir, TofModDepthData* tofFrameData);

bool SavePointDataXYZText(PointData *pPointData, const UINT32 width, const UINT32 height, char* pTxtFile);

#endif