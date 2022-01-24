
#include <stdlib.h> 
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace chrono;

#include "camerademo.h"
#include "camera_control.h"
#include "tof_depth_process.h"
#include "rgbd.h"
#include "tof_rgbd_sdk.h"

#include "calib_params_def.h"

typedef struct tagDevBasicInfo
{
 unsigned char szSycID[4];
 unsigned char byVersion;
 unsigned char szRes1[3];

 char szDeviceSN[64];
 char szFirmwareVersion[16];
 float szLensParamter[9];

 unsigned char szRes2[388];

}DevBasicInfo;

static unsigned char pointcloud_tmp[SEND_BUFF_SIZE];

static TOF_RAW_DATA_CB_S gstTofRawDataCb = {0};
static TOF_OUT_DATA_CB_S gstTofOutDataCb = {0};

static pthread_t stRGBRawTrdId;///rgb
static pthread_t stRGBDRawTrdId;///rgbd

static pthread_t stTofRawTrdId;
static pthread_t stDepthCalTrdId;
static pthread_t stOutDataSendTrdId;
static pthread_mutex_t sttofOutDataMutex;
static pthread_mutex_t strgbOutDataMutex;

static int isRuning = 1;
static int isRGBRuning = 1;

static int giCaptureFlag = 0;
static int giCaptureFlag_10frame = 0;

static int save_data_count = 11;

unsigned int guiRawCaptureFlag;/////for gui
int first_ten_frame_info;
static int giLocalTestFlag = 0;
static int giLocalTestNum = 0;
static int giLocalTestStart = 0;

static int capture_ir = 0;

static unsigned long long get_tick_count()
{
	unsigned long long tick = 0;

	struct timeval tv;
	gettimeofday(&tv, 0);
	tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);

	return tick;
}

static void test_rgbd_fps(void)
{
    static int frames_num = 0;
    static long start_time = 1;

    if (start_time == 1) {
        start_time = get_tick_count();
    }

    frames_num++;

    if (frames_num % 10 == 0) {
        float cur_framerate = 0;
        cur_framerate = (float)frames_num * 1000 / (get_tick_count() - start_time);

        
        printf("$$$$$$$$$$$$$$$$$$$$ RGBD Frame Rate = %.2f\n\n", cur_framerate);
        start_time = get_tick_count();
        frames_num = 0;
    }
}

int time_start(struct timeval *pstTimeVal)
{	
	gettimeofday(pstTimeVal, NULL);
	return 0;
}

int time_stop(struct timeval *pstTimeVal)
{
	unsigned int uiTime = 0;
	struct timeval stStopTimeVal = {0};

	gettimeofday(&stStopTimeVal, NULL);

	uiTime = (stStopTimeVal.tv_sec - pstTimeVal->tv_sec)*1000 + (stStopTimeVal.tv_usec - pstTimeVal->tv_usec)/1000;
	printf("cost time: %u\n\n", uiTime);
	
	return 0;
}

int FileWrite(char *pcFilePath, unsigned char *pucData, unsigned int uiSize)
{
	int iFd;

	iFd = open(pcFilePath, O_CREAT|O_RDWR);
	if (iFd < 0)
	{
		printf("open %s failed\n", pcFilePath);
		return -1;
	}

	write(iFd, pucData, uiSize);
	close(iFd);

	printf("write to %s done\n", pcFilePath);
	return 0;
}

void *TofOutDataHandler(void *para)
{   
	unsigned int uiPointcloud_len = 0;
	unsigned int uiGrey_len = 0;
	unsigned int uiPixelNum = 0;

	DevBasicInfo senddevinfo;
	
	TOF_OUT_DATA_CB_S *pstTofOutDataCb = &gstTofOutDataCb; 
	TofModDepthData *pstTofDepthData = &pstTofOutDataCb->stDepthDataInfo;

	while(isRuning)
	{
		if (pstTofOutDataCb->isAvailable)
		{
			uiPixelNum = pstTofDepthData->frameWidth * pstTofDepthData->frameHeight;
			uiPointcloud_len = uiPixelNum * sizeof(float)*3;
			uiGrey_len = uiPixelNum * sizeof(float);
		
			pstTofOutDataCb->isAvailable = 0;
		}
		else
		{
			usleep(5000);
		}
	}

	return NULL;
}

static unsigned char *rgb_image_buf = NULL;

void *GetRGBRawDataHandler(void *para)
{
	int iRet = 0;

	static int cap_rgb_count = 0;
	
	while(isRGBRuning)
	{
		pthread_mutex_lock(&strgbOutDataMutex);
		iRet = GetRGBRawData(rgb_image_buf);

		if (iRet)
		{	
			usleep(5000);
			continue;
		}
		pthread_mutex_unlock(&strgbOutDataMutex);

		usleep(1000);
	}

	return NULL;
}

void *GetTofRawDataHandler(void *para)
{
	int iRet = 0;
 
	TOF_RAW_DATA_CB_S *pstTofRawDataCb = &gstTofRawDataCb;
	
	while(isRuning)
	{
		if (0 == giLocalTestFlag)
		{
			iRet = GetTofRawData(pstTofRawDataCb);
		}
		else
		{
			iRet = GetTofRawDataFormLocal(pstTofRawDataCb, giLocalTestNum, giLocalTestStart);
			sleep(1);
		}
		
		if (iRet)
		{	
			usleep(5000);
			continue;
		}
		else
		{		
			if (0 == pstTofRawDataCb->isProcess)
			{
				if (guiRawCaptureFlag)
				{
					char acRawPath[32] = {0};
					snprintf(acRawPath, 32, "./%d-Raw.data", pstTofRawDataCb->uiFrameCnt);
					if(NULL != pstTofRawDataCb->stRawData.pRaw){
					FileWrite(acRawPath, pstTofRawDataCb->stRawData.pRaw, pstTofRawDataCb->stRawData.nRawLen);}
					guiRawCaptureFlag = 0;
				}
			
				pstTofRawDataCb->isAvailable = 1;
			}
			else
			{
				printf("drop tof raw frame[%u]...\n", pstTofRawDataCb->uiFrameCnt);
			}

			usleep(1000);
		}
	}

	return NULL;
}


void *TofDepthProcessHandler(void *para)
{
	int iRet = 0;
	unsigned int uiOutDataSize = 0;
	std::string save_path = "./capture";
	char acRawPath[32] = {0};
	struct timeval stTimeVal = {0};

	TOF_RAW_DATA_CB_S *pstTofRawDataCb = &gstTofRawDataCb;
	TOF_OUT_DATA_CB_S *pstTofOutDataCb = &gstTofOutDataCb;

	while(isRuning)
	{
		if (pstTofRawDataCb->isAvailable && pstTofRawDataCb->stRawData.pRaw)
		{		
			pstTofRawDataCb->isProcess = 1;
		
			pthread_mutex_lock(&sttofOutDataMutex);

			//time_start(&stTimeVal);
      printf("[DEBUG] : Position 0 ... \n");
			iRet = TofDepthProcess(&pstTofRawDataCb->stRawData, &pstTofOutDataCb->stDepthDataInfo);
			//time_stop(&stTimeVal);
			if (iRet)
			{
        printf("TofDepthProcess failed xxxxxxxxxxxxxxxxxxxxxx\n");
			}
			else
			{	
				pstTofOutDataCb->uiFrameCnt = pstTofRawDataCb->uiFrameCnt;
				pstTofOutDataCb->isAvailable = 1;
        printf("[DEBUG] : Position 1 ... \n");
				HandleDepthData(0, pstTofOutDataCb->uiFrameCnt, save_path, &pstTofOutDataCb->stDepthDataInfo);

				if (giCaptureFlag || giLocalTestFlag)
				{
					giCaptureFlag = 0;
          printf("[DEBUG] : Position 2 ... \n");
					HandleDepthData_For360(0, pstTofOutDataCb->uiFrameCnt, save_path, &pstTofOutDataCb->stDepthDataInfo);

					/* Save RAW data */
					snprintf(acRawPath, 32, "./capture/%d-Raw.data", pstTofOutDataCb->uiFrameCnt);
					FileWrite(acRawPath, pstTofRawDataCb->stRawData.pRaw, pstTofRawDataCb->stRawData.nRawLen);
				
					printf("-------------> Capture tof raw done\n");
				}

				if(giCaptureFlag_10frame)
				{
					save_data_count--;
    
					if(save_data_count > 0)
					{
            printf("[DEBUG] : Position 3 ... \n");
						HandleDepthData_For360(0, pstTofOutDataCb->uiFrameCnt, save_path, &pstTofOutDataCb->stDepthDataInfo);

						/* Save RAW data */
						snprintf(acRawPath, 32, "./capture/%d-Raw.data", pstTofOutDataCb->uiFrameCnt);
						FileWrite(acRawPath, pstTofRawDataCb->stRawData.pRaw, pstTofRawDataCb->stRawData.nRawLen);
					}

					if(save_data_count == 0)
					{
						giCaptureFlag_10frame = 0;
						save_data_count = 11;
					}
				}
			}
			pthread_mutex_unlock(&sttofOutDataMutex);
			
			pstTofRawDataCb->isAvailable = 0;
			pstTofRawDataCb->isProcess = 0;
			
		}
		else
		{
			usleep(5000);
			continue;
		}
	}

	return NULL;
}


template <class T>
T Utils_FindMaxValue(T* pData, const int nCnt)
{
	T max = pData[0];

	for (int i = 0; i < nCnt; i++)
	{
		if (max < pData[i])
		{
			max = pData[i];
		}
	}

	return max;
}

static bool SaveIRimage(unsigned char * ir, const UINT32 width, const UINT32 height, char* pTxtFile)
{
	if ((NULL == ir) || (0 >= width) || (0 >= height) || (NULL == pTxtFile))
	{
		return false;
	}

	FILE* fp = fopen(pTxtFile, "w");
	if (NULL == fp)
	{
		return false;
	}

	fwrite((char *)ir,1,width*height,fp);

	fclose(fp);
	return true;
}
static bool CGrayConvert(float* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;

	const float max = Utils_FindMaxValue(pGray, pixel_cnt);

	if (0.001 >= max)
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;
		if (0.001 < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pU8[i] = tmp;
	}

	return true;
}

void *RGBD_Handler(void *para)
{   	
	TOF_OUT_DATA_CB_S *pstTofOutDataCb = &gstTofOutDataCb; 
	TofModDepthData *pstTofDepthData = &pstTofOutDataCb->stDepthDataInfo;

	TOF_RAW_DATA_CB_S *pstTofRawDataCbs = &gstTofRawDataCb;
	
	printf("RGBD SDK Version: %s.\n", TOFRGBD_GetSDKVersion());

	const std::string strSaveDir = ("./capture");//�������ݵ�Ŀ¼

	const std::string strModuleName = ("T00P06AN");//ģ���ͺ�
	const TOFRGBD_GUEST_ID guestID = TOFRGBD_GUEST_ID_DEF;

	const unsigned int nTofWidth = 224;//TOF���ݿ�
	const unsigned int nTofHeight = 114;//TOF���ݸ�
	const unsigned int nRgbWidth = 1920;//RGB���ݿ�
	const unsigned int nRgbHeight = 1080;//RGB���ݸ�

	char ir_data_path[64]={0};

	const TofRgbd_Gray_Format inGrayFormat = TofRgbd_Gray_Format_Float;//����ĻҶȸ�ʽ

	TofRgbdHandleParam struInputParam;
	memset(&struInputParam, 0, sizeof(struInputParam));
	strncpy(struInputParam.szModuleName, strModuleName.c_str(), sizeof(struInputParam.szModuleName) - 1);
	struInputParam.guestID = guestID;

	struInputParam.pRgbdCalibData = (unsigned char*)malloc(sizeof(StereoCameraParameters));	

	struInputParam.nRgbdCalibDataLen = sizeof(StereoCameraParameters);

	StereoCameraParameters pstCamera;
	FILE* fp11 = fopen("./360/2/2.bin", "rb");
	fread(struInputParam.pRgbdCalibData, sizeof(StereoCameraParameters), 1, fp11);
	fclose(fp11);

	struInputParam.nTofWidth = nTofWidth;
	struInputParam.nTofHeight = nTofHeight;

	struInputParam.nRgbWidth = nRgbWidth;
	struInputParam.nRgbHeight = nRgbHeight;

	struInputParam.inGrayFormat = inGrayFormat;	
	
	TofRgbdOutputData struDataOut;
	memset(&struDataOut, 0, sizeof(struDataOut));
	
	HTOFRGBD hTofRgbd = TOFRGBD_CreateHandle(&struInputParam);
	if (NULL == hTofRgbd)
	{
		printf("TOFRGBD_CreateHandle failed.\n");
	}			

	TofRgbdInputData struDataIn;
	memset(&struDataIn, 0, sizeof(struDataIn));

	struDataIn.pPointCloud = (TofRgbdPointCloud*)malloc(sizeof(float) * nTofWidth * nTofHeight * 3);
	if(struDataIn.pPointCloud == NULL)
	{
		printf("malloc error!\n");
	}
	struDataIn.pRgb = (unsigned char*)malloc(nRgbWidth * nRgbHeight * 3);
	if(struDataIn.pRgb == NULL)
	{
		printf("malloc error!\n");
	}
		
	struDataIn.pGray = (unsigned char*)malloc(nTofWidth * nTofHeight * 4);
	if(struDataIn.pGray == NULL)
	{
		printf("malloc error!\n");
	}

	while(isRuning)
	{
		if (pstTofOutDataCb->isAvailable)
		{
			if( (pstTofDepthData->pPointData == NULL) || (rgb_image_buf == NULL) )
			{
				printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
				//return;
			}

	#if 1////save float ir and u8 ir
		capture_ir++;

		sprintf(ir_data_path,"/userdata/sunnytest/capture/iru8-%d.dat", capture_ir);
		
		if(capture_ir > 80 && capture_ir < 90)
		{
			if(capture_ir % 5 == 0)
			{
				pthread_mutex_lock(&sttofOutDataMutex);

				int irFd;
				char acPicPath[64] = {0};				
				snprintf(acPicPath, 64, "./capture/tofirfloat_%d.dat", capture_ir);
				irFd = open(acPicPath, O_CREAT|O_RDWR);
				write(irFd, pstTofDepthData->pGrayData, 224*114*4);///tof ir float format
				close(irFd);
				printf("+++++++++++++++++++++++++++ dump tof ir pic %s\n", acPicPath);
				

				int iIrU8Size = nTofWidth * nTofHeight;
				unsigned char *irDataU8 = (unsigned char*)malloc(sizeof(unsigned char)*iIrU8Size);

				CGrayConvert((float *)(pstTofDepthData->pGrayData),224,114,irDataU8);////tof ir float to U8
				
				SaveIRimage(irDataU8, 224, 114, ir_data_path); ///u8 ir
				free(irDataU8);
				
				pthread_mutex_unlock(&sttofOutDataMutex);							
			}
		}

	#endif	
			struDataIn.nGrayLen = nTofWidth * nTofHeight * 4;//tof ir
			struDataIn.nRgbLen = nRgbWidth * nRgbHeight * 3;//bgr

			memcpy(struDataIn.pPointCloud, (TofRgbdPointCloud*)(pstTofDepthData->pPointData), sizeof(float) * nTofWidth * nTofHeight * 3);
			memcpy(struDataIn.pRgb, rgb_image_buf, nRgbWidth * nRgbHeight * 3);
			memcpy(struDataIn.pGray, (float *)(pstTofDepthData->pGrayData), nTofWidth * nTofHeight * 4);///float ir

			static int loopCnt = 0;
			loopCnt++;
				
			const unsigned long long tick = Utils_GetTickCount();
			const TOFRGBDRET retVal = TOFRGBD_DoCal(hTofRgbd, &struDataIn, &struDataOut);
			//printf("TOFRGBD_DoCal====%llu ms....\n", Utils_GetTickCount() - tick);
			
			if (TOFRGBDRET_SUCCESS == retVal)
			{
				HandleTofRgbdOutputData(loopCnt, strSaveDir, &struDataOut);
			}
			else
			{
				printf("TOFRGBD_DoCal failed, retVal=0x%08x.\n", retVal);
				break;
			}

		pstTofOutDataCb->isAvailable = 0;

		test_rgbd_fps();
			
		}
		else
		{
			usleep(5000);
		}
	}

	free(struInputParam.pRgbdCalibData);
	
	free(struDataIn.pPointCloud);
	free(struDataIn.pRgb);
	free(struDataIn.pGray);
	
	TOFRGBD_CloseHandle(hTofRgbd);

	return NULL;
}

static void ST_Flush(void)
{
    char c;
    while((c = getchar()) != '\n' && c != EOF);
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	int iRet = 0;
	int i;
	int result;
	unsigned int uiSelect;
	char acCalibFilePath[] = "./calib.bin";
	
	TOF_RAW_DATA_CB_S *pstTofRawDataCb = &gstTofRawDataCb;
	TOF_OUT_DATA_CB_S *pstTofOutDataCb = &gstTofOutDataCb;

  printf("\n for 360 tof+rgb+rgbd SDK !\n");
	printf("SDK version: %s\n", tof_get_sdk_version());
	
	memset(pstTofRawDataCb, 0, sizeof(*pstTofRawDataCb));
	memset(pstTofOutDataCb, 0, sizeof(*pstTofOutDataCb));

	pstTofRawDataCb->stRawData.pRaw = NULL;
	
	pthread_mutex_init(&sttofOutDataMutex, NULL);
	pthread_mutex_init(&strgbOutDataMutex, NULL);

	i2c3_open(2);///for depthlib set exp

	if (rgb_image_buf == NULL)
    {
        rgb_image_buf = (unsigned char *)malloc(1920 * 1080 * 3*2);
    }

	while((result = getopt(argc, argv, "n:s:")) != -1)
    {
	   switch(result)
	   {
		   case 'n':
		   	   giLocalTestFlag = 1;
			   giLocalTestNum = strtol(optarg, NULL, 10);
			   break;

		   case 's':
			   giLocalTestStart = strtol(optarg, NULL, 10);
			   break;

		   default: break;
	   }
    }

	printf("local test enable(%d), test num(%d), test start index(%d)\n", giLocalTestFlag, giLocalTestNum, giLocalTestStart);

	if (!giLocalTestFlag)
	{
		iRet = OpenTofCamera();
		if (iRet)
		{
			printf("OpenTofCamera failed\n");
			return -1;
		}
	#if 1
	iRet = OpenRGBCamera();///rgb 
	if (iRet)
	{
		printf("OpenRGBCamera failed\n");
		return -1;
	}
	#endif
	}
	else
	{
		iRet = I2cCbInit();
		if (iRet)
		{
			printf("I2cCbInit failed\n");
		}

		iRet = TofDepthSdkInit(acCalibFilePath);
		if (iRet)
		{
			printf("TofDepthSdkInit failed\n");
		}
	}

	pthread_create(&stTofRawTrdId, NULL, GetTofRawDataHandler, NULL);///tof
	pthread_create(&stRGBRawTrdId, NULL, GetRGBRawDataHandler, NULL);///rgb
	pthread_create(&stDepthCalTrdId, NULL, TofDepthProcessHandler, NULL);///tof depth
	///pthread_create(&stOutDataSendTrdId, NULL, TofOutDataHandler, NULL);
	pthread_create(&stRGBDRawTrdId, NULL, RGBD_Handler, NULL);///rgbd

	while(1)
	{
		char cmd;
		cmd = getchar();
		printf("-------------> cmd = %c\n", cmd);
		ST_Flush();
		switch(cmd)
		{
			case 'q':
				isRuning = 0;
				isRGBRuning = 0;
				printf("-------------> Demo Exit Start\n");
				goto EXIT;

			case 'c':
				giCaptureFlag = 1;
				printf("---------------------------------------------\n");
				printf("--------------------------> Capture Pic Start\n");
				break;

			case 'p':
				giCaptureFlag_10frame = 1;
				printf("**********************************************\n");
				printf("*************************** Capture 10 Pic Start\n");
				break;

			default:
				break;
		}
	}

EXIT:
		pthread_join(stRGBDRawTrdId, NULL);///rgbd
		//pthread_join(stOutDataSendTrdId, NULL);
		pthread_join(stDepthCalTrdId, NULL);		
		pthread_join(stRGBRawTrdId, NULL);
		pthread_join(stTofRawTrdId, NULL);

		i2c3_close();

		if (!giLocalTestFlag)
		{
			CloseTofCamera();
			CloseRGBCamera();
		}
		else
		{
			TofDepthSdkUnInit();
			I2cCbUninit();
		}
		
		printf("-------------> Demo Exit End\n");
	
	return 0;
}

