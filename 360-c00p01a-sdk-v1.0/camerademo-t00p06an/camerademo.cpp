#include "camerademo.h"
#include "camera_control.h"
#include "tof_depth_process.h"

typedef struct tagDevBasicInfo
{
 unsigned char szSycID[4];//同步头，固定为DEV_BASIC_INFO_SYCID
 unsigned char byVersion;//协议版本，最初版本（默认值）为0x00
 unsigned char szRes1[3];//预留，4字节对齐

 char szDeviceSN[64];//设备序列号，设备唯一性识别码，可以是模组id号
 char szFirmwareVersion[16];//固件版本号
 float szLensParamter[9];//TOF模组内参和畸变

 unsigned char szRes2[388];//预留，凑齐512字节

}DevBasicInfo;


static unsigned char pointcloud_tmp[SEND_BUFF_SIZE];

static TOF_RAW_DATA_CB_S gstTofRawDataCb = {0};
static TOF_OUT_DATA_CB_S gstTofOutDataCb = {0};

static pthread_t stTofRawTrdId;
static pthread_t stDepthCalTrdId;
static pthread_t stOutDataSendTrdId;
static pthread_mutex_t stOutDataMutex;

static int isRuning = 1;
static int giCaptureFlag = 0;
unsigned int guiRawCaptureFlag;
int first_ten_frame_info;
static int giLocalTestFlag = 0;
static int giLocalTestNum = 0;
static int giLocalTestStart = 0;


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
			usleep(50);
		}
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
			usleep(50);
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
					FileWrite(acRawPath, pstTofRawDataCb->stRawData.pRaw, pstTofRawDataCb->stRawData.nRawLen);
					guiRawCaptureFlag = 0;
				}
			
				pstTofRawDataCb->isAvailable = 1;
			}
			else
			{
				printf("drop frame[%u]...\n", pstTofRawDataCb->uiFrameCnt);
			}
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
		if (pstTofRawDataCb->isAvailable)
		{		
			pstTofRawDataCb->isProcess = 1;
		
			pthread_mutex_lock(&stOutDataMutex);

			//time_start(&stTimeVal);
			iRet = TofDepthProcess(&pstTofRawDataCb->stRawData, &pstTofOutDataCb->stDepthDataInfo);
			//time_stop(&stTimeVal);
			if (iRet)
			{
				printf("TofDepthProcess failed\n");
			}
			else
			{	
				pstTofOutDataCb->uiFrameCnt = pstTofRawDataCb->uiFrameCnt;
				pstTofOutDataCb->isAvailable = 1;

				if (giCaptureFlag || giLocalTestFlag)
				{
					giCaptureFlag = 0;

					HandleDepthData_For360(0, pstTofOutDataCb->uiFrameCnt, save_path, &pstTofOutDataCb->stDepthDataInfo);

					/* Save RAW data */
					snprintf(acRawPath, 32, "./capture/%d-Raw.data", pstTofOutDataCb->uiFrameCnt);
					FileWrite(acRawPath, pstTofRawDataCb->stRawData.pRaw, pstTofRawDataCb->stRawData.nRawLen);
				
					printf("-------------> Capture tof raw done\n");
				}
			}
			pthread_mutex_unlock(&stOutDataMutex);
			
			pstTofRawDataCb->isAvailable = 0;
			pstTofRawDataCb->isProcess = 0;
			
		}
		else
		{
			usleep(50);
			continue;
		}
	}

	return NULL;
}


static void ST_Flush(void)
{
    char c;
    while((c = getchar()) != '\n' && c != EOF);
}


int main(int argc, char **argv)
{
	int iRet = 0;
	int i;
	int result;
	unsigned int uiSelect;
	char acCalibFilePath[] = "./calib.bin";
	
	TOF_RAW_DATA_CB_S *pstTofRawDataCb = &gstTofRawDataCb;
	TOF_OUT_DATA_CB_S *pstTofOutDataCb = &gstTofOutDataCb;
	
	memset(pstTofRawDataCb, 0, sizeof(*pstTofRawDataCb));
	memset(pstTofOutDataCb, 0, sizeof(*pstTofOutDataCb));

	pstTofRawDataCb->stRawData.pRaw = (unsigned char*)malloc(RAW_SIZE);
	memset(pstTofRawDataCb->stRawData.pRaw, 0, RAW_SIZE);
	
	pthread_mutex_init(&stOutDataMutex, NULL);

	i2c3_open(2);///for depthlib set exp

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

	//for (i = 0; i < 100; i++)
	//{

		if (!giLocalTestFlag)
		{
			iRet = OpenTofCamera();
			if (iRet)
			{
				printf("OpenTofCamera failed\n");
				return -1;
			}
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

		pthread_create(&stTofRawTrdId, NULL, GetTofRawDataHandler, NULL);
		pthread_create(&stDepthCalTrdId, NULL, TofDepthProcessHandler, NULL);
		pthread_create(&stOutDataSendTrdId, NULL, TofOutDataHandler, NULL);

#if 1
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
					printf("-------------> Demo Exit Start\n");
					goto EXIT;

				case 'c':
					giCaptureFlag = 1;
					printf("---------------------------------------------\n");
					printf("--------------------------> Capture Pic Start\n");
					break;

				default:
					break;
			}
		}
#endif

EXIT:
		pthread_join(stTofRawTrdId, NULL);
		pthread_join(stDepthCalTrdId, NULL);
		pthread_join(stOutDataSendTrdId, NULL);

		i2c3_close();///

		if (!giLocalTestFlag)
		{
			CloseTofCamera();
		}
		else
		{
			TofDepthSdkUnInit();
			I2cCbUninit();
		}
		//isRuning = 1;
	//}

	free(pstTofRawDataCb->stRawData.pRaw);
	printf("-------------> Demo Exit End\n");

	return 0;
}
