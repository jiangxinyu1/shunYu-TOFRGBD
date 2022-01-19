#include <stdlib.h> 
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <chrono> 
using namespace std;
using namespace chrono; 

#include "rgbd_registration.h"

#define INPUT 
#define OUTPUT
#define INOUTPUT

#define TEST_OPENCV 0

#if TEST_OPENCV
#include <opencv2/opencv.hpp>
#endif

/*
// I420转bgr
void I420ToBGR(unsigned char *src, int width, int height, unsigned char *rgb)
{
	int numOfPixel = width * height;
	int positionOfU = numOfPixel;
	int positionOfV = numOfPixel / 4 + numOfPixel;

	for (int i = 0; i<height; i++) {
		int startY = i*width;
		int step = (i / 2)*(width / 2);
		int startU = positionOfU + step;
		int startV = positionOfV + step;
		for (int j = 0; j < width; j++) {
			int Y = startY + j;
			int U = startU + j / 2;
			int V = startV + j / 2;
			int index = Y * 3;
			rgb[index + 2] = (int)((src[Y] & 0xff) + 1.4075 * ((src[U] & 0xff) - 128));
			rgb[index + 1] = (int)((src[Y] & 0xff) - 0.3455 * ((src[V] & 0xff) - 128) - 0.7169*((src[U] & 0xff) - 128));
			rgb[index + 0] = (int)((src[Y] & 0xff) + 1.779 * ((src[V] & 0xff) - 128));
		}
	}
}*/

// 起始参数配置
int _SettingStartParas(int iIrRows, int iIrCols, int iRgbRows, int iRgbCols,
	RgbdRegInitInput &stRgbdRegInitInput)
{
	int ret = 0;

	stRgbdRegInitInput.eCameraType = FisheyeStereoCamera; //鱼眼标定模型
	stRgbdRegInitInput.eRegType = RGBD_TRI; //三图对齐，参见“calib_params_def.h”
	stRgbdRegInitInput.bHighPrecision = false; // 用于开辟内存判断和调用高低精度函数判断； // false是低精度， true时高精度；
	stRgbdRegInitInput.bUseRgbDist = true;

	stRgbdRegInitInput.iDepthImgH = iIrRows;
	stRgbdRegInitInput.iDepthImgW = iIrCols;
	stRgbdRegInitInput.iRgbImgH = iRgbRows;
	stRgbdRegInitInput.iRgbImgW = iRgbCols;

	//rgbd标定文件
	StereoCameraParameters pstCamera;
	FILE* fp11 = fopen("360/5/5.bin", "rb");
	fread(&pstCamera, sizeof(StereoCameraParameters), 1, fp11);
	fclose(fp11);

	// 标定参数初始化*****************************************************************************************************
	stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.fx = pstCamera.stFirstCameraParameters.fx;//tof相机焦距
	stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.fy = pstCamera.stFirstCameraParameters.fy;
	stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.cx = pstCamera.stFirstCameraParameters.cx;//tof相机主点
	stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.cy = pstCamera.stFirstCameraParameters.cy;
	stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.fx = pstCamera.stSecondCameraParameters.fx;//rgb相机焦距
	stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.fy = pstCamera.stSecondCameraParameters.fy;
	stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.cx = pstCamera.stSecondCameraParameters.cx;//rgb相机主点
	stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.cy = pstCamera.stSecondCameraParameters.cy;
	if (CommonStereoCamera == stRgbdRegInitInput.eCameraType)//畸变系数初始化
	{
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stDistCoeffs.k1 = pstCamera.stFirstCameraParameters.stDistCoeffs.k1;
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stDistCoeffs.k2 = pstCamera.stFirstCameraParameters.stDistCoeffs.k2;
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stDistCoeffs.p1 = pstCamera.stFirstCameraParameters.stDistCoeffs.p1;
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stDistCoeffs.p2 = pstCamera.stFirstCameraParameters.stDistCoeffs.p2;
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stDistCoeffs.k3 = pstCamera.stFirstCameraParameters.stDistCoeffs.k3;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stDistCoeffs.k1 = pstCamera.stSecondCameraParameters.stDistCoeffs.k1;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stDistCoeffs.k2 = pstCamera.stSecondCameraParameters.stDistCoeffs.k2;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stDistCoeffs.p1 = pstCamera.stSecondCameraParameters.stDistCoeffs.p1;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stDistCoeffs.p2 = pstCamera.stSecondCameraParameters.stDistCoeffs.p2;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stDistCoeffs.k3 = pstCamera.stSecondCameraParameters.stDistCoeffs.k3;
	}
	else
	{
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stFisheyeDistCoeffs.k1 = pstCamera.stFirstCameraParameters.stFisheyeDistCoeffs.k1;
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stFisheyeDistCoeffs.k2 = pstCamera.stFirstCameraParameters.stFisheyeDistCoeffs.k2;
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stFisheyeDistCoeffs.k3 = pstCamera.stFirstCameraParameters.stFisheyeDistCoeffs.k3;
		stRgbdRegInitInput.stStereoCameraParameters.stFirstCameraParameters.stFisheyeDistCoeffs.k4 = pstCamera.stFirstCameraParameters.stFisheyeDistCoeffs.k4;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stFisheyeDistCoeffs.k1 = pstCamera.stSecondCameraParameters.stFisheyeDistCoeffs.k1;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stFisheyeDistCoeffs.k2 = pstCamera.stSecondCameraParameters.stFisheyeDistCoeffs.k2;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stFisheyeDistCoeffs.k3 = pstCamera.stSecondCameraParameters.stFisheyeDistCoeffs.k3;
		stRgbdRegInitInput.stStereoCameraParameters.stSecondCameraParameters.stFisheyeDistCoeffs.k4 = pstCamera.stSecondCameraParameters.stFisheyeDistCoeffs.k4;
	}
	//旋转矩阵初始化
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[0][0] = pstCamera.stereoRotMatrix[0][0];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[0][1] = pstCamera.stereoRotMatrix[0][1];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[0][2] = pstCamera.stereoRotMatrix[0][2];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[1][0] = pstCamera.stereoRotMatrix[1][0];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[1][1] = pstCamera.stereoRotMatrix[1][1];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[1][2] = pstCamera.stereoRotMatrix[1][2];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[2][0] = pstCamera.stereoRotMatrix[2][0];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[2][1] = pstCamera.stereoRotMatrix[2][1];
	stRgbdRegInitInput.stStereoCameraParameters.stereoRotMatrix[2][2] = pstCamera.stereoRotMatrix[2][2];
	stRgbdRegInitInput.stStereoCameraParameters.stereoTransMatrix[0] = pstCamera.stereoTransMatrix[0];
	stRgbdRegInitInput.stStereoCameraParameters.stereoTransMatrix[1] = pstCamera.stereoTransMatrix[1];
	stRgbdRegInitInput.stStereoCameraParameters.stereoTransMatrix[2] = pstCamera.stereoTransMatrix[2];

	return ret;
}

// InputData保存在结构体中
int _GetInputData(int iIrRows, int iIrCols, int iRgbRows, int iRgbCols,
	float *irFloaltData, unsigned char *charDeRgbData, unsigned char *charDeIrData,
	RgbdRegInitInput &stRgbdRegInitInput,
	RgbdRegProcessInput &stRgbdRegProcessInput)
{

	// 标定参数 入参判断
	INOUTPUT int ret = RgbdRegistrationInit(&stRgbdRegInitInput);
	if (0 != ret)
	{
		return -1;
	}

	// 将 irXYZ 保存在结构体中  ******************************************************************
	stRgbdRegProcessInput.stIrXYZ.iImgH = iIrRows;
	stRgbdRegProcessInput.stIrXYZ.iImgW = iIrCols;
	stRgbdRegProcessInput.stIrXYZ.iChannels = 3;
	unsigned int iIrXyzMemSize = sizeof(float) * iIrRows * iIrCols * 3;
	stRgbdRegProcessInput.stIrXYZ.imgData = (void*)malloc(iIrXyzMemSize);
	if (!stRgbdRegProcessInput.stIrXYZ.imgData)
	{
		return -1;
	}
	memcpy(stRgbdRegProcessInput.stIrXYZ.imgData, irFloaltData, iIrXyzMemSize);

	// 将 RgbImg 保存在结构体中  ******************************************************************
	stRgbdRegProcessInput.stRgbImg.iImgH = iRgbRows;
	stRgbdRegProcessInput.stRgbImg.iImgW = iRgbCols;
	stRgbdRegProcessInput.stRgbImg.iChannels = 3;
	unsigned int rgbImgMemSize = sizeof(unsigned char)*iRgbRows*iRgbCols * 3;
	stRgbdRegProcessInput.stRgbImg.imgData = (void*)malloc(rgbImgMemSize);
	if (!stRgbdRegProcessInput.stRgbImg.imgData)
	{

		if (stRgbdRegProcessInput.stIrXYZ.imgData)
		{
			free(stRgbdRegProcessInput.stIrXYZ.imgData);
			stRgbdRegProcessInput.stIrXYZ.imgData = NULL;
		}

		return -1;
	}
	memcpy(stRgbdRegProcessInput.stRgbImg.imgData, charDeRgbData, rgbImgMemSize);

	// 将 ir 保存在结构体中  ******************************************************************
	{
		if (RGBD_TRI == stRgbdRegInitInput.eRegType)
		{
			stRgbdRegProcessInput.stIr.iImgH = iIrRows;
			stRgbdRegProcessInput.stIr.iImgW = iIrCols;
			stRgbdRegProcessInput.stIr.iChannels = 1;
			unsigned int irImgMemSize = sizeof(unsigned char)*iIrRows*iIrCols * 1;
			stRgbdRegProcessInput.stIr.imgData = (void*)malloc(irImgMemSize);
			if (!stRgbdRegProcessInput.stIr.imgData)
			{

				if (stRgbdRegProcessInput.stIrXYZ.imgData)
				{
					free(stRgbdRegProcessInput.stIrXYZ.imgData);
					stRgbdRegProcessInput.stIrXYZ.imgData = NULL;
				}
				if (stRgbdRegProcessInput.stRgbImg.imgData)
				{
					free(stRgbdRegProcessInput.stRgbImg.imgData);
					stRgbdRegProcessInput.stRgbImg.imgData = NULL;
				}
				return -1;
			}

			memcpy(stRgbdRegProcessInput.stIr.imgData, charDeIrData, irImgMemSize);
		}
	}

	return ret;
}

// CreateOutputBuf ,Output中的变量开辟内存
int _CreateOutputBuf(int iIrRows, int iIrCols, int iRgbRows, int iRgbCols,
	RgbdRegInitInput &stRgbdRegInitInput,
	RgbdRegProcessInput &stRgbdRegProcessInput,
	RgbdRegProcessOutput &stRgbdRegistrationOutput)
{
	int ret = 0;

	if (RGBD_TRI == stRgbdRegInitInput.eRegType)
	{
		// 开辟 Ir2Rgb 的内存，并初始化为0；  ******************************************************************		
		stRgbdRegistrationOutput.stIr2Rgb.imgData = (void*)malloc(sizeof(unsigned char)*iIrRows*iIrCols);
		memset(stRgbdRegistrationOutput.stIr2Rgb.imgData, 0, sizeof(unsigned char)*iIrRows*iIrCols);
		if (!stRgbdRegistrationOutput.stIr2Rgb.imgData)
		{
			if (stRgbdRegProcessInput.stIr.imgData)
			{
				free(stRgbdRegProcessInput.stIr.imgData);
				stRgbdRegProcessInput.stIr.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stIrXYZ.imgData)
			{
				free(stRgbdRegProcessInput.stIrXYZ.imgData);
				stRgbdRegProcessInput.stIrXYZ.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stRgbImg.imgData)
			{
				free(stRgbdRegProcessInput.stRgbImg.imgData);
				stRgbdRegProcessInput.stRgbImg.imgData = NULL;
			}
			return -1;
		}

		// 开辟 Depth2Rgb 的内存，并初始化为0；  ******************************************************************		
		stRgbdRegistrationOutput.stDepth2Rgb.imgData = (void*)malloc(sizeof(float)*iIrRows*iIrCols * 3);
		memset(stRgbdRegistrationOutput.stDepth2Rgb.imgData, 0, sizeof(float)*iIrRows*iIrCols * 3); 
		if (!stRgbdRegistrationOutput.stDepth2Rgb.imgData)
		{
			if (stRgbdRegProcessInput.stIr.imgData)
			{
				free(stRgbdRegProcessInput.stIr.imgData);
				stRgbdRegProcessInput.stIr.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stIrXYZ.imgData)
			{
				free(stRgbdRegProcessInput.stIrXYZ.imgData);
				stRgbdRegProcessInput.stIrXYZ.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stRgbImg.imgData)
			{
				free(stRgbdRegProcessInput.stRgbImg.imgData);
				stRgbdRegProcessInput.stRgbImg.imgData = NULL;
			}

			if (stRgbdRegistrationOutput.stIr2Rgb.imgData)
			{
				free(stRgbdRegistrationOutput.stIr2Rgb.imgData);
				stRgbdRegistrationOutput.stIr2Rgb.imgData = NULL;
			}

			return -1;
		}

		// 开辟 Rgb2Depth 的内存，并初始化为0；  ***********************************************************
		stRgbdRegistrationOutput.stRgb2Depth.imgData = (void*)malloc(sizeof(unsigned char)*iRgbRows*iRgbCols * 3);
		memset(stRgbdRegistrationOutput.stRgb2Depth.imgData, 0, sizeof(unsigned char)*iRgbRows*iRgbCols * 3);
		if (!stRgbdRegistrationOutput.stRgb2Depth.imgData)
		{
			if (stRgbdRegProcessInput.stIr.imgData)
			{
				free(stRgbdRegProcessInput.stIr.imgData);
				stRgbdRegProcessInput.stIr.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stIrXYZ.imgData)
			{
				free(stRgbdRegProcessInput.stIrXYZ.imgData);
				stRgbdRegProcessInput.stIrXYZ.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stRgbImg.imgData)
			{
				free(stRgbdRegProcessInput.stRgbImg.imgData);
				stRgbdRegProcessInput.stRgbImg.imgData = NULL;
			}
			
			if (stRgbdRegistrationOutput.stIr2Rgb.imgData)
			{
				free(stRgbdRegistrationOutput.stIr2Rgb.imgData);
				stRgbdRegistrationOutput.stIr2Rgb.imgData = NULL;
			}

			if (stRgbdRegistrationOutput.stDepth2Rgb.imgData)
			{
				free(stRgbdRegistrationOutput.stDepth2Rgb.imgData);
				stRgbdRegistrationOutput.stDepth2Rgb.imgData = NULL;
			}

			return -1;

		}

		// 开辟 Colored_PointCloud 的内存，并初始化为0；  *******************************************************
		stRgbdRegistrationOutput.stColorPointCloud.imgData = (void*)malloc(sizeof(float)*iIrRows*iIrCols * 6);
		memset(stRgbdRegistrationOutput.stColorPointCloud.imgData, 0, sizeof(float)*iIrRows*iIrCols * 6);
		if (!stRgbdRegistrationOutput.stColorPointCloud.imgData)
		{
			if (stRgbdRegProcessInput.stIr.imgData)
			{
				free(stRgbdRegProcessInput.stIr.imgData);
				stRgbdRegProcessInput.stIr.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stIrXYZ.imgData)
			{
				free(stRgbdRegProcessInput.stIrXYZ.imgData);
				stRgbdRegProcessInput.stIrXYZ.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stRgbImg.imgData)
			{
				free(stRgbdRegProcessInput.stRgbImg.imgData);
				stRgbdRegProcessInput.stRgbImg.imgData = NULL;
			}

			if (stRgbdRegistrationOutput.stIr2Rgb.imgData)
			{
				free(stRgbdRegistrationOutput.stIr2Rgb.imgData);
				stRgbdRegistrationOutput.stIr2Rgb.imgData = NULL;
			}

			if (stRgbdRegistrationOutput.stDepth2Rgb.imgData)
			{
				free(stRgbdRegistrationOutput.stDepth2Rgb.imgData);
				stRgbdRegistrationOutput.stDepth2Rgb.imgData = NULL;
			}

			if (stRgbdRegistrationOutput.stRgb2Depth.imgData)
			{
				free(stRgbdRegistrationOutput.stRgb2Depth.imgData);
				stRgbdRegistrationOutput.stRgb2Depth.imgData = NULL;
			}
			return -1;
		}
	}

	return ret;
}

//   InitRgbdReg ( )
//         a、GetInputData保存在结构体中；
//         b、CreateOutputBuf中的变量开辟内存；
int  _InitRgbdReg(int iIrRows, int iIrCols, int iRgbRows, int iRgbCols,
	float *irFloaltData, unsigned char *charDeRgbData, unsigned char *charDeIrData,
	RgbdRegInitInput &stRgbdRegInitInput,
	RgbdRegProcessInput &stRgbdRegProcessInput,
	RgbdRegProcessOutput &stRgbdRegistrationOutput)
{
	//InputData保存在结构体中；
	int ret = _GetInputData(iIrRows, iIrCols, iRgbRows, iRgbCols,
		irFloaltData, charDeRgbData, charDeIrData,
		stRgbdRegInitInput, stRgbdRegProcessInput);

	// CreateOutputBuf ,Output中的变量开辟内存；
	if (0 == ret)
	{
		ret = _CreateOutputBuf(iIrRows, iIrCols, iRgbRows, iRgbCols,
			stRgbdRegInitInput,
			stRgbdRegProcessInput,
			stRgbdRegistrationOutput);
	}

	return ret;
}






// 保存输出数据
int _SaveDataAndFreeBuf(std::string commonFileName,
	int iIrRows, int iIrCols, int iRgbRows, int iRgbCols,
	RgbdRegInitInput &stRgbdRegInitInput,
	RgbdRegProcessInput &stRgbdRegProcessInput,
	RgbdRegProcessOutput &stRgbdRegistrationOutput)
{
	int ret = 0;

	if (RGBD_TRI == stRgbdRegInitInput.eRegType)
	{
		//保存配准后的rgb图
		/*cv::Mat Rgb2DepthMat(iIrRows, iIrCols, CV_8UC3, stRgbdRegistrationOutput.stRgb2Depth.imgData);
		cv::Mat Rgb2DepthMatClone = Rgb2DepthMat.clone();
		cv::normalize(Rgb2DepthMat, Rgb2DepthMatClone, 0, 255, cv::NORM_MINMAX);
		cv::imwrite(commonFileName + "rgb2ir.png", Rgb2DepthMatClone);

		//保存配准后的ir图
		cv::Mat Ir2RgbMat(iIrRows, iIrCols, CV_8UC1, stRgbdRegistrationOutput.stIr2Rgb.imgData);
		cv::Mat Ir2RgbMatClone = Ir2RgbMat.clone();
		cv::normalize(Ir2RgbMat, Ir2RgbMatClone, 0, 255, cv::NORM_MINMAX);
		cv::imwrite(commonFileName + "ir2rgb.png", Ir2RgbMatClone);*/

		FILE *fp_rgb2ir = NULL;
		fp_rgb2ir = fopen("360/5/rgb2ir.dat","wb");
		fwrite(stRgbdRegistrationOutput.stRgb2Depth.imgData, iIrRows*iIrCols*1*3, 1, fp_rgb2ir);
		fclose(fp_rgb2ir);


		FILE *fp_ir2rgb = NULL;
		fp_ir2rgb = fopen("360/5/ir2rgb.dat","wb");
		fwrite(stRgbdRegistrationOutput.stIr2Rgb.imgData, iIrRows*iIrCols, 1, fp_ir2rgb);
		fclose(fp_ir2rgb);
		

		//保存配准后的点云
		float *Depth2RgbData = (float*)stRgbdRegistrationOutput.stDepth2Rgb.imgData;
		std::ofstream fout_xyz(commonFileName + "depth2rgb.txt", std::ios::out);
		for (int i = 0; i < iIrRows; i++)
		{
			for (int j = 0; j < iIrCols; j++)
			{
				fout_xyz << Depth2RgbData[(i * iIrCols + j) * 3 + 0] / 1000.0 << ";" << Depth2RgbData[(i * iIrCols + j) * 3 + 1] / 1000.0 << ";" << Depth2RgbData[(i * iIrCols + j) * 3 + 2] / 1000.0 << std::endl;
			}		
		}
		fout_xyz.close();

		//释放内存
		{
			if (stRgbdRegProcessInput.stIr.imgData)
			{
				free(stRgbdRegProcessInput.stIr.imgData);
				stRgbdRegProcessInput.stIr.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stIrXYZ.imgData)
			{
				free(stRgbdRegProcessInput.stIrXYZ.imgData);
				stRgbdRegProcessInput.stIrXYZ.imgData = NULL;
			}

			if (stRgbdRegProcessInput.stRgbImg.imgData)
			{
				free(stRgbdRegProcessInput.stRgbImg.imgData);
				stRgbdRegProcessInput.stRgbImg.imgData = NULL;
			}
			//
			if (stRgbdRegistrationOutput.stRgb2Depth.imgData)
			{
				free(stRgbdRegistrationOutput.stRgb2Depth.imgData);
				stRgbdRegistrationOutput.stRgb2Depth.imgData = NULL;
			}
			if (stRgbdRegistrationOutput.stIr2Rgb.imgData)
			{
				free(stRgbdRegistrationOutput.stIr2Rgb.imgData);
				stRgbdRegistrationOutput.stIr2Rgb.imgData = NULL;
			}
			if (stRgbdRegistrationOutput.stDepth2Rgb.imgData)
			{
				free(stRgbdRegistrationOutput.stDepth2Rgb.imgData);
				stRgbdRegistrationOutput.stDepth2Rgb.imgData = NULL;
			}

			if (stRgbdRegistrationOutput.stColorPointCloud.imgData)
			{
				free(stRgbdRegistrationOutput.stColorPointCloud.imgData);
				stRgbdRegistrationOutput.stColorPointCloud.imgData = NULL;
			}
		} //释放内存 end
	} //RGBD_TRI

	return 0;
}

/*
1、 StartParasSetting() 起始参数配置： bool ，enum, CamParas
2、 InitRgbdReg(),
       a、GetInputData保存在结构体中；
       b、CreateOutputBuf中的变量开辟内存；
3、 RgbdRegistrationProcess（ ）
4、 RgbdRegistrationUninit（ ）
5、 SaveDataAndFreeBuf();
*/

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


bool CGrayConvert(float* pGray, const int width, const int height, unsigned char* pU8)
{
	const int pixel_cnt = width*height;
	//const float min = Utils_FindMinValue(pGray, pixel_cnt);//*min_element(pGray, pGray + pixel_cnt);//min_element耗时长
	const float max = Utils_FindMaxValue(pGray, pixel_cnt);// *max_element(pGray, pGray + pixel_cnt);//max_element耗时长

	if (0.001 >= max)//0值用黑色表示
	{
		memset(pU8, 0, pixel_cnt * sizeof(pU8[0]));
		return true;
	}

	const float K = (255 * 1.0 / max);//最大值是255的多少倍

	for (int i = 0; i < pixel_cnt; i++)
	{
		unsigned char tmp = 0;//0值用黑色表示
		if (0.001 < pGray[i])
		{
			tmp = (unsigned char)(pGray[i] * K);
		}
		pU8[i] = tmp;
	}

	return true;
}


int test_360()
{
	int iIrRows = 114;
	int iIrCols = 224;
	int iRgbRows = 1080;
	int iRgbCols = 1920;

	// 共同目录
	std::string commonFileName = "360/5/";
	
#if 0
	// 读取rgb
	cv::Mat rgbImg = cv::imread("360/5/5.bmp", 1);
	// bgr转yuv
	int buflen = (int)(iRgbRows * iRgbCols * 3 / 2);
	unsigned char* pYuvBuf = new unsigned char[buflen];
	cv::Mat OpencvYUV, OpencvRGB;
	cv::cvtColor(rgbImg, OpencvYUV, cv::COLOR_BGR2YUV_I420);
	cv::cvtColor(OpencvYUV, OpencvRGB, cv::COLOR_YUV2BGR_I420);
	memcpy(pYuvBuf, OpencvYUV.data, buflen * sizeof(unsigned char));
	// yuv转bgr
	unsigned char *rgbData = (unsigned char*)malloc(sizeof(unsigned char) * iRgbRows * iRgbCols * 3);
	I420ToBGR(pYuvBuf, iRgbCols, iRgbRows, rgbData);
	cv::Mat temprgbImg;
	temprgbImg.create(iRgbRows, iRgbCols, CV_8UC3);
	memcpy(temprgbImg.data, rgbData, sizeof(unsigned char) * iRgbRows * iRgbCols * 3);

#endif


	FILE *fprgb = NULL;
	fprgb = fopen("360/5/5-bmp-cut-header", "rb");
	unsigned char *rgbData = (unsigned char*)malloc(sizeof(unsigned char) * iRgbRows * iRgbCols * 3);
	fread(rgbData, 1, iRgbRows * iRgbCols * 3, fprgb);
	fclose(fprgb);


	// 读取Ir  
	std::string irFileName = commonFileName + "5.raw";
	int iIrSize = iIrRows * iIrCols;
	unsigned char *irData = (unsigned char*)malloc(sizeof(float)*iIrSize);
	FILE *fp2 = fopen(irFileName.c_str(), "rb");
	fread(irData, iIrSize, sizeof(float), fp2);
	fclose(fp2);

	unsigned char *irDataU8 = (unsigned char*)malloc(sizeof(unsigned char)*iIrSize);
	CGrayConvert((float *)irData,224,114,irDataU8);

	#if 0
	cv::Mat tempMat, irMat;
	tempMat.create(114, 224, CV_32FC1);
	memcpy(tempMat.data, irData, iIrRows * iIrCols * sizeof(float));
	tempMat.convertTo(irMat, CV_8UC1); //float转uint8
	memcpy(irData, irMat.data, iIrRows * iIrCols * sizeof(unsigned char));
	#endif

	// 读取irXYZ 
	std::string XYZ_Name = commonFileName + "5.txt";
	float *pIrXYZ0 = new float[iIrRows * iIrCols * 3];
	std::ifstream test(XYZ_Name.data(), std::ios::in);
	float x, y, z;
	char char_1, char_2; // .xyz 文件 中数据内有 ；和间隔
	// 对于 .xyz 文件 的读取; 数据内有；和间隔
	for (int i = 0; i < iIrRows*iIrCols; i++)
	{
		test >> x >> char_1 >> y >> char_2 >> z;
		pIrXYZ0[3 * i + 0] = x * 1000;  //m转为mm
		pIrXYZ0[3 * i + 1] = y * 1000;
		pIrXYZ0[3 * i + 2] = z * 1000;
	}

	std::cout << "test_360_c: --" << std::endl;

	RgbdRegInitInput stRgbdRegInitInput;
	RgbdRegProcessInput stRgbdRegProcessInput;
	RgbdRegProcessOutput stRgbdRegistrationOutput;

	INOUTPUT int ret = _SettingStartParas(iIrRows, iIrCols, iRgbRows, iRgbCols, stRgbdRegInitInput);

	INOUTPUT _InitRgbdReg(iIrRows, iIrCols, iRgbRows, iRgbCols,
		pIrXYZ0, rgbData, irDataU8,
							stRgbdRegInitInput,
							stRgbdRegProcessInput,
							stRgbdRegistrationOutput);

	// 计时
	auto start = system_clock::now();
	int countNUM = 1;
	for (int i = 0; i < countNUM; i++)
	{
		INOUTPUT ret = RgbdRegistrationProcess(&stRgbdRegProcessInput, &stRgbdRegistrationOutput);
	}
	auto end = system_clock::now(); 
	auto duration = duration_cast<microseconds>(end - start);
	double timeCost = double(duration.count()) * microseconds::period::num / microseconds::period::den;
	cout << "RgbdRegistrationProcess:  " << 1000 * timeCost / countNUM << "  ms" << std::endl;
		
	INOUTPUT ret = RgbdRegistrationUninit(&stRgbdRegInitInput);

	INOUTPUT _SaveDataAndFreeBuf(commonFileName,
									iIrRows, iIrCols, iRgbRows, iRgbCols,
									stRgbdRegInitInput,
									stRgbdRegProcessInput,
									stRgbdRegistrationOutput);

	free(rgbData);
	free(irData);
	free(irDataU8);
	free(pIrXYZ0);
	

	return 0;
}
