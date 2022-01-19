/*
============================================================================
Name        : calib_params_def.h
Author      : linan@sunnyoptical.com
Version     : 2019年6月3日 下午10:18
Copyright   : sunnyoptical.com
Description :
History     :
============================================================================
*/

#ifndef _CALIB_PARAMS_DEF_H_
#define _CALIB_PARAMS_DEF_H_

#include "common_def.h"


#if 0
typedef struct DistCoeffsS  
{
	double k1;
	double k2;
	double k3;
	double p1;
	double p2;
}DistCoeffs;

typedef struct FisheyeDistCoeffsS
{
	double k1;
	double k2;
	double k3;
	double k4;

}FisheyeDistCoeffs;

typedef struct CameraParametersS
{
	SImageSize stImgSize;

	double cx;
	double cy;
	double fx;
	double fy;

	//double rotMatrix[3][3];
	//double transMatrix[3];

	DistCoeffs stDistCoeffs;
	FisheyeDistCoeffs stFisheyeDistCoeffs;

}CameraParameters;


typedef struct StereoCameraParametersS
{
	CameraParameters stFirstCameraParameters;   
	CameraParameters stSecondCameraParameters;   

	double stereoRotMatrix[3][3];
	double stereoTransMatrix[3];

}StereoCameraParameters;

typedef enum CameraTypeE
{
	CommonStereoCamera = 0,
	FisheyeStereoCamera
}CameraType;

typedef enum SRgbDRegistratoinTypeS
{
	/*输出说明：
		detph2Rgb{RGB不变，点云Z值映射到rgb相机坐标系下，生成新的深度图像(Z值)}
		rgb2Depth{IR不变，RGB映射到ir相机坐标系下,生成新的RGB图像}
		rgb2Xyz{rgb相机坐标系下的XYZ点云不变，RGB映射到rgb相机坐标系中的点云XYZ，生成rgb相机下的6通道数据}
		Ir4Rgb{RGB不变，IR图像映射到rgb相机坐标系下，生成新的Ir图像}
	*/
	//RGBD_DEPTH2RGB：输出RGBD
	//RGBD_RGB2DEPTH：输出ColorDepth
	//
	RGBD_DEPTH2RGB = 0,		// 输出detph2Rgb
	RGBD_RGB2DEPTH = 1,
	RGBD_BOTH = 2,          // 输出detph2Rgb、rgb2Depth
	RGBD_TRI = 3,           // 输出detph2Rgb、rgb2Depth、rgb2Xyz
	RGBD_Rect=4,            // 输出detph2Rgb、rebRect
	RGB_DEPTH2RGB_AND_IR2RGB=5//输出detph2Rgb、Ir4Rgb
}SRgbdRegType;

#endif

typedef struct DistCoeffsS
{
	float k1;
	float k2;
	float k3;
	float p1;
	float p2;
}DistCoeffs;

typedef struct FisheyeDistCoeffsS
{
	float k1;
	float k2;
	float k3;
	float k4;

}FisheyeDistCoeffs;

typedef struct CameraParametersS
{
	float cx;
	float cy;
	float fx;
	float fy;

	float rotMatrix[3][3];
	float transMatrix[3];

	DistCoeffs stDistCoeffs;
	FisheyeDistCoeffs stFisheyeDistCoeffs;

}CameraParameters;


typedef struct StereoCameraParametersS
{
	CameraParameters stFirstCameraParameters;
	CameraParameters stSecondCameraParameters;

	float stereoRotMatrix[3][3];
	float stereoTransMatrix[3];

	double errorLeft;
	double errorRight;
	double errorStereo;
}StereoCameraParameters;

typedef enum CameraTypeE
{
	CommonStereoCamera = 0,
	FisheyeStereoCamera
}CameraType;

typedef enum SRgbDRegistratoinTypeS
{
	/*输出说明：
	Detph2Rgb{RGB不变，点云Z值映射到rgb相机坐标系下，生成新的深度图像(Z值)}
	Rgb2Depth{IR不变，RGB映射到ir相机坐标系下,生成新的RGB图像}
	Rgb2Xyz{rgb相机坐标系下的XYZ点云不变，RGB映射到rgb相机坐标系中的点云XYZ，生成rgb相机下的6通道数据}
	Ir4Rgb{RGB不变，IR图像映射到rgb相机坐标系下，生成新的IR图像}
	Rgb4Ir{IR不变，RGB图像映射到ir相机坐标系下，生成新的RGB图像}
	*/
	//RGBD_DEPTH2RGB：输出RGBD
	//RGBD_RGB2DEPTH：输出ColorDepth

	RGBD_DEPTH2RGB = 0,		//输出Detph2Rgb
	RGBD_RGB2DEPTH = 1,     //输出Rgb2Depth
	RGBD_BOTH = 2,          //输出Detph2Rgb、Rgb2Depth
	RGBD_TRI = 3,           //输出Detph2Rgb、Rgb2Depth、Rgb2Xyz
	RGBD_Rect = 4,               //输出Detph2Rgb、rebRect
	RGB_DEPTH2RGB_AND_IR2RGB = 5,//输出Detph2Rgb、Ir4Rgb
	RGBD_RGB2IR = 6         //输出Rgb4Ir
}SRgbdRegType;




#endif