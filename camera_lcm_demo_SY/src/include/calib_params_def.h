
#ifndef _CALIB_PARAMS_DEF_H_
#define _CALIB_PARAMS_DEF_H_

#include "common_def.h"

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