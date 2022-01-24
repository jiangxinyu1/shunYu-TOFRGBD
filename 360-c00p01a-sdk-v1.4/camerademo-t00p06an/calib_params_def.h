
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
	/*���˵����
	Detph2Rgb{RGB���䣬����Zֵӳ�䵽rgb�������ϵ�£������µ����ͼ��(Zֵ)}
	Rgb2Depth{IR���䣬RGBӳ�䵽ir�������ϵ��,�����µ�RGBͼ��}
	Rgb2Xyz{rgb�������ϵ�µ�XYZ���Ʋ��䣬RGBӳ�䵽rgb�������ϵ�еĵ���XYZ������rgb����µ�6ͨ������}
	Ir4Rgb{RGB���䣬IRͼ��ӳ�䵽rgb�������ϵ�£������µ�IRͼ��}
	Rgb4Ir{IR���䣬RGBͼ��ӳ�䵽ir�������ϵ�£������µ�RGBͼ��}
	*/
	//RGBD_DEPTH2RGB�����RGBD
	//RGBD_RGB2DEPTH�����ColorDepth

	RGBD_DEPTH2RGB = 0,		//���Detph2Rgb
	RGBD_RGB2DEPTH = 1,     //���Rgb2Depth
	RGBD_BOTH = 2,          //���Detph2Rgb��Rgb2Depth
	RGBD_TRI = 3,           //���Detph2Rgb��Rgb2Depth��Rgb2Xyz
	RGBD_Rect = 4,               //���Detph2Rgb��rebRect
	RGB_DEPTH2RGB_AND_IR2RGB = 5,//���Detph2Rgb��Ir4Rgb
	RGBD_RGB2IR = 6         //���Rgb4Ir
}SRgbdRegType;

#endif