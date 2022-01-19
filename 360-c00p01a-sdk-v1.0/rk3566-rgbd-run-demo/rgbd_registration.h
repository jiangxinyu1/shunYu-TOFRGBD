#ifndef _RGBDREGISTRATION__H__
#define _RGBDREGISTRATION__H__


#include "common_def.h"
#include "calib_params_def.h"



typedef struct RgbdRegistrationInitInputS
{

	int iDepthImgH;
	int iDepthImgW;
	int iRgbImgH;
	int iRgbImgW;

	SRgbdRegType eRegType; // 控制输出的数据的种类
	CameraType eCameraType;// 相机类型选择

	bool bUseRgbDist;      // =true:考虑rgb畸变
	bool bHighPrecision;   // 用于开辟内存判断和调用高低精度函数判断；

	StereoCameraParameters stStereoCameraParameters;	

	float *pfMapMatrix ;   // add 2020/06/04  为了去除内部开辟的内存
	short *sum;            // add 2020/07/30  ir2rgb 提升精度；
	unsigned char  *count; // add 2020/07/30  ir2rgb 提升精度；

}RgbdRegInitInput;


typedef struct RgbdRegistrationProcessInputS
{
	SImage stRgbImg;  
	SImage stIrXYZ;         // add ir坐标系下的点云，单位mm；
	SImage stIr;            // add ir坐标系下的ir image

	SImage stIrDepth;       // add ir坐标系下的深度z值，单位mm

	//RgbdRegistrationInitInputS RgbdRegistrationInitInput; //add

}RgbdRegProcessInput;



typedef struct RgbdRegistrationProcessOutputS
{
	SImage stDepth2Rgb;       //ir坐标系下的深度z值,单通道，ir的分辨率；float
	SImage stIr2Rgb;          //ir的灰度， 单通道， ir分辨率；unsigned char
	SImage stRgb2Depth;       //映射到ir坐标系的rgb图像数据，三通道，ir分辨率； unsigned char


	SImage stColorPointCloud; //彩色点云，6通道，Depth分辨率,RGB坐标系下；float
	SRect stRgbRoi;           // 2020/05/27  人脸小组需求
	
	//
	SImage stRgbDepth_test;   //add by 20200116 用于精度测试

}RgbdRegProcessOutput;


/*
==========================================================================================
*Func Name：RgbdRegistrationInit
*Desc：  入参判断及内存开辟功能；
*Params:
		RgbdRegInitInput *pstRgbdRegInitInput  :入参判断及内存开辟功能；		
*Return: 0-sucess;	-1-fail;
*Author：jinrui
*Time:2020/06/08
============================================================================================
*/
int RgbdRegistrationInit(RgbdRegInitInput *pstRgbdRegInitInput);


/*
==========================================================================================
*Func Name：RgbdRegistrationProcess
*Desc：  根据参数设置，获取想要的结果；
*Params:
		RgbdRegProcessInput *pstRgbdRegProcessInput  :
		RgbdRegProcessOutput *pstRgbdRegProcessOutput   ;
*Return: 0-sucess;	-1-fail;
*Author：jinrui
*Time:2020/06/08
============================================================================================
*/
int RgbdRegistrationProcess(RgbdRegProcessInput *pstRgbdRegProcessInput, RgbdRegProcessOutput *pstRgbdRegProcessOutput);



/*
==========================================================================================
*Func Name：RgbdRegistrationUninit
*Desc：  内存释放；
*Params:
		RgbdRegInitInput *pstRgbdRegInitInput : 内存释放		
*Return: 0-sucess;	-1-fail;
*Author：jinrui
*Time:2020/06/08
============================================================================================
*/
int RgbdRegistrationUninit(RgbdRegInitInput *pstRgbdRegInitInput);

#endif
