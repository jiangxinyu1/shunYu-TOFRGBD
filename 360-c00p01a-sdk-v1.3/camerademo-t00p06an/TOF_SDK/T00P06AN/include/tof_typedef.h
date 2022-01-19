#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__


typedef unsigned int       UINT32;
typedef unsigned short     UINT16;
typedef unsigned char	   UINT8;
typedef unsigned long long UINT64;

typedef signed int    	   SINT32;
typedef signed short  	   SINT16;
typedef signed char		   SINT8;
typedef signed long long   SINT64;

typedef float              FLOAT32;
typedef double 		       FLOAT64;
typedef bool			   SBOOL;
typedef char			   SCHAR;	


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef NULL
    #define NULL 0
#endif


#define MAKE_UNIQUE_ID(major, sub, a, b) ((major<<24) | (sub<<16) | (a<<8) | (b))


typedef enum tagTOF_MODE
{
	//˫Ƶ
	TOF_MODE_STERO_5FPS  = 0x00000001,
	TOF_MODE_STERO_10FPS = 0x00000002,
	TOF_MODE_STERO_15FPS = 0x00000004,
	TOF_MODE_STERO_30FPS = 0x00000008,
	TOF_MODE_STERO_45FPS = 0x00000010,
	TOF_MODE_STERO_60FPS = 0x00000020,

	//��Ƶ
	TOF_MODE_MONO_5FPS   = 0x00000040,
	TOF_MODE_MONO_10FPS  = 0x00000080,
	TOF_MODE_MONO_15FPS  = 0x00000100,
	TOF_MODE_MONO_30FPS  = 0x00000200,
	TOF_MODE_MONO_45FPS  = 0x00000400,
	TOF_MODE_MONO_60FPS  = 0x00000800,

	//HDRZ���⼸��ģʽ�������raw���ݵ�HDRZ�ںϵ�
	TOF_MODE_HDRZ_5FPS   = 0x00001000,
	TOF_MODE_HDRZ_10FPS  = 0x00002000,
	TOF_MODE_HDRZ_15FPS  = 0x00004000,
	TOF_MODE_HDRZ_30FPS  = 0x00008000,
	TOF_MODE_HDRZ_45FPS  = 0x00010000,
	TOF_MODE_HDRZ_60FPS  = 0x00020000,

	//һ��Ƶ
	TOF_MODE_5FPS        = 0x00040000,
	TOF_MODE_10FPS       = 0x00080000,
	TOF_MODE_20FPS       = 0x00100000,
	TOF_MODE_30FPS       = 0x00200000,
	TOF_MODE_45FPS       = 0x00400000,
	TOF_MODE_60FPS       = 0x00800000,

	//���ƴ���
	TOF_MODE_ADI_1M5     = 0x01000000,
	TOF_MODE_ADI_5M      = 0x02000000,


}TOF_MODE;


typedef enum tagTOF_FILTER
{
	TOF_FILTER_RemoveFlyingPixel   = 0x00000001,
	TOF_FILTER_AdaptiveNoiseFilter = 0x00000002,
	TOF_FILTER_InterFrameFilter    = 0x00000004,
	TOF_FILTER_PointCloudFilter    = 0x00000008,
	TOF_FILTER_StraylightFilter    = 0x00000010,
	TOF_FILTER_CalcIntensities     = 0x00000020,
	TOF_FILTER_MPIFlagAverage      = 0x00000040,
	TOF_FILTER_MPIFlagAmplitude    = 0x00000080,
	TOF_FILTER_MPIFlagDistance     = 0x00000100,
	TOF_FILTER_ValidateImage       = 0x00000200,
	TOF_FILTER_SparsePointCloud    = 0x00000400,
	TOF_FILTER_Average             = 0x00000800,
	TOF_FILTER_Median              = 0x00001000,
	TOF_FILTER_Confidence          = 0x00002000,
	TOF_FILTER_MPIFilter           = 0x00004000,
	TOF_FILTER_PointCloudCorrect   = 0x00008000,
	TOF_FILTER_LineRecognition     = 0x00010000,

}TOF_FILTER;


typedef struct tagTofFilterCfg_RemoveFlyingPixel
{
	FLOAT32 f0;
	FLOAT32 f1;
	FLOAT32 nd;
	FLOAT32 fd;
}TofFilterCfg_RemoveFlyingPixel;

typedef struct tagTofFilterCfg_AdaptiveNoiseFilter
{
	SINT32  k;
	FLOAT32 s;
	SINT32  t;
}TofFilterCfg_AdaptiveNoiseFilter;

typedef struct tagTofFilterCfg_InterFrameFilter
{
	FLOAT32 mdg;
	FLOAT32 mdt;
	FLOAT32 fg1;
	FLOAT32 fg2;
}TofFilterCfg_InterFrameFilter;

typedef struct tagTofFilterCfg_PointCloudFilter
{
	SINT32 k;
}TofFilterCfg_PointCloudFilter;

typedef struct tagTofFilterCfg_StraylightFilter
{
	FLOAT32 d[16];
	FLOAT32 t[16];
}TofFilterCfg_StraylightFilter;

typedef struct tagTofFilterCfg_CalcIntensities
{
	UINT8 szRes[4];//Ԥ��,4�ֽڶ���
}TofFilterCfg_CalcIntensities;

typedef struct tagTofFilterCfg_MPIFlagAverage
{
	UINT8 szRes[4];//Ԥ��,4�ֽڶ���
}TofFilterCfg_MPIFlagAverage;

typedef struct tagTofFilterCfg_MPIFlagAmplitude
{
	FLOAT32 mat;
	FLOAT32 ndt;
}TofFilterCfg_MPIFlagAmplitude;

typedef struct tagTofFilterCfg_MPIFlagDistance
{
	UINT8 szRes[4];//Ԥ��,4�ֽڶ���
}TofFilterCfg_MPIFlagDistance;

typedef struct tagTofFilterCfg_ValidateImage
{
	UINT8 szRes[4];//Ԥ��,4�ֽڶ���
}TofFilterCfg_ValidateImage;

typedef struct tagTofFilterCfg_SparsePointCloud
{
	UINT8 szRes[4];//Ԥ��,4�ֽڶ���
}TofFilterCfg_SparsePointCloud;

typedef struct tagTofFilterCfg_Average
{
	UINT8 szRes[4];//Ԥ��,4�ֽڶ���
}TofFilterCfg_Average;

typedef struct tagTofFilterCfg_Median
{
	UINT8 szRes[4];//Ԥ��,4�ֽڶ���
}TofFilterCfg_Median;

typedef struct tagTofFilterCfg_Confidence
{
	FLOAT32 t;
}TofFilterCfg_Confidence;

typedef struct tagTofFilterCfg_MPIFilter
{
	FLOAT32 ndt;
	FLOAT32 fdt;
	FLOAT32 nnr;
	FLOAT32 mnr;
	FLOAT32 fnr;
	FLOAT32 rd;
}TofFilterCfg_MPIFilter;

typedef struct tagTofFilterCfg_PointCloudCorrect
{
	FLOAT32 da;//ģ����б�ĽǶ�
	FLOAT32 tgd;//ģ������ĸ߶�
	FLOAT32 t1;
	FLOAT32 t2;
}TofFilterCfg_PointCloudCorrect;

typedef struct tagTofFilterCfg_LineRecognition
{
	SINT32 ht;
	SINT32 cgt;
	SINT32 fgst;
	FLOAT32 gstr;
	SINT32 spgt;
	SINT32 opgt;
	SINT32 rc;
	SINT32 lc;
	SINT32 ma;
}TofFilterCfg_LineRecognition;


typedef struct tagTofFilterCfg
{
	TOF_FILTER type;//ĳһ���˲����ͣ�һ�������������ֻ����

	union
	{
		SBOOL bEnable;//�Ƿ����ã���������������������(��ʱ������,Ŀǰ������Ч�ֶ�)
		UINT8 szRes[4];//Ԥ��,4�ֽڶ���
	}uRes;

	union
	{
		TofFilterCfg_RemoveFlyingPixel struRemoveFlyingPixel;//��typeȡֵΪTOF_FILTER_RemoveFlyingPixelʱ��Ч
		TofFilterCfg_AdaptiveNoiseFilter struAdaptiveNoiseFilter;//��typeȡֵΪTOF_FILTER_AdaptiveNoiseFilterʱ��Ч
		TofFilterCfg_InterFrameFilter struInterFrameFilter;//��typeȡֵΪTOF_FILTER_InterFrameFilterʱ��Ч
		TofFilterCfg_PointCloudFilter struPointCloudFilter;//��typeȡֵΪTOF_FILTER_PointCloudFilterʱ��Ч
		TofFilterCfg_StraylightFilter struStraylightFilter;//��typeȡֵΪTOF_FILTER_StraylightFilterʱ��Ч
		TofFilterCfg_CalcIntensities struCalcIntensities;//��typeȡֵΪTOF_FILTER_CalcIntensitiesʱ��Ч
		TofFilterCfg_MPIFlagAverage struMPIFlagAverage;//��typeȡֵΪTOF_FILTER_MPIFlagAverageʱ��Ч
		TofFilterCfg_MPIFlagAmplitude struMPIFlagAmplitude;//��typeȡֵΪTOF_FILTER_MPIFlagAmplitudeʱ��Ч
		TofFilterCfg_MPIFlagDistance struMPIFlagDistance;//��typeȡֵΪTOF_FILTER_MPIFlagDistanceʱ��Ч
		TofFilterCfg_ValidateImage struValidateImage;//��typeȡֵΪTOF_FILTER_ValidateImageʱ��Ч
		TofFilterCfg_SparsePointCloud struSparsePointCloud;//��typeȡֵΪTOF_FILTER_SparsePointCloudʱ��Ч
		TofFilterCfg_Average struAverage;//��typeȡֵΪTOF_FILTER_Averageʱ��Ч
		TofFilterCfg_Median struMedian;//��typeȡֵΪTOF_FILTER_Medianʱ��Ч
		TofFilterCfg_Confidence struConfidence;//��typeȡֵΪTOF_FILTER_Confidenceʱ��Ч
		TofFilterCfg_MPIFilter struMPIFilter;//��typeȡֵΪTOF_FILTER_MPIFilterʱ��Ч
		TofFilterCfg_PointCloudCorrect struPointCloudCorrect;//��typeȡֵΪTOF_FILTER_PointCloudCorrectʱ��Ч
		TofFilterCfg_LineRecognition struLineRecognition;//��typeȡֵΪTOF_FILTER_LineRecognitionʱ��Ч
	}uCfg;//ĳ���˲��ľ������ã���������������������

}TofFilterCfg;

typedef enum tagEXP_MODE
{
	EXP_MODE_MANUAL = 0x00000001,//�ֶ��ع�
	EXP_MODE_AUTO   = 0x00000002,//�Զ��ع�(AE)
}EXP_MODE;



//�Ҷ����ݸ�ʽ
typedef enum tagGRAY_FORMAT
{
	GRAY_FORMAT_UINT8  = 0,//8λ����
	GRAY_FORMAT_UINT16,//�޷���16λ����
	GRAY_FORMAT_FLOAT,//����������
	GRAY_FORMAT_BGRD,//ÿ����32λ�� ��B/G/R/D˳����

}GRAY_FORMAT;


typedef struct tagPointData
{
	FLOAT32 x;
	FLOAT32 y;
	FLOAT32 z;
}PointData;


//TOF Expouse
typedef struct tagTofExpouse
{
	UINT32  	nCurrent;//��ǰֵ���ɶ�д
	UINT32  	nDefault;//Ĭ��ֵ��ֻ��
	UINT32  	nStep;//����ֵ��ֻ��
	UINT32  	nMax;//���ֵ��ֻ��
	UINT32  	nMin;//��Сֵ��ֻ��
}TofExpouse;


//TOF Expouse Group1
typedef struct tagTofExpouseGroup1
{
	TofExpouse exp;//�ع����
}TofExpouseGroup1;


//TOF Expouse Group2
typedef struct tagTofExpouseGroup2
{
	TofExpouse exp_AEF;//�Զ��ع�֡�ع����
	TofExpouse exp_FEF;//�̶��ع�֡�ع����
}TofExpouseGroup2;

//TOF Expouse Items
typedef struct tagTofExpouseItems
{
	UINT32 nIndex;//1---g1��Ч, 2---g2��Ч

	union
	{
		//[��1��]: ��������ֻ�е�Ƶ����˫Ƶraw���ݵ�ʱ��
		TofExpouseGroup1 g1;//�ع����

		//[��2��]: �������ھ����Զ��ع�֡�͹̶��ع�֡��raw���ݵ�ʱ��֡��HDRZ�ں�ʱ��
		TofExpouseGroup2 g2;//�ع����
	}uParam;

}TofExpouseItems;

//TOF Expouse Current Group1
typedef struct tagTofExpouseCurrentGroup1
{
	UINT32 exp;//�ع�ֵ
}TofExpouseCurrentGroup1;


//TOF Expouse Current Group2
typedef struct tagTofExpouseCurrentGroup2
{
	UINT32 exp_AEF;//�Զ��ع�֡�ع�ֵ
	UINT32 exp_FEF;//�̶��ع�֡�ع�ֵ
}TofExpouseCurrentGroup2;

//TOF Expouse Current Items
typedef struct tagTofExpouseCurrentItems
{
	UINT32 nIndex;//1---g1��Ч, 2---g2��Ч

	union
	{
		//[��1��]: ��������ֻ�е�Ƶ����˫Ƶraw���ݵ�ʱ��
		TofExpouseCurrentGroup1 g1;//�ع�ֵ

		//[��2��]: �������ھ����Զ��ع�֡�͹̶��ع�֡��raw���ݵ�ʱ��֡��HDRZ�ں�ʱ��
		TofExpouseCurrentGroup2 g2;//�ع�ֵ
	}uParam;

}TofExpouseCurrentItems;


//TOF Expouse Range Group1
typedef struct tagTofExpouseRangeGroup1
{
	UINT32 min;//�ع�ֵ(��С)
	UINT32 max;//�ع�ֵ(���)
}TofExpouseRangeGroup1;


//TOF Expouse Range Group2
typedef struct tagTofExpouseRangeGroup2
{
	UINT32 min_AEF;//�Զ��ع�֡�ع�ֵ(��С)
	UINT32 max_AEF;//�Զ��ع�֡�ع�ֵ(���)
	
	UINT32 min_FEF;//�̶��ع�֡�ع�ֵ(��С)
	UINT32 max_FEF;//�̶��ع�֡�ع�ֵ(���)
}TofExpouseRangeGroup2;


//TOF Expouse Range Items
typedef struct tagTofExpouseRangeItems
{
	UINT32 nIndex;//1---g1��Ч, 2---g2��Ч

	union
	{
		//[��1��]: ��������ֻ�е�Ƶ����˫Ƶraw���ݵ�ʱ��
		TofExpouseRangeGroup1 g1;//�عⷶΧ

		//[��2��]: �������ھ����Զ��ع�֡�͹̶��ع�֡��raw���ݵ�ʱ��֡��HDRZ�ں�ʱ��
		TofExpouseRangeGroup2 g2;//�عⷶΧ
	}uParam;

}TofExpouseRangeItems;


//�Զ�������ͻ�ʶ���
typedef enum tagCUSTOM_PARAM_GUEST_ID
{
	CUSTOM_PARAM_GUEST_ID_1 = 1,//�ͻ�1
	CUSTOM_PARAM_GUEST_ID_2 = 2,//�ͻ�2

}CUSTOM_PARAM_GUEST_ID;

//�ͻ�1�Զ���Ĳ���
typedef struct tagCustomParamGuest1
{
	SINT32 quantileThreshold;//AE ����
	FLOAT32 referenceAmplitude;//�ο�����
	FLOAT32 amplitudeThreshold;//������ֵ
	UINT8 szRes[496];//�ܳ�508�ֽڣ�4�ֽڶ��룬Ԥ��
}CustomParamGuest1;

//�ͻ�2�Զ���Ĳ���
typedef struct tagCustomParamGuest2
{
	UINT8 szRes[508];//�ܳ�508�ֽڣ�4�ֽڶ��룬Ԥ��
}CustomParamGuest2;


//�ͻ��Զ���Ĳ���
typedef struct tagGuestCustomParam
{
	CUSTOM_PARAM_GUEST_ID id;//���������ֻ��

	union
	{
		CustomParamGuest1 p1;//��idΪCUSTOM_PARAM_GUEST_ID_1ʱ��Ч��
		CustomParamGuest2 p2;//��idΪCUSTOM_PARAM_GUEST_ID_2ʱ��Ч��


		UINT8 data[508];//�޶�������Ϊ508�ֽڳ��ȣ����ֶβ�ʹ�ã����������ݽṹ���ȶ��壩
	}uParam;
}GuestCustomParam;




//TOFģ���ڲκͻ��䣨ͨ��ģ�ͣ�
typedef struct tagTofModuleLensGeneral
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
}TofModuleLensGeneral;

//TOFģ���ڲκͻ��䣨����ģ�ͣ�
typedef struct tagTofModuleLensFishEye
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 k3;
	FLOAT32 k4;
}TofModuleLensFishEye;

//TOFģ���ڲκͻ��䣨V1.0�汾�����鲻Ҫ���ã���Ϊ��������������ģ�ͣ�
typedef struct tagTofModuleLensParameter
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
	//FLOAT32 k4;
}TofModuleLensParameter;

//TOFģ���ڲκͻ��䣨V2.0�汾��
typedef struct tagTofModuleLensParameterV20
{
	UINT32 nIndex;//1---general��Ч, 2---fishEye��Ч

	union
	{
		//[��1��]: ��ͨģ��
		TofModuleLensGeneral general;//��ͨģ��

		//[��2��]: ����ģ��
		TofModuleLensFishEye fishEye;//����ģ��
	}uParam;

}TofModuleLensParameterV20;

//TOFģ��궨����
typedef struct tagTofCalibData
{
	UINT8* pData;//ָ��궨����
	UINT32 nDataLen;//pData�ڱ궨���ݳ���
}TofCalibData;

typedef struct tagTofRawData
{
	//RAW����
	UINT8* pRaw;//һ֡RAW����
	UINT32 nRawLen;//RAW���ݳ��ȣ��ֽ�����

	//RAW�����������Բ���
	FLOAT32 fTemperature;//��RAW����ʱģ���¶ȣ�ע�⣺�����ͺ�ģ�鲻��Ҫ���ֶΡ�����ģ��RAW�����Դ������ݣ���ô��������0ֵ��

}TofRawData;

typedef struct tagExterntionHooks
{
	void* pUserData;//�û��Զ�������

	/**********������ǰ�ͳ�����������ع�ֵ**********/
	//@    pExp:  ��������ع�ֵ��Ϣ;
	//@    user_data:  �û��Զ������ݣ���pUserData����ͬһ��;
	//@    ���ر�ע�⡿:  �����ڸûص������ڵ���TOFM_XXX�ӿ�ʱ��ֻ�����������㷨���ֽӿڣ��������������������
	void(*RecvTofExpTime)(TofExpouseCurrentItems* pExp, void*user_data);//����ģ��ʵ�����ѡ���Ƿ�ʵ��


}ExterntionHooks;

#endif //__TYPEDEF_H__

