#ifndef __TOF_RGBD_SDK_H__
#define __TOF_RGBD_SDK_H__


#ifdef WIN32
    #ifdef TOF_RGBD_SDK_EXPORT
        #define TOFRGBDDLL __declspec(dllexport)
    #else
        #define TOFRGBDDLL __declspec(dllimport)
    #endif
#else
    #define TOFRGBDDLL 
#endif

#ifndef NULL
	#define NULL 0
#endif


//������
typedef enum tagTOFRGBDRET
{
	TOFRGBDRET_SUCCESS              = 0x00000000,//�ɹ�
		
	TOFRGBDRET_FAILED               = 0x80000001,//ʧ��
	TOFRGBDRET_ERROR_INVALID_PARAM  = 0x80000002,//��Ч����
	TOFRGBDRET_ERROR_ACCESS         = 0x80000003,//��Ȩ��
	TOFRGBDRET_ERROR_OVERFLOW       = 0x80000004,//���
	TOFRGBDRET_ERROR_NO_MEM         = 0x80000005,//�ڴ治��
	TOFRGBDRET_ERROR_WRONG_STATUS   = 0x80000006,//״̬����
	TOFRGBDRET_ERROR_NOT_SUPPORTED  = 0x80000007,//���ܲ�֧��
	TOFRGBDRET_ERROR_DATA           = 0x80000008,//���������
	


	TOFRGBDRET_ERROR_OTHER          = 0x8fffffff,//����δ�ض�ָ���Ĵ���
}TOFRGBDRET;

//RGBD SDK�Ŀͻ�ʶ���
typedef enum tagTOFRGBD_GUEST_ID
{
	TOFRGBD_GUEST_ID_DEF = 0x00,//Ĭ�Ͽͻ�
	TOFRGBD_GUEST_ID_01 = 0x01,//�ͻ�01
	TOFRGBD_GUEST_ID_02 = 0x02,//�ͻ�02
	TOFRGBD_GUEST_ID_03 = 0x03,//�ͻ�03
	TOFRGBD_GUEST_ID_04 = 0x04,//�ͻ�04
	TOFRGBD_GUEST_ID_05 = 0x05,//�ͻ�05
	TOFRGBD_GUEST_ID_06 = 0x06,//�ͻ�06
	TOFRGBD_GUEST_ID_07 = 0x07,//�ͻ�07
	TOFRGBD_GUEST_ID_08 = 0x08,//�ͻ�08
	TOFRGBD_GUEST_ID_09 = 0x09,//�ͻ�09

	TOFRGBD_GUEST_ID_10 = 0x10,//�ͻ�10
	TOFRGBD_GUEST_ID_11 = 0x11,//�ͻ�11
	TOFRGBD_GUEST_ID_12 = 0x12,//�ͻ�12
	TOFRGBD_GUEST_ID_13 = 0x13,//�ͻ�13
	TOFRGBD_GUEST_ID_14 = 0x14,//�ͻ�14
	TOFRGBD_GUEST_ID_15 = 0x15,//�ͻ�15
	TOFRGBD_GUEST_ID_16 = 0x16,//�ͻ�16
	TOFRGBD_GUEST_ID_17 = 0x17,//�ͻ�17
	TOFRGBD_GUEST_ID_18 = 0x18,//�ͻ�18
	TOFRGBD_GUEST_ID_19 = 0x19,//�ͻ�19

	TOFRGBD_GUEST_ID_20 = 0x20,//�ͻ�20
	TOFRGBD_GUEST_ID_21 = 0x21,//�ͻ�21
	TOFRGBD_GUEST_ID_22 = 0x22,//�ͻ�22
	TOFRGBD_GUEST_ID_23 = 0x23,//�ͻ�23
	TOFRGBD_GUEST_ID_24 = 0x24,//�ͻ�24
	TOFRGBD_GUEST_ID_25 = 0x25,//�ͻ�25
	TOFRGBD_GUEST_ID_26 = 0x26,//�ͻ�26
	TOFRGBD_GUEST_ID_27 = 0x27,//�ͻ�27
	TOFRGBD_GUEST_ID_28 = 0x28,//�ͻ�28
	TOFRGBD_GUEST_ID_29 = 0x29,//�ͻ�29


}TOFRGBD_GUEST_ID;


//�Ҷȸ�ʽ
typedef enum tagTofRgbd_Gray_Format
{
	TofRgbd_Gray_Format_U8    = 0,//U8��ʽ
	TofRgbd_Gray_Format_U16   = 1,//U16��ʽ
	TofRgbd_Gray_Format_Float = 2,//float��ʽ
}TofRgbd_Gray_Format;


//����
typedef struct tagTofRgbdPointCloud
{
	float x;
	float y;
	float z;
}TofRgbdPointCloud;

//��ɫ����
typedef struct tagTofRgbdPointCloudColor
{
	float x;
	float y;
	float z;

	float r;
	float g;
	float b;
}TofRgbdPointCloudColor;


//�ͻ����Ƶ�˽������
typedef struct tagTofRgbdImage_Priv
{
	void*        pData;
	unsigned int nDataLen; //pData�������ֽ���
}TofRgbdImage_Priv;

//δָ����ʽ��ͼ����Ϣ
typedef struct tagTofRgbdImage_Void
{
	void*        pData;
	unsigned int nWidth;
	unsigned int nHeight;
	unsigned int nDataLen; //pData�������ֽ���
}TofRgbdImage_Void;

//U8ͼ����Ϣ
typedef struct tagTofRgbdImage_U8
{
	unsigned char* pData;//�����ܳ���nWidth * nHeight * sizeof(pData[0])
	unsigned int   nWidth;
	unsigned int   nHeight;
}TofRgbdImage_U8;

//U16ͼ����Ϣ
typedef struct tagTofRgbdImage_U16
{
	unsigned short* pData;//�����ܳ���nWidth * nHeight * sizeof(pData[0])
	unsigned int    nWidth;
	unsigned int    nHeight;
}TofRgbdImage_U16;

//floatͼ����Ϣ
typedef struct tagTofRgbdImage_Float
{
	float*       pData;//�����ܳ���nWidth * nHeight * sizeof(pData[0])
	unsigned int nWidth;
	unsigned int nHeight;
}TofRgbdImage_Float;

//����ͼ����Ϣ
typedef struct tagTofRgbdImage_PointCloud
{
	TofRgbdPointCloud* pData;//�����ܳ���nWidth * nHeight * sizeof(pData[0])
	unsigned int       nWidth;
	unsigned int       nHeight;
}TofRgbdImage_PointCloud;

//��ɫ����ͼ����Ϣ
typedef struct tagTofRgbdImage_PointCloudColor
{
	TofRgbdPointCloudColor* pData;//�����ܳ���nWidth * nHeight * sizeof(pData[0])
	unsigned int            nWidth;
	unsigned int            nHeight;
}TofRgbdImage_PointCloudColor;



//����RGBD�������ĳ�ʼ������
typedef struct tagTofRgbdHandleParam
{
	char szModuleName[32];//ģ���ͺ�
	TOFRGBD_GUEST_ID guestID;//�ͻ�ʶ���

	unsigned char* pRgbdCalibData;//RGBD�궨����
	unsigned int   nRgbdCalibDataLen;//pRgbdCalibData��RGBD�궨���ݳ��ȣ��ֽ�����

	unsigned int nTofWidth;//TOF�ֱ��ʿ�
	unsigned int nTofHeight;//TOF�ֱ��ʸ�

	unsigned int nRgbWidth;//RGB�ֱ��ʿ�
	unsigned int nRgbHeight;//RGB�ֱ��ʸ�

	TofRgbd_Gray_Format inGrayFormat;//����ĻҶȸ�ʽ

}TofRgbdHandleParam;


//RGBD������������
typedef struct tagTofRgbdInputData
{
	TofRgbdPointCloud*  pPointCloud;//�������ݣ�����Ϊ��λ��

	void*        pGray;//�Ҷ����ݣ���ʽ�������ʼ������inGrayFormatָ����һ�£�
	unsigned int nGrayLen;//pGray�ڻҶ����ݳ��ȣ��ֽ�����

	unsigned char* pRgb;//RGB���ݣ�������bgr/rgb�ȸ�ʽ
	unsigned int   nRgbLen;//pRgb��RGB���ݳ��ȣ��ֽ�����

}TofRgbdInputData;

//RGBD������������
typedef struct tagTofRgbdOutputData
{
	TofRgbdImage_PointCloud pointCloud2Rgb;//��׼��ĵ��ƣ�����Ϊ��λ��������Ϊ�գ�
	TofRgbdImage_U8   gray2Rgb;//��׼��ĻҶ����ݣ�����Ϊ�գ�
	TofRgbdImage_Void rgb2Tof;//��׼���RGB���ݣ���ʽ���������ͬ��������Ϊ�գ�
	TofRgbdImage_PointCloudColor colorPointCloud;//��׼���R��ɫ�������ݣ�����Ϊ�գ�

	TofRgbdImage_Priv privData;//˽�����ݣ����ڿͻ����ƣ���һ��Ϊ�գ�

}TofRgbdOutputData;


//RGBD����ľ��
typedef void* HTOFRGBD;


#ifdef __cplusplus
extern "C" {
#endif


//��ȡSDK�汾�ţ�����ֵΪ�ַ����Ͱ汾�ţ�
TOFRGBDDLL char* TOFRGBD_GetSDKVersion(void);

//����/�ͷž����Դ
TOFRGBDDLL HTOFRGBD   TOFRGBD_CreateHandle(TofRgbdHandleParam* pInputParam);
TOFRGBDDLL TOFRGBDRET TOFRGBD_CloseHandle(HTOFRGBD hTofRgbd);
//RGBD����
TOFRGBDDLL TOFRGBDRET TOFRGBD_DoCal(HTOFRGBD hTofRgbd, TofRgbdInputData* pDataIn, TofRgbdOutputData* pDataOut);//���ýӿڱ�����TOFRGBD_CreateHandle֮��TOFRGBD_CloseHandle֮ǰ���ã�


#ifdef __cplusplus
}
#endif

#endif

