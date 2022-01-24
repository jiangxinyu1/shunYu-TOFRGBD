#ifndef __TOF_MODULE_H__
#define __TOF_MODULE_H__

#include "tof_typedef.h"
#include "tof_error.h"

#ifdef WIN32
    #ifdef TOF_MODULE_SDK_EXPORT
        #define TOFMDLL __declspec(dllexport)
    #else
        #define TOFMDLL __declspec(dllimport)
    #endif
#else
    #define TOFMDLL 
#endif


typedef struct tagTofModuleInitParam
{
	SCHAR szDepthCalcCfgFileDir[200];//��ȼ������������ļ���Ŀ¼����home/user/temp
	UINT8 nLogLevel;//��־��ӡ����

}TofModuleInitParam;

typedef struct tagTofModuleCaps
{	
	UINT32 supportedTOFMode;//TOF_MODE�����
	UINT32 tofResWidth;
	UINT32 tofResHeight;
	GRAY_FORMAT grayFormat;//�Ҷ����ݸ�ʽ

	//TOF HDRZ
	SBOOL bTofHDRZSupported;
	UINT8 byRes1[3];//�ֽڶ��룬Ԥ��

	//TOF RemoveINS
	SBOOL bTofRemoveINSSupported;
	//TOF MPIFlag
	SBOOL bTofMPIFlagSupported;//[���ֶ�������]
	UINT8 byRes2[2];//�ֽڶ��룬Ԥ��

	//TOF Filter
	UINT32 supportedTOFFilter; //TOF_FILTER�����

	//TOF Expouse
	UINT32 supportedTofExpMode;//EXP_MODE�����[���ֶ�������]

}TofModuleCaps;

typedef enum tagMODULE_NAME
{
	MODULE_NAME_FIRST = 0,

	MODULE_NAME_MD101D = MODULE_NAME_FIRST,
	MODULE_NAME_MTP004, 
	MODULE_NAME_MTP004C,
	MODULE_NAME_MTP006,
	MODULE_NAME_MTP007,
	MODULE_NAME_MTP008,
	MODULE_NAME_MTP009,
	MODULE_NAME_MTP009A,
	MODULE_NAME_MTP012,
	MODULE_NAME_MTT010,
	MODULE_NAME_MTT011,
	MODULE_NAME_MTT013,
	MODULE_NAME_MTT014,
	MODULE_NAME_MTT015,
	MODULE_NAME_MTT015A,
	MODULE_NAME_MTT016,
	MODULE_NAME_MTT020,
	MODULE_NAME_YMTT002,
	MODULE_NAME_YMTT003,
	MODULE_NAME_MTP013,



	MODULE_NAME_LAST,//����ģ���ͺŷ��ڸ�ö��ֵ֮ǰ��Ϊ�˼��ݣ��谴�Ⱥ�˳����ӣ����øı�ԭ��ö��ȡֵ��
}MODULE_NAME;


//ģ��SDK�Ŀͻ�ʶ���
typedef enum tagMODULE_GUEST_ID
{
	MODULE_GUEST_ID_DEF = 0x00,//Ĭ�Ͽͻ�
	MODULE_GUEST_ID_01  = 0x01,//�ͻ�01
	MODULE_GUEST_ID_02  = 0x02,//�ͻ�02
	MODULE_GUEST_ID_03  = 0x03,//�ͻ�03
	MODULE_GUEST_ID_04  = 0x04,//�ͻ�04
	MODULE_GUEST_ID_05  = 0x05,//�ͻ�05
	MODULE_GUEST_ID_06  = 0x06,//�ͻ�06
	MODULE_GUEST_ID_07  = 0x07,//�ͻ�07
	MODULE_GUEST_ID_08  = 0x08,//�ͻ�08
	MODULE_GUEST_ID_09  = 0x09,//�ͻ�09

	MODULE_GUEST_ID_10  = 0x10,//�ͻ�10
	MODULE_GUEST_ID_11  = 0x11,//�ͻ�11
	MODULE_GUEST_ID_12  = 0x12,//�ͻ�12
	MODULE_GUEST_ID_13  = 0x13,//�ͻ�13
	MODULE_GUEST_ID_14  = 0x14,//�ͻ�14
	MODULE_GUEST_ID_15  = 0x15,//�ͻ�15
	MODULE_GUEST_ID_16  = 0x16,//�ͻ�16
	MODULE_GUEST_ID_17  = 0x17,//�ͻ�17
	MODULE_GUEST_ID_18  = 0x18,//�ͻ�18
	MODULE_GUEST_ID_19  = 0x19,//�ͻ�19

	MODULE_GUEST_ID_20  = 0x20,//�ͻ�20
	MODULE_GUEST_ID_21  = 0x21,//�ͻ�21
	MODULE_GUEST_ID_22  = 0x22,//�ͻ�22
	MODULE_GUEST_ID_23  = 0x23,//�ͻ�23
	MODULE_GUEST_ID_24  = 0x24,//�ͻ�24
	MODULE_GUEST_ID_25  = 0x25,//�ͻ�25
	MODULE_GUEST_ID_26  = 0x26,//�ͻ�26
	MODULE_GUEST_ID_27  = 0x27,//�ͻ�27
	MODULE_GUEST_ID_28  = 0x28,//�ͻ�28
	MODULE_GUEST_ID_29  = 0x29,//�ͻ�29


}MODULE_GUEST_ID;


typedef void* HTOFM;

typedef struct tagTofModuleHal
{
	SBOOL(*Init)(void* user_data);
	SBOOL(*Deinit)(void* user_data);
	
	/**********д��һ��ֵ���Ĵ���**********/
	//@    slave_addr:  �ӻ���ַ;
	//@    regAddr:      �Ĵ�����ַ;
	//@    value:           Ҫд��Ĵ�����ֵ;
	//@    ����ֵ:       �ɹ���ʧ��;
	SBOOL(*WriteReg16)(const UINT8 slave_addr, const UINT16 regAddr, const UINT16 value, void*user_data);//����ģ��ʵ�����ѡ���Ƿ�ʵ��

	/**********��ȡһ���Ĵ�����ֵ**********/
	//@    slave_addr:  �ӻ���ַ;
	//@    regAddr:      �Ĵ�����ַ;
	//@    value:           ��ȡ����ֵ;
	//@    ����ֵ:       �ɹ���ʧ��;
	SBOOL(*ReadReg16)(const UINT8 slave_addr, const UINT16 regAddr, UINT16 *value, void*user_data);//����ģ��ʵ�����ѡ���Ƿ�ʵ��

	/**********��ȡ����Ĵ�����ֵ**********/
	//@    slave_addr:  �ӻ���ַ;
	//@    startAddr:    Ҫ��ȡ����ʼ��ַ;
	//@    addrCnt:      Ҫ��ȡ�ĵ�ַ����;
	//@    pBufOut:     ��Ŷ�ȡ��������;
	//@    ����ֵ:      ��ȡ�������ݳ���;
	UINT32(*ReadRegBulk)(const UINT8 slave_addr, const UINT32 startAddr, const UINT32 addrCnt, void *pBufOut, void*user_data);//����ģ��ʵ�����ѡ���Ƿ�ʵ��


}TofModuleHal;


typedef struct tagRoiItem
{
	UINT32  left;//��ʼ�У���0��ʼ;
	UINT32  top;//��ʼ�У���0��ʼ;
	UINT32  right;//��ֹ�У�������ͼ���;
	UINT32  bottom;//��ֹ�У�������ͼ���;

}RoiItem;


typedef struct tagDepthCalRoi
{
	RoiItem struMax;//���ֵ��ֻ��
	RoiItem struDefault;//Ĭ��ֵ��ֻ��

	RoiItem struCurrent;//��ǰֵ���ɶ�д

}DepthCalRoi;


typedef struct tagTofModDepthData
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

    //
	FLOAT32* pDepthData;//���߾���
	//
	PointData *pPointData;//��������
	//
	GRAY_FORMAT grayFormat;//pGrayData�����ݸ�ʽ
	void   *pGrayData;//�Ҷ�����

	//��չ����(һ����Կͻ���������)����ͬ�豸/��ͬ�ͻ�����ͬ������Ϊ�գ�
	void   *pExtData;//��չ����
	UINT32 nExtDataLen;//pExtData����չ���ݳ��ȣ��ֽ���

	TofExpouseCurrentItems  autoExp;//��������Զ��ع�ֵ������Ҫ�Զ��ع�Ч��ʱ����Ҫ����ֵ���õ�ģ���У�

}TofModDepthData;



typedef struct tagTofModDepthDataV20
{
	UINT64  timeStamp;
	UINT32  frameWidth;
	UINT32  frameHeight;

	//
	FLOAT32* pDepthData;//���߾���
	//
	PointData *pPointData;//��������
	//
	GRAY_FORMAT grayFormat;//pGrayData�����ݸ�ʽ
	void   *pGrayData;//�Ҷ�����

	//��չ����(һ����Կͻ���������)����ͬ�豸/��ͬ�ͻ�����ͬ������Ϊ�գ�
	void   *pExtData;//��չ����
	UINT32 nExtDataLen;//pExtData����չ���ݳ��ȣ��ֽ���

	TofExpouseCurrentItems  autoExp;//��������Զ��ع�ֵ������Ҫ�Զ��ع�Ч��ʱ����Ҫ����ֵ���õ�ģ���У�


	/**************֡��HDRZ�ں�ʱ��������************/

	//��������Զ��ع�֡���ݣ��м����ݣ�
	FLOAT32* pDepthData_AEF;//���߾���
	PointData *pPointData_AEF;//��������
	GRAY_FORMAT grayFormat_AEF;//pGrayData_AEF�����ݸ�ʽ
	void   *pGrayData_AEF;//�Ҷ�����

	//������Ĺ̶��ع�֡���ݣ��м����ݣ�
	FLOAT32* pDepthData_FEF;//���߾���
	PointData *pPointData_FEF;//��������
	GRAY_FORMAT grayFormat_FEF;//pGrayData_FEF�����ݸ�ʽ
	void   *pGrayData_FEF;//�Ҷ�����


}TofModDepthDataV20;


typedef struct tagSomeCalibParam
{
	//
	TofModuleLensParameter struLensParamter;//TOFģ���ڲκͻ��䣨V1.0�汾�����鲻Ҫ���ã���Ϊ��������������ģ�ͣ�
	TofModuleLensParameterV20 struLensParamterV20;//TOFģ���ڲκͻ��䣨V2.0�汾��

	//

}SomeCalibParam;



#ifdef __cplusplus
extern "C" {
#endif


/*****************************************************************************/
/*********************��һ���֣�SDK�����ӿڣ����ã�**************************/
/*****************************************************************************/

//��ʼ��/����ʼ��SDK�������κνӿڵ�ʹ�ö���������������ӿ�֮�䣩
TOFMDLL TOFRET TOFM_Init(TofModuleInitParam* pInitParam);
TOFMDLL TOFRET TOFM_Uninit(void);

//��ȡSDK�汾�ţ�����ֵΪ�ַ����Ͱ汾�ţ�
TOFMDLL SCHAR* TOFM_GetSDKVersion(void);

//����/�ͷž����Դ
//�Ͻӿڣ��𽥷�������������MODULE_NAMEö�٣�
TOFMDLL HTOFM  TOFM_OpenDevice(const MODULE_NAME mod_name, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
//�½ӿڣ������Ͻӿڣ�����MODULE_NAMEö�ٲ�֧�ֵ�ģ���ͺ�һ��Ҫ���½ӿڣ�
TOFMDLL HTOFM  TOFM_OpenDeviceV20(SCHAR* pModName, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
//�½ӿڣ������Ͻӿڣ�����MODULE_NAMEö�ٲ�֧�ֵ�ģ���ͺ�һ��Ҫ���½ӿڣ������V20�ӿڣ�����ʵ�ֲ�ͬ�ͻ���ͬһģ��Ĳ�ͬ��������
TOFMDLL HTOFM  TOFM_OpenDeviceV30(SCHAR* pModName, const MODULE_GUEST_ID guestID, TofModuleHal* pHal, void* pHalUserData, TofModuleCaps* pCaps);
TOFMDLL TOFRET TOFM_CloseDevice(HTOFM hTofMod);

//��TOF ģʽ����ϵ�������ı궨���ݽ�������ȼ����ʼ����ȡ�����ܣ�
//�Ͻӿڣ�����Ĭ�ϵ������ļ���
TOFMDLL TOFRET TOFM_SetTofMode(HTOFM hTofMod, const TOF_MODE tofMode);
//�½ӿڣ���������ָ�������ļ���, pModCfgFileΪ�����ļ�����·�����ҿ���ΪNULLֵ��NULLʱ��ʾ����Ĭ�������ļ���
TOFMDLL TOFRET TOFM_SetTofModeV20(HTOFM hTofMod, const TOF_MODE tofMode, SCHAR* pModCfgFile);

//��һЩ�ص��������Ǳ��뺯����������ã�
TOFMDLL TOFRET TOFM_SetExterntionHooks(HTOFM hTofMod, ExterntionHooks* pHooks);


/*****************************************************************************/
/*********************�ڶ����֣��ò���Ϊģ��Ӳ����ز�����Ӳ��������**********/
/*****************************************************************************/

//��ģ�����ȡ�궨����
TOFMDLL UINT32 TOFM_ReadCalibData(HTOFM hTofMod, UINT8* pCalibData, const UINT32 nBufLen);
//��ȡ/����ģ���ع�
TOFMDLL TOFRET TOFM_GetTofExpTime(HTOFM hTofMod, TofExpouseItems* pExp);
TOFMDLL TOFRET TOFM_SetTofExpTime(HTOFM hTofMod, TofExpouseCurrentItems* pExp);
//��ȡģ���¶�
TOFMDLL TOFRET TOFM_GetTemperature(HTOFM hTofMod, FLOAT32* pTemperature);
//����/�ر�ģ��MIPI����
TOFMDLL TOFRET TOFM_StartTofStream(HTOFM hTofMod);
TOFMDLL TOFRET TOFM_StopTofStream(HTOFM hTofMod);


/*****************************************************************************/
/*********************�������֣��ò���Ϊ��ȼ�������㷨������㷨��**********/
/*****************************************************************************/

//����/�ͷű궨����
TOFMDLL TOFRET TOFM_LoadCalibData(HTOFM hTofMod, UINT8* pCalibData, const UINT32 nCalibDataLen);
TOFMDLL TOFRET TOFM_UnLoadCalibData(HTOFM hTofMod);
//��ȡ�궨�����ڲ��ֱ궨����
TOFMDLL TOFRET TOFM_GetSomeCalibParam(HTOFM hTofMod, SomeCalibParam* pParamOut); //���ýӿڱ�����TOFM_LoadCalibData֮��TOFM_UnLoadCalibData֮ǰ���ã�

//��ȼ���ģ�飨�ò��ֱ�����TOFM_LoadCalibData֮��TOFM_UnLoadCalibData֮ǰ���ã�
TOFMDLL TOFRET TOFM_InitDepthCal(HTOFM hTofMod);//(�ȽϺ�ʱ)
TOFMDLL TOFRET TOFM_UnInitDepthCal(HTOFM hTofMod);
TOFMDLL TOFRET TOFM_DoDepthCal(HTOFM hTofMod, TofRawData* pRawData, TofModDepthData* pDataOut);//���ýӿڱ�����TOFM_InitDepthCal֮��TOFM_UnInitDepthCal֮ǰ���ã�
TOFMDLL TOFRET TOFM_DoDepthCalV20(HTOFM hTofMod, TofRawData* pRawData, TofModDepthDataV20* pDataOut);//���ýӿڱ�����TOFM_InitDepthCal֮��TOFM_UnInitDepthCal֮ǰ���ã�

TOFMDLL TOFRET TOFM_DoDepthCalV20_OnlyAEExp(HTOFM hTofMod, TofRawData* pRawData, TofModDepthDataV20* pDataOut);//���ýӿڱ�����TOFM_InitDepthCal֮��TOFM_UnInitDepthCal֮ǰ���ã�
TOFMDLL TOFRET TOFM_DoDepthCalV20_OnlyDepth(HTOFM hTofMod, TofRawData* pRawData, TofModDepthDataV20* pDataOut);//���ýӿڱ�����TOFM_InitDepthCal֮��TOFM_UnInitDepthCal֮ǰ���ã�


//����/����AE�㷨[������]
TOFMDLL TOFRET TOFM_SetTofAE(HTOFM hTofMod, const SBOOL bEnable);
//����/����ĳ���˲��㷨
TOFMDLL TOFRET TOFM_GetTofFilter(HTOFM hTofMod, const TOF_FILTER type, SBOOL* pbEnable);
TOFMDLL TOFRET TOFM_SetTofFilter(HTOFM hTofMod, const TOF_FILTER type, const SBOOL bEnable);
//����/����ĳ���˲��㷨�������������
TOFMDLL TOFRET TOFM_GetTofFilterV20(HTOFM hTofMod, TofFilterCfg* pCfg);
TOFMDLL TOFRET TOFM_SetTofFilterV20(HTOFM hTofMod, TofFilterCfg* pCfg);

//����/����HDRZ�㷨
TOFMDLL TOFRET TOFM_SetTofHDRZ(HTOFM hTofMod, const SBOOL bEnable);

//����/����RemoveINS�㷨
TOFMDLL TOFRET TOFM_SetTofRemoveINS(HTOFM hTofMod, const SBOOL bEnable);

//����/����MPIFlag�㷨[�����ã���ʹ��TOFM_SetTofFilter(xxx, TOF_FILTER_MPIFilter, xxx)]
TOFMDLL TOFRET TOFM_SetTofMPIFlag(HTOFM hTofMod, const SBOOL bEnable);

//����raw���ݵĸ�ʽ���Ǳ��뺯����������ã�������Ƕ��ʽƽ̨�´�С��ת����
TOFMDLL TOFRET TOFM_ConvertRawData(HTOFM hTofMod, UINT8* pRaw, const UINT32 nRawLen, UINT8* pRawOut, const UINT32 nOutBufLen);

//������ȼ����ROI���Ǳ��뺯����������ã�
TOFMDLL TOFRET TOFM_GetDepthCalRoi(HTOFM hTofMod, DepthCalRoi* pRoi);
TOFMDLL TOFRET TOFM_SetDepthCalRoi(HTOFM hTofMod, DepthCalRoi* pRoi);

//��ȡ/�����ع������ޣ��Ǳ��뺯����������ã�
TOFMDLL TOFRET TOFM_GetTofExpTimeRange(HTOFM hTofMod, TofExpouseRangeItems* pRange);
TOFMDLL TOFRET TOFM_SetTofExpTimeRange(HTOFM hTofMod, TofExpouseRangeItems* pRange);

//��ȡ/�����ͻ����Զ���Ҫ������ݣ��Ǳ��뺯����������ã�
TOFMDLL TOFRET TOFM_GetGuestCustomParam(HTOFM hTofMod, GuestCustomParam* pParam);
TOFMDLL TOFRET TOFM_SetGuestCustomParam(HTOFM hTofMod, GuestCustomParam* pParam);



#ifdef __cplusplus
}
#endif

#endif

