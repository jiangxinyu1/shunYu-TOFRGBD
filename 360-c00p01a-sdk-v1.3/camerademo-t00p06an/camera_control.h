#ifndef	__CAMERA_CONTROL_H__
#define __CAMERA_CONTROL_H__

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <semaphore.h>
#include <ctype.h>
#include <errno.h>
#include <stdio.h>

#include "tof_i2c.h"
#include "tof_typedef.h"
#include "tof_mod_sdk.h"

#define RAW_NUM 17

#define WIDTH           		224
#define HEIGHT          		173

#define RAW_HEIGHT      		(HEIGHT * RAW_NUM)
#define MTP013_RAW_HEIGHT		(129 * RAW_NUM)
#define PIXEL_FORMAT   	 		V4L2_PIX_FMT_SBGGR12
#define PER_FRAME_PIXEL_NUM		(WIDTH * HEIGHT)
#define ACTIVE_HEIGHT			114
#define RAW_SIZE				(WIDTH*MTP013_RAW_HEIGHT*2)////RAW_HEIGHT      MTP013_RAW_HEIGHT
#define POINTCLOUD_SIZE			(WIDTH*ACTIVE_HEIGHT*sizeof(float)*4)
#define SEND_BUFF_SIZE			(POINTCLOUD_SIZE+RAW_SIZE+sizeof(unsigned int))
#define ACTIVE_PIXEL_NUM		(WIDTH*ACTIVE_HEIGHT)
#define CAMERA_DBG_EN			0

#define RGB_WIDTH           		1920
#define RGB_HEIGHT          		1080
#define RGB_PIXEL_FORMAT   	 		V4L2_PIX_FMT_NV12

//for internel debug
#define camera_dbg(x,arg...) do{ \
                                if(CAMERA_DBG_EN) \
                                    printf("[CAMERA_DEBUG]" x ,##arg); \
                            }while(0)

//print when error happens
#define camera_err(x,arg...) do{ \
                                printf("\033[1m\033[;31m[CAMERA_ERR]" x ,##arg); \
                                printf("\033[0m"); \
                                fflush(stdout); \
                            }while(0)

#define camera_prompt(x,arg...) do{ \
                                printf("\033[1m\033[;32m[CAMERA_PROMPT]" x "\033[0m",##arg); \
                                fflush(stdout); \
                            }while(0)

#define camera_warn(x,arg...) printf("[CAMERA_WARN]" x ,##arg)

//print unconditional, for important info
#define camera_print(x,arg...) printf("[CAMERA]" x ,##arg)


struct buffer
{
    void *start[3];
    size_t length[3];
};

typedef struct camera_hal
{
    int camera_index;
    int videofd;
    int driver_type;
    int photo_num;
    char save_path[64];
    struct buffer *buffers;
	struct v4l2_buffer stV4l2Buf;
    int buf_count;
    int nplanes;
	int isStreaming;
    unsigned int win_width;
    unsigned int win_height;
    unsigned int pixelformat;
} camera_handle;

typedef struct camera_rgbhal
{
    int camera_index;
    int videofd;
    int driver_type;
    int photo_num;
    char save_path[64];
    struct buffer *buffers;
	struct v4l2_buffer stV4l2Buf;
    int buf_count;
    int nplanes;
	int isStreaming;
    unsigned int win_width;
    unsigned int win_height;
    unsigned int pixelformat;
} camera_rgb_handle;

typedef struct TofRawDataCb
{
	int isAvailable;
	int isProcess; 
	unsigned int uiFrameCnt; 
	TofRawData stRawData;
} TOF_RAW_DATA_CB_S;

typedef struct TofOutDataCb
{
	int isAvailable;
	unsigned int uiFrameCnt; 
	TofModDepthData stDepthDataInfo;
} TOF_OUT_DATA_CB_S;

int OpenTofCamera(void);
void CloseTofCamera(void);
int GetTofRawData(TOF_RAW_DATA_CB_S *pstTofRawDataCb);
int GetTofRawDataFormLocal(TOF_RAW_DATA_CB_S *pstTofRawDataCb, unsigned int uiNum, unsigned int uiStartIndex);

int OpenRGBCamera(void);
void CloseRGBCamera(void);
int GetRGBRawData(unsigned char *rgb_buf);

#endif	/* __CAMERA_CONTROL_H__ */

