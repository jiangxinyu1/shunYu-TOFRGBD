#ifndef __I2C_H_
#define __I2C_H_

#include <stdio.h>
#include <linux/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <cstring>
#include <pthread.h>

#define	I2C_RETRIES				0x0701		/*设置收不到ACK时的重试次数*/
#define	I2C_TIMEOUT				0x0702 		/* 设置超时时限的jiffies*/
#define	I2C_RDWR				0x0707		/*Combined R/W transfer (one STOP only) */
#define	I2C_M_RD				0x0001		/* 读I2C*/
#define	I2C_TENBIT				0x0704		/*设置从设备地址10bit or 7bit*/
#define	I2C_SLAVE_FORCE			0x0706		/*设置从设备地址*/

#define TOF_I2C_DEV_PATH		"/dev/i2c-2"
#define TOF_I2C_SLAVE_ADDR		0x3d

int I2cCbInit();
void I2cCbUninit();
int Control_Register_R_W(int iRegAddr, int iFlag, int* iValue);
int Read_Cailb_Data_FromFlash(char * calib_file_name, char * RINS_file_name);
int Sunny_MTP009A_17Raw_Mr813( unsigned int expTime_AEF,  unsigned int expTime_FEF);
int data_convert(unsigned char* data,unsigned char* buffer_out,unsigned int num);
int Mtp013RawDataConvert(unsigned char *input_raw, unsigned char *output_raw);

int i2c3_open(int indexfd);
int i2c3_close(void);
int Control_Register_ForDepthSetExp(int slave_address,int register_address,int flag,int* value);

#endif

