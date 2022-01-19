#ifndef CAMERADEMO__H__
#define CAMERADEMO__H__

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
#include <pthread.h>


int time_start(struct timeval *pstTimeVal);
int time_stop(struct timeval *pstTimeVal);
int FileWrite(char *pcFilePath, unsigned char *pucData, unsigned int uiSize);


#endif
