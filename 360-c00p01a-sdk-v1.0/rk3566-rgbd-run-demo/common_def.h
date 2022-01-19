#ifndef _COMMON_DEF_H_
#define _COMMON_DEF_H_
typedef struct SIamgeS
{
	int iImgW;
	int iImgH;
	int iChannels;
	void* imgData;
}SImage;

typedef struct SImageSizeS
{
	int iImgW;
	int iImgH;
	
}SImageSize;

typedef struct SRectS
{
	int x;//left top corner x coord
	int y;//left top corner y coord
	int width;
	int height;
}SRect;

#endif
