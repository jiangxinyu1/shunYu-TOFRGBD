#ifndef __RGA_YUV_RGB_H__
#define __RGA_YUV_RGB_H__

#if defined(__cplusplus)
extern "C" {
#endif

int rga_yuv420sp_to_rgba(unsigned char * dst_buf, unsigned char * src_buf, int width_buf, int height_buf);

#if defined(__cplusplus)
}
#endif

#endif

