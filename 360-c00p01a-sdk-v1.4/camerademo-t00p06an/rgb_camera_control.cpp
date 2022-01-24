#include <math.h>

#include "camera_control.h"
#include "rga-yuv-rgba.h"

static camera_rgb_handle gstCameraHandle;

int capture_rgb_yuv = 0;
char source_data_path[64];

static unsigned long long get_tick_count()
{
	unsigned long long tick = 0;

	struct timeval tv;
	gettimeofday(&tv, 0);
	tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);

	return tick;
}

static void test_rgb_fps(void)
{
    static int frames_num = 0;
    static long start_time = 1;

    if (start_time == 1) {
        start_time = get_tick_count();
    }

    frames_num++;

    if (frames_num % 10 == 0) {
        float cur_framerate = 0;
        cur_framerate = (float)frames_num * 1000 / (get_tick_count() - start_time);

        printf("#################### RGB Frame Rate = %.2f\n\n", cur_framerate);
        start_time = get_tick_count();
        frames_num = 0;
    }
}

static const char *get_format_name(unsigned int format)
{
    if(format == V4L2_PIX_FMT_SRGGB12) 
        return "RGGB12";

	if(format == V4L2_PIX_FMT_NV12) 
        return "NV12";
	
    else
        return "unkown";
}

static int V4L2_StreamON_RGB()
{
	int iRet = 0;	
	enum v4l2_buf_type type;
	camera_rgb_handle *pst_camera = &gstCameraHandle;

	/* streamon */
	if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	else
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(pst_camera->videofd, VIDIOC_STREAMON, &type) == -1) {
		camera_err(" VIDIOC_STREAMON error! %s\n",strerror(errno));
		return -1;
	}else
		camera_print(" stream on succeed\n");

	pst_camera->isStreaming = 1;
	return 0;
}

static int V4L2_Init_RGB(void)
{
	char camera_path[16];
	struct v4l2_capability cap; 	 /* Query device capabilities */
	struct v4l2_format fmt; 		 /* try a format */
	struct v4l2_input inp;			 /* select the current video input */
	struct v4l2_streamparm parms;	 /* set streaming parameters */
	struct v4l2_requestbuffers req;  /* Initiate Memory Mapping or User Pointer I/O */
	struct v4l2_buffer buf; 		 /* Query the status of a buffer */
	camera_rgb_handle *pst_camera = &gstCameraHandle;

	int n_buffers = 0;
	int i = 0;

	/* default settings */
	memset(pst_camera, 0, sizeof(camera_rgb_handle));
	memset(camera_path, 0, sizeof(camera_path));

	pst_camera->camera_index 	= 5;///dev/video0 for tof cif,  video5 for old kernel tof isp
	pst_camera->win_width 		= RGB_WIDTH;
	pst_camera->win_height 		= RGB_HEIGHT;
	pst_camera->photo_num 		= 5000000;//²É¼¯Ö¡Êý
	pst_camera->pixelformat 	= RGB_PIXEL_FORMAT;///V4L2_PIX_FMT_NV12 

	camera_print("**********************************************************\n");
	camera_print("* 													   *\n");
	camera_print("* 			 this is sunny rgb 360 camera.  		   *\n");
	camera_print("* 													   *\n");
	camera_print("**********************************************************\n");

	/* 1.open /dev/videoX node */
	sprintf(camera_path, "%s%d", "/dev/video", pst_camera->camera_index);
	camera_print("**********************************************************\n");
	camera_print(" open %s!\n", camera_path);
	camera_print("**********************************************************\n");

	pst_camera->videofd = open((const char*)camera_path, O_RDWR, 0);
	if(pst_camera->videofd < 0){
		camera_err(" open %s fail!!!\n", camera_path);
		return -1;
	}

	/* 2.Query device capabilities */
	memset(&cap, 0, sizeof(cap));
	if(ioctl(pst_camera->videofd,VIDIOC_QUERYCAP,&cap) < 0){
		camera_err(" Query device capabilities fail!!!\n");
	}else{
		camera_dbg(" Querey device capabilities succeed\n");
		camera_dbg(" cap.driver=%s\n",cap.driver);
		camera_dbg(" cap.card=%s\n",cap.card);
		camera_dbg(" cap.bus_info=%s\n",cap.bus_info);
		camera_dbg(" cap.version=0x%08x\n",cap.version);
		camera_dbg(" cap.capabilities=0x%08x\n",cap.capabilities);
	}

	if((cap.capabilities & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)) <= 0){
		camera_err(" The device is not supports the Video Capture interface!!!\n");
		close(pst_camera->videofd);
		return -1;
	}

	if(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE){
		pst_camera->driver_type = V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	}else if(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE){
		pst_camera->driver_type = V4L2_CAP_VIDEO_CAPTURE;
	}else{
		camera_err(" %s is not a capture device.\n",camera_path);
		close(pst_camera->videofd);
		return -1;
	}

	camera_print("**********************************************************\n");
	camera_print(" The number of captured photos is %d.\n", pst_camera->photo_num);

	/* 3.set the data format */
	memset(&fmt, 0, sizeof(struct v4l2_format));
	if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE){
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		fmt.fmt.pix_mp.width 		= pst_camera->win_width;
		fmt.fmt.pix_mp.height 		= pst_camera->win_height;
		fmt.fmt.pix_mp.pixelformat 	= pst_camera->pixelformat;
		fmt.fmt.pix_mp.field 		= V4L2_FIELD_NONE;
	}else{
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width 		= pst_camera->win_width;
		fmt.fmt.pix.height 		= pst_camera->win_height;
		fmt.fmt.pix.pixelformat = pst_camera->pixelformat;
		fmt.fmt.pix.field 		= V4L2_FIELD_NONE;
	}

	if (ioctl(pst_camera->videofd, VIDIOC_S_FMT, &fmt) < 0){
		camera_err(" setting the data format failed!\n");
		close(pst_camera->videofd);
		return -1;
	}

	if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE){
		if(pst_camera->win_width != fmt.fmt.pix_mp.width || pst_camera->win_height != fmt.fmt.pix_mp.height)
			camera_err("MPLane does not support %u * %u\n", pst_camera->win_width, pst_camera->win_height);

		pst_camera->win_width 	= fmt.fmt.pix_mp.width;
		pst_camera->win_height 	= fmt.fmt.pix_mp.height;
		camera_print("$$$$$ sunny VIDIOC_S_FMT succeed\n");///
		camera_print(" fmt.type = %d\n",fmt.type);
		camera_print(" fmt.fmt.pix.width = %d\n",fmt.fmt.pix_mp.width);
		camera_print(" fmt.fmt.pix.height = %d\n",fmt.fmt.pix_mp.height);
		camera_print(" fmt.fmt.pix.pixelformat = %s\n",get_format_name(fmt.fmt.pix_mp.pixelformat));
		camera_print(" fmt.fmt.pix.field = %d\n",fmt.fmt.pix_mp.field);

		if (ioctl(pst_camera->videofd, VIDIOC_G_FMT, &fmt) < 0)
			camera_err(" get the data format failed!\n");

		pst_camera->nplanes = fmt.fmt.pix_mp.num_planes;
	}
	else{
		if(pst_camera->win_width != fmt.fmt.pix.width || pst_camera->win_height != fmt.fmt.pix.height)
			camera_err(" does not support %u * %u\n", pst_camera->win_width, pst_camera->win_height);

		pst_camera->win_width 	= fmt.fmt.pix.width;
		pst_camera->win_height 	= fmt.fmt.pix.height;
		camera_print(" VIDIOC_S_FMT succeed\n");
		camera_print(" fmt.type = %d\n",fmt.type);
		camera_print(" fmt.fmt.pix.width = %d\n",fmt.fmt.pix.width);
		camera_print(" fmt.fmt.pix.height = %d\n",fmt.fmt.pix.height);
		camera_print(" fmt.fmt.pix.pixelformat = %s\n",get_format_name(fmt.fmt.pix.pixelformat));
		camera_print(" fmt.fmt.pix.field = %d\n",fmt.fmt.pix.field);
	}

	/* 4.Initiate Memory Mapping or User Pointer I/O */
	memset(&req, 0, sizeof(struct v4l2_requestbuffers));
	req.count = 4;

	if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
		req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	else
		req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	req.memory = V4L2_MEMORY_MMAP;
	if(ioctl(pst_camera->videofd, VIDIOC_REQBUFS, &req) < 0){
		camera_err(" VIDIOC_REQBUFS failed\n");
		close(pst_camera->videofd);
		return -1;
	}

	/* Query the status of a buffers */
	// camera.buf_count = req.count;
	pst_camera->buf_count = 4;

	printf("!!!!!!!!!!!!!!! reqbuf number is %d\n", pst_camera->buf_count);

	pst_camera->buffers = (buffer*)calloc(req.count, sizeof(struct buffer));
	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		memset(&buf, 0, sizeof(struct v4l2_buffer));
		if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		else
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;
		if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE){
			buf.length = pst_camera->nplanes;
			buf.m.planes =	(struct v4l2_plane *)calloc(buf.length, sizeof(struct v4l2_plane));
		}

		if (ioctl(pst_camera->videofd, VIDIOC_QUERYBUF, &buf) == -1) {
			camera_err(" VIDIOC_QUERYBUF error\n");

			if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
				free(buf.m.planes);
			free(pst_camera->buffers);

			close(pst_camera->videofd);

			return -1;
		}

		if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE){
			for(i=0; i < pst_camera->nplanes; i++){
				pst_camera->buffers[n_buffers].length[i] = buf.m.planes[i].length;
				pst_camera->buffers[n_buffers].start[i] = mmap(NULL , buf.m.planes[i].length,
												   PROT_READ | PROT_WRITE, \
												   MAP_SHARED , pst_camera->videofd, \
												   buf.m.planes[i].m.mem_offset);

				camera_dbg(" map buffer index: %d, mem: %p, len: %x, offset: %x\n",
				   n_buffers, pst_camera->buffers[n_buffers].start[i],buf.m.planes[i].length,
				   buf.m.planes[i].m.mem_offset);
			}
			free(buf.m.planes);
		}else{
			pst_camera->buffers[n_buffers].length[0] = buf.length;
			pst_camera->buffers[n_buffers].start[0] = mmap(NULL , buf.length,
												PROT_READ | PROT_WRITE, \
												MAP_SHARED , pst_camera->videofd, \
												buf.m.offset);
			camera_dbg(" map buffer index: %d, mem: %p, len: %x, offset: %x\n", \
					n_buffers, pst_camera->buffers[n_buffers].start[0],buf.length,buf.m.offset);
		}
	}

	/* 5.Exchange a buffer with the driver */
	for(n_buffers = 0; n_buffers < req.count; n_buffers++) {
		memset(&buf, 0, sizeof(struct v4l2_buffer));
		if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		else
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory= V4L2_MEMORY_MMAP;
		buf.index= n_buffers;
		if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE){
			buf.length = pst_camera->nplanes;
			buf.m.planes =	(struct v4l2_plane *)calloc(buf.length, sizeof(struct v4l2_plane));
		}

		if (ioctl(pst_camera->videofd, VIDIOC_QBUF, &buf) == -1) {
			camera_err(" VIDIOC_QBUF error\n");

			if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
				free(buf.m.planes);
			free(pst_camera->buffers);

			close(pst_camera->videofd);
			return -1;
		}
		if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
			free(buf.m.planes);
	}

	/* 6.Stream ON */
	if(V4L2_StreamON_RGB())
	{
		camera_err(" V4L2_StreamON failed\n");
		return -1;
	}

	/* 7.malloc v4l2 buffer */
	memset(&pst_camera->stV4l2Buf, 0, sizeof(struct v4l2_buffer));
	
    if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        pst_camera->stV4l2Buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    else
        pst_camera->stV4l2Buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
    pst_camera->stV4l2Buf.memory = V4L2_MEMORY_MMAP;
	
    if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE){
        pst_camera->stV4l2Buf.length = pst_camera->nplanes;
        pst_camera->stV4l2Buf.m.planes = (struct v4l2_plane *)calloc(pst_camera->nplanes, sizeof(struct v4l2_plane));
    }

	return 0;
}

static void V4L2_UnInit_RGB(void)
{
	int i,j;

	enum v4l2_buf_type type;
	camera_rgb_handle *pst_camera = &gstCameraHandle;

    /* streamoff */
    if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    else
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (1 == pst_camera->isStreaming)
	{
	    if(ioctl(pst_camera->videofd, VIDIOC_STREAMOFF, &type) == -1)
	        camera_err(" VIDIOC_STREAMOFF error! %s\n",strerror(errno));
		else
			pst_camera->isStreaming = 0;
	}

	/* munmap camera->buffers */
	if (NULL != pst_camera->buffers)
	{
		if(pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE){
			for (i = 0; i < pst_camera->buf_count; ++i)
				for (j = 0; j < pst_camera->nplanes; j++)
					munmap(pst_camera->buffers[i].start[j], pst_camera->buffers[i].length[j]);
		}else{
			for(i=0; i<pst_camera->buf_count; i++)
				munmap(pst_camera->buffers[i].start[0],pst_camera->buffers[i].length[0]);
		}
	}

	/* free camera->buffers and close camera->videofd */
	if((pst_camera->driver_type == V4L2_CAP_VIDEO_CAPTURE_MPLANE)
		&& (NULL != pst_camera->stV4l2Buf.m.planes))
	{
		free(pst_camera->stV4l2Buf.m.planes);
		pst_camera->stV4l2Buf.m.planes = NULL;
	}

	if (NULL != pst_camera->buffers)
	{
		free(pst_camera->buffers);
		pst_camera->buffers = NULL;
	}

	if (pst_camera->videofd > 0)
	{
		close(pst_camera->videofd);
		pst_camera->videofd = -1;
		camera_print("close /dev/video%d\n", pst_camera->camera_index);
	}
}

int OpenRGBCamera(void)
{
	int iRet = 0;

	/* V4L2 Init */
	iRet = V4L2_Init_RGB();
	if (iRet)
	{
		printf("V4L2_Init failed\n");
		return -1;
	}
	return 0;
}

void CloseRGBCamera(void)
{
	V4L2_UnInit_RGB();
}

static bool SaveBufRAW(void* pBufData, const UINT32 size, const UINT32 size_cnt, char* pFile)
{
	if ((NULL == pBufData) || (0 >= size) || (0 >= size_cnt) || (NULL == pFile))
	{
		return false;
	}

	FILE* fp = fopen(pFile, "wb");
	if (NULL == fp)
	{
		return false;
	}

	fwrite(pBufData, 1, size * size_cnt, fp);
	fclose(fp);

	return true;
}

int GetRGBRawData(unsigned char *dst_image_buf)
{
	int iRet = 0;
	unsigned int uiFrameCnt = 0;
	
	enum v4l2_buf_type type;
	struct timeval tv;
	fd_set fds;
	
	unsigned short *pu16RawDataUser = NULL;
	camera_rgb_handle *pst_camera = &gstCameraHandle;

	static int tofrawdata_count = 0;
	tofrawdata_count++;

	FD_ZERO(&fds);
    FD_SET(pst_camera->videofd, &fds);

	/* wait for sensor capture data */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    iRet = select(pst_camera->videofd+1,&fds,NULL,NULL,&tv);
    if (iRet == -1){
        camera_err(" select error\n");
        return -1;
    }else if (iRet == 0){
        camera_err(" select timeout,end capture thread!\n");
        return -1;
    }	

    /* dqbuf */
    iRet = ioctl(pst_camera->videofd, VIDIOC_DQBUF, &pst_camera->stV4l2Buf);
    if (iRet == 0)
        camera_dbg("*****DQBUF[%d] FINISH*****\n",pst_camera->stV4l2Buf.index);
    else
        camera_err("****DQBUF FAIL*****\n");

#if 0

	capture_rgb_yuv ++;
	sprintf(source_data_path, "./capture/rgb_yuv%d.raw", capture_rgb_yuv);	
	if(capture_rgb_yuv > 37 && capture_rgb_yuv < 40)
	{
		if(capture_rgb_yuv % 2 == 0)
		{
			SaveBufRAW(pst_camera->buffers[pst_camera->stV4l2Buf.index].start[0], 1, RGB_WIDTH * RGB_HEIGHT*3/2,source_data_path); ///nv12  
			printf("---------------------------------saving rgb yuv");
		}
	}
#endif

	rga_yuv420sp_to_rgba(dst_image_buf,(unsigned char *)(pst_camera->buffers[pst_camera->stV4l2Buf.index].start[0]), 1920, 1080);

#if 0
	if (capture_rgb_yuv > 37 && capture_rgb_yuv < 40)
	{
		if(capture_rgb_yuv % 2 == 0)
		{			
			int iYuvFd;
			char acPicPath[64] = {0};
			snprintf(acPicPath, 64, "./capture/rgb_%d.dat", capture_rgb_yuv);
			iYuvFd = open(acPicPath, O_CREAT|O_RDWR);
			write(iYuvFd, dst_image_buf, 1920*1080*3);///rgb format
			close(iYuvFd);
			
			printf("+++++++++ dump rgb pic %s\n", acPicPath);
		}

	}
#endif

	/* qbuf */
    if (ioctl(pst_camera->videofd, VIDIOC_QBUF, &pst_camera->stV4l2Buf) == 0)
        camera_dbg("************QBUF[%d] FINISH**************\n", pst_camera->stV4l2Buf.index);
    else
        camera_err("*****QBUF FAIL*****\n");

	test_rgb_fps();
	return 0;	
}

