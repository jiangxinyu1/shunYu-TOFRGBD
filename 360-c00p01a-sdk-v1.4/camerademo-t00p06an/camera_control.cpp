#include <math.h>

#include "camera_control.h"
#include "tof_depth_process.h"

static camera_handle gstCameraHandle;
static char str_calib_name[128] = {0};////标定文件路径
static char str_removeINS_name[128] = {0};////标定文件路径

static unsigned long long get_tick_count()
{
	unsigned long long tick = 0;

	struct timeval tv;
	gettimeofday(&tv, 0);
	tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);

	return tick;
}

static void test_tof_fps(void)
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

        
        printf(">>>>>>>>>>>>>>>>>>>> TOf Frame Rate = %.2f\n", cur_framerate);
        start_time = get_tick_count();
        frames_num = 0;
    }
}

static const char *get_format_name(unsigned int format)
{
    if(format == V4L2_PIX_FMT_SRGGB12) 
        return "RGGB12";
    else
        return "unkown";
}

static int V4L2_StreamON()
{
	int iRet = 0;	
	enum v4l2_buf_type type;
	camera_handle *pst_camera = &gstCameraHandle;

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

static int V4L2_Init(void)
{
	char camera_path[16];
	struct v4l2_capability cap; 	 /* Query device capabilities */
	struct v4l2_format fmt; 		 /* try a format */
	struct v4l2_input inp;			 /* select the current video input */
	struct v4l2_streamparm parms;	 /* set streaming parameters */
	struct v4l2_requestbuffers req;  /* Initiate Memory Mapping or User Pointer I/O */
	struct v4l2_buffer buf; 		 /* Query the status of a buffer */
	camera_handle *pst_camera = &gstCameraHandle;

	int n_buffers = 0;
	int i = 0;

	/* default settings */
	memset(pst_camera, 0, sizeof(camera_handle));
	memset(camera_path, 0, sizeof(camera_path));

	pst_camera->camera_index 	= 0;///dev/video0 for tof cif,  video5 for old kernel tof isp
	pst_camera->win_width 		= WIDTH;
	pst_camera->win_height 		= MTP013_RAW_HEIGHT;
	pst_camera->photo_num 		= 5000000;//采集帧数 5000000	 
	pst_camera->pixelformat 	= V4L2_PIX_FMT_SRGGB12;

	camera_print("**********************************************************\n");
	camera_print("* 													   *\n");
	camera_print("* 			 this is sunny tof 360 camera.  		   *\n");
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
	if(V4L2_StreamON())
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

static void V4L2_UnInit(void)
{
	int i,j;

	enum v4l2_buf_type type;
	camera_handle *pst_camera = &gstCameraHandle;

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

static int read_calibdata_from_flash(void)
{
	int iRet = 0;
	int id[4]={0};

	/* read the sensor ID from i2c*/
	Control_Register_R_W(0xA097, 0, &id[0]);
	Control_Register_R_W(0xA098, 0, &id[1]);
	Control_Register_R_W(0xA099, 0, &id[2]);
	Control_Register_R_W(0xA09A, 0, &id[3]);
	printf("sensorID: %x-%x-%x-%x \n\r", id[0],id[1],id[2],id[3]);

	sprintf(str_calib_name, "./calib-%04x-%04x-%04x-%04x.bin",id[0],id[1],id[2],id[3]);
	sprintf(str_removeINS_name, "./RINS.bin");

	if(NULL == fopen(str_calib_name, "rb"))
	{
		printf("there is no calib_file ,read from flash\n\n");
		iRet = Read_Cailb_Data_FromFlash(str_calib_name, str_removeINS_name);
	}
	else
	{
		printf("find calib_file ok\n\n");
	}

	return iRet;
}

int OpenTofCamera(void)
{
	int iRet = 0;

	/* V4L2 Init */
	iRet = V4L2_Init();
	if (iRet)
	{
		printf("V4L2_Init failed\n");
		return -1;
	}

	/* I2C init */
	iRet = I2cCbInit();
	if (iRet)
	{
		printf("I2cCbInit failed\n");
		goto EXIT;
	}

	/* Read calibration data */	
	iRet = read_calibdata_from_flash();
	if (iRet)
	{
		printf("read_calibdata_from_flash failed\n");
		goto EXIT;
	}

	/* Tof depth SDK init */
	iRet = TofDepthSdkInit(str_calib_name);
	if (iRet)
	{
		printf("TofDepthSdkInit failed\n");
		goto EXIT;
	}

	return 0;

EXIT:
	V4L2_UnInit();

	return -1;
}

void CloseTofCamera(void)
{
	TofDepthSdkUnInit();
	V4L2_UnInit();
	I2cCbUninit();
}

int GetTofRawData(TOF_RAW_DATA_CB_S *pstTofRawDataCb)
{
	int iRet = 0;
	unsigned int uiFrameCnt = 0;
	
	enum v4l2_buf_type type;
	struct timeval tv;
	fd_set fds;
	
	unsigned short *pu16RawDataUser = NULL;
	camera_handle *pst_camera = &gstCameraHandle;

	static int tofrawdata_count = 0;
	tofrawdata_count++;

	if (!pstTofRawDataCb )
	{
		printf("NULL ptr!\n");
		return -1;
	}

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
	
	pstTofRawDataCb->stRawData.pRaw = (unsigned char*)pst_camera->buffers[pst_camera->stV4l2Buf.index].start[0];

	pstTofRawDataCb->stRawData.nRawLen = RAW_SIZE;
	pstTofRawDataCb->uiFrameCnt = tofrawdata_count;///自己填一个count进去，方便capture数据命名

	/* qbuf */
    if (ioctl(pst_camera->videofd, VIDIOC_QBUF, &pst_camera->stV4l2Buf) == 0)
        camera_dbg("************QBUF[%d] FINISH**************\n", pst_camera->stV4l2Buf.index);
    else
        camera_err("*****QBUF FAIL*****\n");

	test_tof_fps();
	return 0;	
}

int GetTofRawDataFormLocal(TOF_RAW_DATA_CB_S *pstTofRawDataCb, unsigned int uiNum, unsigned int uiStartIndex)
{
	int iReadLen = 0;
	static int iCnt = 0;
	
	char acRawPath[64] = {0};
	struct stat statbuf = {0};

	unsigned char *pu8RawDataUser;
	FILE *pFileRaw = NULL;

	if (iCnt >= uiNum)
	{
		printf("local raw data process done, process exit\n");
		sleep(1);
		exit(0);
	}

	if (!pstTofRawDataCb)
	{
		printf("NULL ptr!\n");
		return -1;
	}

	pu8RawDataUser = pstTofRawDataCb->stRawData.pRaw;

	snprintf(acRawPath, 64, "./%d-raw.data", iCnt + uiStartIndex);
	pFileRaw = fopen(acRawPath, "rb");
	if (!pFileRaw)
	{
		printf("fopen %s failed\n", acRawPath);
		return -1;
	}

	stat(acRawPath, &statbuf);
	iReadLen = fread(pu8RawDataUser, 1, statbuf.st_size, pFileRaw);
	if (iReadLen == RAW_SIZE)
	{
		printf("read %s successed, read size = %u\n", acRawPath, statbuf.st_size);
	}
	else
	{
		printf("read %s failed, read size = %u\n", acRawPath, statbuf.st_size);
		fclose(pFileRaw);
		return -1;
	}

	pstTofRawDataCb->uiFrameCnt = iCnt + uiStartIndex;

	iCnt++;
	fclose(pFileRaw);

	return 0;
}

