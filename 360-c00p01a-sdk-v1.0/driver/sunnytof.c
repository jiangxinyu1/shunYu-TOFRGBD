// SPDX-License-Identifier: GPL-2.0
/*
 * sunnytof driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)

#define TOF_WIDTH  224 
#define TOF_HEIGHT    2193  ///2193 , 17phase

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

/* 45Mhz * 4 Binning */
#define SUNNYTOF_PIXEL_RATE		(45 * 1000 * 1000 * 4)
#define SUNNYTOF_XVCLK_FREQ		24000000

#define SUNNYTOF_VTS_MAX		0x7fff  //TODO: Check this value


#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define CHIP_2877_ID				0x2877
#define CHIP_2381_ID				0x2381
#define SUNNYTOF_REG_CHIP_ID		0xA0A4

#define SUNNYTOF_REG_CTRL_MODE		0x9400
#define SUNNYTOF_MODE_SW_STANDBY	0x0000
#define SUNNYTOF_MODE_STREAMING		0x0001

#define SUNNYTOF_REG_EXPOSURE		0x3500
#define	SUNNYTOF_EXPOSURE_MIN		4
#define	SUNNYTOF_EXPOSURE_STEP		1
#define SUNNYTOF_VTS_MAX		0x7fff

#define SUNNYTOF_REG_ANALOG_GAIN	0x3509
#define	ANALOG_GAIN_MIN			0x10
#define	ANALOG_GAIN_MAX			0xf8
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x10

#define SUNNYTOF_REG_DIGI_GAIN_H		0x350a
#define SUNNYTOF_REG_DIGI_GAIN_L		0x350b
#define SUNNYTOF_DIGI_GAIN_L_MASK		0x3f
#define SUNNYTOF_DIGI_GAIN_H_SHIFT	6
#define SUNNYTOF_DIGI_GAIN_MIN		0
#define SUNNYTOF_DIGI_GAIN_MAX		(0x4000 - 1)
#define SUNNYTOF_DIGI_GAIN_STEP		1
#define SUNNYTOF_DIGI_GAIN_DEFAULT	1024

#define SUNNYTOF_REG_TEST_PATTERN	0x4503
#define	SUNNYTOF_TEST_PATTERN_ENABLE	0x80
#define	SUNNYTOF_TEST_PATTERN_DISABLE	0x0

#define SUNNYTOF_REG_VTS		0x380e

#define REG_NULL			0xFFFF

#define SUNNYTOF_REG_VALUE_08BIT	1
#define SUNNYTOF_REG_VALUE_16BIT	2
#define SUNNYTOF_REG_VALUE_24BIT	3

//#define SUNNYTOF_LANES			2
//#define SUNNYTOF_BITS_PER_SAMPLE	10

#define I2C_M_WR			0
#define I2C_MSG_MAX			300
#define I2C_DATA_MAX			(I2C_MSG_MAX * 3)

#define SUNNYTOF_VTS_MAX		0x7fff  //TODO: Check this value


#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"



#define SUNNYTOF_NAME			"irs2381"

static const char * const sunnytof_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SUNNYTOF_NUM_SUPPLIES ARRAY_SIZE(sunnytof_supply_names)

struct regval {
	u16 addr;
	u16 val;
};

struct sunnytof_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

struct sunnytof {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SUNNYTOF_NUM_SUPPLIES];
	
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct sunnytof_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

#define to_sunnytof(sd) container_of(sd, struct sunnytof, subdev)

static void __sunnytof_power_off(struct sunnytof *sunnytof);
static int __sunnytof_power_on(struct sunnytof *sunnytof);

///t00p06an-200 ok
static const struct regval irs2381_224x2193_10_regs_noSSC[] = {/////129ÐÐraw  17phase  129*17 = 2193  mid {0x92F0, 0x00D4},  for t00p06an
{0xA007, 0x1313},
{0xA008, 0x1313},
{0xA00C, 0x0135},
{0xA039, 0x1AA1},
{0xA03A, 0xAAAB},
{0xA03B, 0x000A},
{0xA03C, 0x0000},
{0xA03D, 0x03C0},
{0xA03E, 0x0000},
{0xA03F, 0x0017},
{0x9000, 0x1E1E},
{0x9002, 0x0FB0},
{0x9004, 0x0FB0},
{0x9006, 0x0FB0},
{0x9008, 0x0FB0},
{0x900A, 0x0BC4},
{0x900C, 0x0BC4},
{0x900E, 0x0BC4},
{0x9010, 0x0BC4},
{0x9012, 0x6738},
{0x9014, 0x6738},
{0x9016, 0x6738},
{0x9018, 0x6738},
{0x901A, 0x5D6A},
{0x901C, 0x5D6A},
{0x901E, 0x5D6A},
{0x9020, 0x5D6A},
{0x9080, 0x1E1E},
{0x9082, 0x10A2},
{0x9083, 0x00A2},
{0x9084, 0x0000},
{0x9085, 0x0FB0},
{0x9087, 0x4100},
{0x9088, 0x0000},
{0x9089, 0x0000},
{0x908A, 0x0FB0},
{0x908C, 0x4100},
{0x908D, 0x0000},
{0x908E, 0x0003},
{0x908F, 0x0FB0},
{0x9091, 0x4100},
{0x9092, 0x0000},
{0x9093, 0x0006},
{0x9094, 0x0FB0},
{0x9096, 0x4100},
{0x9097, 0x0000},
{0x9098, 0x0009},
{0x9099, 0x0BC4},
{0x909B, 0x5102},
{0x909C, 0x0002},
{0x909D, 0x0000},
{0x909E, 0x0BC4},
{0x90A0, 0x5102},
{0x90A1, 0x0002},
{0x90A2, 0x0003},
{0x90A3, 0x0BC4},
{0x90A5, 0x5102},
{0x90A6, 0x0002},
{0x90A7, 0x0006},
{0x90A8, 0x0BC4},
{0x90AA, 0x5102},
{0x90AB, 0x0002},
{0x90AC, 0x0009},
{0x90AD, 0x6738},
{0x90AF, 0x4100},
{0x90B0, 0x0000},
{0x90B1, 0x0000},
{0x90B2, 0x6738},
{0x90B4, 0x4100},
{0x90B5, 0x0000},
{0x90B6, 0x0003},
{0x90B7, 0x6738},
{0x90B9, 0x4100},
{0x90BA, 0x0000},
{0x90BB, 0x0006},
{0x90BC, 0x6738},
{0x90BE, 0x4100},
{0x90BF, 0x0000},
{0x90C0, 0x0009},
{0x90C1, 0x5D6A},
{0x90C3, 0x5102},
{0x90C4, 0x0002},
{0x90C5, 0x0000},
{0x90C6, 0x5D6A},
{0x90C8, 0x5102},
{0x90C9, 0x0002},
{0x90CA, 0x0003},
{0x90CB, 0x5D6A},
{0x90CD, 0x5102},
{0x90CE, 0x0002},
{0x90CF, 0x0006},
{0x90D0, 0x5D6A},
{0x90D2, 0x5102},
{0x90D3, 0x0002},
{0x90D4, 0xC009},
{0x91C0, 0x0592},
{0x91C1, 0xDF00},
{0x91C2, 0x9516},
{0x91C3, 0x0488},
{0x91C4, 0x0008},
{0x91C5, 0x0020},
{0x91C6, 0x8008},
{0x91CF, 0x0011},
{0x91D3, 0x1250},
{0x91DB, 0x0008},
{0x91EA, 0x16A1},
{0x91EB, 0x1EB8},
{0x91EC, 0x0005},
{0x91ED, 0x0C01},
{0x91EE, 0x0000},
{0x91EF, 0x04A0},
{0x91F0, 0x0000},
{0x91F1, 0x0000},
{0x91F2, 0x1AA1},
{0x91F3, 0xD70A},
{0x91F4, 0x0003},
{0x91F5, 0x1F01},
{0x91F6, 0x0000},
{0x91F7, 0x0360},
{0x91F8, 0x0000},
{0x91F9, 0x0010},
{0x9220, 0x000B},
{0x9221, 0x0FC0},
{0x9229, 0x000B},
{0x922A, 0x0FC0},
{0x9244, 0x000B},
{0x9245, 0x0F80},
{0x924D, 0x000B},
{0x924E, 0x0F80},
{0x9268, 0x0202},
{0x926D, 0x0202},
{0x9278, 0x0602},
{0x927D, 0x0902},
{0x9288, 0x0248},
{0x9289, 0x024C},
{0x928A, 0x0249},
{0x928B, 0x0248},
{0x928C, 0x0248},
{0x928D, 0x0248},
{0x928E, 0x0248},
{0x928F, 0x0248},
{0x9290, 0x0248},
{0x9291, 0x0248},
{0x9292, 0x024B},
{0x9293, 0x024B},
{0x9294, 0x0122},
{0x9295, 0x012D},
{0x9296, 0x0117},
{0x9297, 0x012D},
{0x9298, 0x012D},
{0x9299, 0x0117},
{0x929A, 0x012D},
{0x929B, 0x012D},
{0x929C, 0x0080},
{0x929D, 0x0080},
{0x929E, 0x00D7},
{0x929F, 0x0083},
{0x92A0, 0x0080},
{0x92A1, 0x0080},
{0x92A2, 0x0080},
{0x92A3, 0x0080},
{0x92A4, 0x0040},
{0x92A5, 0x0040},
{0x92A6, 0x0040},
{0x92A7, 0x0040},
{0x92A8, 0x0040},
{0x92A9, 0x0040},
{0x92AA, 0x0040},
{0x92AB, 0x0040},
{0x92AC, 0x0040},
{0x92AD, 0x0040},
{0x92AE, 0x0040},
{0x92AF, 0x0040},
{0x92B0, 0x0040},
{0x92B1, 0x0040},
{0x92B2, 0x0040},
{0x92B3, 0x0040},
{0x92B4, 0x0040},
{0x92B5, 0x0040},
{0x92B6, 0x0040},
{0x92B7, 0x0040},
{0x92B8, 0x0040},
{0x92B9, 0x0040},
{0x92BA, 0x0040},
{0x92BB, 0x0040},
{0x92BC, 0x0040},
{0x92BD, 0x0040},
{0x92BE, 0x0040},
{0x92BF, 0x0040},
{0x92C0, 0x0040},
{0x92C1, 0x0040},
{0x92C2, 0x0040},
{0x92C3, 0x0040},
{0x92C4, 0x2110},
{0x92C5, 0x4332},
{0x92C6, 0x6554},
{0x92C7, 0x8776},
{0x92C8, 0x3221},
{0x92C9, 0x5443},
{0x92CA, 0x6D61},
{0x92CB, 0x0078},
{0x92CC, 0x2319},
{0x92CD, 0x342B},
{0x92CE, 0x433C},
{0x92CF, 0x004B},
{0x92D0, 0x1610},
{0x92D1, 0x211B},
{0x92D2, 0x2B26},
{0x92D3, 0x002F},
{0x92D4, 0x1C14},
{0x92D5, 0x2B24},
{0x92D6, 0x3832},
{0x92D7, 0x003F},
{0x92D8, 0x261B},
{0x92D9, 0x322C},
{0x92DA, 0x3D38},
{0x92DB, 0x0042},
{0x92DC, 0x110D},
{0x92DD, 0x1A16},
{0x92DE, 0x231E},
{0x92DF, 0x2C27},
{0x92E0, 0x3631},
{0x92E1, 0x003B},
{0x92E2, 0x0000},
{0x92E3, 0x08FF},
{0x92E4, 0x087F},
{0x92E5, 0x0855},
{0x92E6, 0x082A},
{0x92E7, 0x0001},
{0x92E8, 0x0100},
{0x92E9, 0x0300},
{0x92EA, 0x0000},
{0x92EB, 0x0000},
{0x92EC, 0x0000},
{0x92ED, 0x0000},
{0x92EE, 0x0000},
{0x92EF, 0x0000},
{0x92F0, 0x00D4},
{0x92F1, 0x228E},
{0x9401, 0x0002},
{0x8423, 0x00A2},
{0xFFFF, 0x0000},
};

static const struct sunnytof_mode supported_modes[] = {
	{
		.width = TOF_WIDTH,
		.height = TOF_HEIGHT,
		.max_fps = {
			.numerator = 10000,
			.denominator = 50000,
		},
		.exp_def = 0x0600,
		.hts_def = 0x12c0,
		.vts_def = 0x08a0,
		.reg_list = irs2381_224x2193_10_regs_noSSC,
	},
};

#define SUNNYTOF_LINK_FREQ_800MHZ		400000000

static const s64 link_freq_menu_items[] = {
	SUNNYTOF_LINK_FREQ_800MHZ
};

static const char * const sunnytof_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int sunnytof_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}


/* Read registers up to 4 at a time */
static int sunnytof_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			   u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}
static int sunnytof_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i,j;
	int ret = 0;
	u32 read_reg = 0;
	struct device *dev = &client->dev;
	dev_err(dev, "Write array enter\n");
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++){
		ret = sunnytof_write_reg(client, regs[i].addr,
					SUNNYTOF_REG_VALUE_16BIT,
					regs[i].val);
		if(ret)
			printk(KERN_ERR "Write reg [0x%x] =[0x%x] failed,ret=%d\n",regs[i].addr, regs[i].val,ret);
	}

	for(j = 0; j>50;j++)
	{
		dev_info(dev,"reg_len = %d regs[%d] = ox%x",i,j,regs[j].val);
	}
	dev_err(dev, "Write array ret=%d\n",ret);
	sunnytof_read_reg(client,0x9002, SUNNYTOF_REG_VALUE_16BIT ,&read_reg);
	dev_info(dev, "read_reg 0x9002 -> 0x%4x",read_reg);
	
	sunnytof_read_reg(client,0xA03F, SUNNYTOF_REG_VALUE_16BIT ,&read_reg);
	dev_info(dev, "read_reg 0xA03F -> 0x%4x",read_reg);
	
	sunnytof_read_reg(client,0x9400, SUNNYTOF_REG_VALUE_16BIT ,&read_reg);
	dev_info(dev, "read_reg 0x9400 -> 0x%4x",read_reg);

	return ret;
}

static int sunnytof_get_reso_dist(const struct sunnytof_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sunnytof_mode *
sunnytof_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = sunnytof_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int sunnytof_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct device *dev = sd->dev;
	struct sunnytof *sunnytof = to_sunnytof(sd);
	const struct sunnytof_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&sunnytof->mutex);

	mode = sunnytof_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SBGGR12_1X12;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sunnytof->mutex);
		return -ENOTTY;
#endif
	} else {
		sunnytof->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sunnytof->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sunnytof->vblank, vblank_def,
					 SUNNYTOF_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&sunnytof->mutex);
	dev_err(dev,"sunnytof_set_fmt,%d,%d,%d,%d \n",
			fmt->format.width, fmt->format.height,
			fmt->format.code, fmt->format.field);
	return 0;
}

static int sunnytof_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct device *dev = sd->dev;
	struct sunnytof *sunnytof = to_sunnytof(sd);
	const struct sunnytof_mode *mode = sunnytof->cur_mode;
	dev_err(dev,"sunnytof_get_fmt entry\n");

	mutex_lock(&sunnytof->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		dev_err(dev,"sunnytof_get_fmt V4L2_SUBDEV_FORMAT_TRY\n");
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sunnytof->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR12_1X12;
		fmt->format.field = V4L2_FIELD_NONE;
		dev_err(dev,"sunnytof_get_fmt,%d,%d,%d,%d \n",
			fmt->format.width, fmt->format.height,
			fmt->format.code, fmt->format.field);
	}
	mutex_unlock(&sunnytof->mutex);

	return 0;
}

static int sunnytof_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SBGGR12_1X12;

	return 0;
}

static int sunnytof_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct device *dev = sd->dev;
	dev_err(dev,"sunnytof_enum_frame_sizes entry\n");
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR12_1X12)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	dev_err(dev, "sunnytof_enum_frame_sizes,min_width:%d, max_width:%d,max_height:%d, min_height:%d\n",
		fse->min_width, fse->max_width, fse->max_height, fse->min_height);			   
	return 0;
}

static int sunnytof_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sunnytof *sunnytof = to_sunnytof(sd);
	const struct sunnytof_mode *mode = sunnytof->cur_mode;

	mutex_lock(&sunnytof->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&sunnytof->mutex);

	return 0;
}

static int __sunnytof_start_stream(struct sunnytof *sunnytof)
{
	int ret;

	struct device *dev = &sunnytof->client->dev;

	dev_err(dev, "Now start streaming\n");

	__sunnytof_power_on(sunnytof);
	ret = sunnytof_write_array(sunnytof->client, sunnytof->cur_mode->reg_list);
	if (ret)
		return ret;

	dev_err(dev, "start stream, write array ret=:%d\n", ret);
	
	/* In case these controls are set before streaming */
	mutex_unlock(&sunnytof->mutex);
	ret = v4l2_ctrl_handler_setup(&sunnytof->ctrl_handler);
	mutex_lock(&sunnytof->mutex);
	if (ret)
	{
		dev_err(dev, "start stream, mutex_lock error, ret=%d\n", ret);
		return ret;
	}

	ret = sunnytof_write_reg(sunnytof->client, SUNNYTOF_REG_CTRL_MODE,
				SUNNYTOF_REG_VALUE_16BIT, SUNNYTOF_MODE_STREAMING);
	dev_err(dev, "start stream, return :%d\n", ret);
	return ret;
}

static int __sunnytof_stop_stream(struct sunnytof *sunnytof)
{
	int ret;	
	struct device *dev = &sunnytof->client->dev;

	dev_err(dev, "stop stream\n");

	ret = sunnytof_write_reg(sunnytof->client,
				 SUNNYTOF_REG_CTRL_MODE,
				 SUNNYTOF_REG_VALUE_16BIT,
				 SUNNYTOF_MODE_SW_STANDBY);				 
	
	__sunnytof_power_off(sunnytof);	
	return ret;
}

static int sunnytof_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sunnytof *sunnytof = to_sunnytof(sd);
	struct i2c_client *client = sunnytof->client;
	int ret = 0;

	mutex_lock(&sunnytof->mutex);
	on = !!on;
	if (on == sunnytof->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sunnytof_start_stream(sunnytof);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sunnytof_stop_stream(sunnytof);
		pm_runtime_put(&client->dev);
	}

	sunnytof->streaming = on;

unlock_and_return:
	mutex_unlock(&sunnytof->mutex);

	return ret;
}

static int sunnytof_s_power(struct v4l2_subdev *sd, int on)
{
	struct sunnytof *sunnytof = to_sunnytof(sd);
	struct i2c_client *client = sunnytof->client;
	int ret = 0;

	mutex_lock(&sunnytof->mutex);

	/* If the power state is not modified - no work to do. */
	if (sunnytof->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		sunnytof->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sunnytof->power_on = false;
	}

	unlock_and_return:
		mutex_unlock(&sunnytof->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 sunnytof_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, SUNNYTOF_XVCLK_FREQ / 1000 / 1000);
}

static int __sunnytof_power_on(struct sunnytof *sunnytof)
{
	int ret;
	u32 delay_us;
	struct device *dev = &sunnytof->client->dev;

    if (!IS_ERR_OR_NULL(sunnytof->pins_default)) {
            ret = pinctrl_select_state(sunnytof->pinctrl,
                                       sunnytof->pins_default);
            if (ret < 0)
                    dev_err(dev, "could not set pins\n");
    }

    ret = clk_set_rate(sunnytof->xvclk, SUNNYTOF_XVCLK_FREQ);
    if (ret < 0) {
            dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
            return ret;
    }

    if (clk_get_rate(sunnytof->xvclk) != SUNNYTOF_XVCLK_FREQ)
            dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

    ret = clk_prepare_enable(sunnytof->xvclk);
    if (ret < 0) {
            dev_err(dev, "Failed to enable xvclk\n");
            return ret;
    }

	if (!IS_ERR(sunnytof->reset_gpio))
		gpiod_set_value_cansleep(sunnytof->reset_gpio, 0);

	ret = regulator_bulk_enable(SUNNYTOF_NUM_SUPPLIES, sunnytof->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sunnytof->reset_gpio))
		gpiod_set_value_cansleep(sunnytof->reset_gpio, 1);

	if (!IS_ERR(sunnytof->pwdn_gpio))
		gpiod_set_value_cansleep(sunnytof->pwdn_gpio, 1);


	sunnytof->reset_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if(!IS_ERR(sunnytof->reset_gpio))
		dev_info(dev,"power on reset 0");	
	
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = sunnytof_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	dev_info(dev,"power on");	

	return 0;

disable_clk:
	clk_disable_unprepare(sunnytof->xvclk);

	return ret;
}

static void __sunnytof_power_off(struct sunnytof *sunnytof)
{
	if (!IS_ERR(sunnytof->pwdn_gpio))
		gpiod_set_value_cansleep(sunnytof->pwdn_gpio, 0);

	clk_disable_unprepare(sunnytof->xvclk);

	if (!IS_ERR(sunnytof->reset_gpio))
		gpiod_set_value_cansleep(sunnytof->reset_gpio, 1);

	regulator_bulk_disable(SUNNYTOF_NUM_SUPPLIES, sunnytof->supplies);
}

static int sunnytof_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sunnytof *sunnytof = to_sunnytof(sd);

	return __sunnytof_power_on(sunnytof);
}

static int sunnytof_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sunnytof *sunnytof = to_sunnytof(sd);

	__sunnytof_power_off(sunnytof);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sunnytof_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sunnytof *sunnytof = to_sunnytof(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sunnytof_mode *def_mode = &supported_modes[0];

	mutex_lock(&sunnytof->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR12_1X12;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sunnytof->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops sunnytof_pm_ops = {
	SET_RUNTIME_PM_OPS(sunnytof_runtime_suspend,
			   sunnytof_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sunnytof_internal_ops = {
	.open = sunnytof_open,
};
#endif
static const struct v4l2_subdev_core_ops sunnytof_core_ops = {
	.s_power = sunnytof_s_power,
//	.ioctl = sunnytof_ioctl,
#ifdef CONFIG_COMPAT
//	.compat_ioctl32 = sunnytof_compat_ioctl32,
#endif
};
static const struct v4l2_subdev_video_ops sunnytof_video_ops = {
	.s_stream = sunnytof_s_stream,
	.g_frame_interval = sunnytof_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sunnytof_pad_ops = {
	.enum_mbus_code = sunnytof_enum_mbus_code,
	.enum_frame_size = sunnytof_enum_frame_sizes,
	.get_fmt = sunnytof_get_fmt,
	.set_fmt = sunnytof_set_fmt,
};

static const struct v4l2_subdev_ops sunnytof_subdev_ops = {
	.core	= &sunnytof_core_ops,
	.video	= &sunnytof_video_ops,
	.pad	= &sunnytof_pad_ops,
};

static int sunnytof_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sunnytof *sunnytof = container_of(ctrl->handler,
					     struct sunnytof, ctrl_handler);
	struct i2c_client *client = sunnytof->client;
	s64 max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sunnytof->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(sunnytof->exposure,
					 sunnytof->exposure->minimum, max,
					 sunnytof->exposure->step,
					 sunnytof->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {

	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sunnytof_ctrl_ops = {
	.s_ctrl = sunnytof_set_ctrl,
};

static int sunnytof_initialize_controls(struct sunnytof *sunnytof)
{
	const struct sunnytof_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	//s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &sunnytof->ctrl_handler;
	mode = sunnytof->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &sunnytof->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, SUNNYTOF_PIXEL_RATE, 1, SUNNYTOF_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	sunnytof->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (sunnytof->hblank)
		sunnytof->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (handler->error) {
		ret = handler->error;
		dev_err(&sunnytof->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sunnytof->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sunnytof_check_sensor_id(struct sunnytof *sunnytof,
				  struct i2c_client *client)
{
	struct device *dev = &sunnytof->client->dev;
	u32 id = 0;
	int ret;

	ret = sunnytof_read_reg(client, SUNNYTOF_REG_CHIP_ID,
			      SUNNYTOF_REG_VALUE_16BIT, &id);
				  
	if( (id != CHIP_2877_ID) && (id != CHIP_2381_ID) ) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected %06x sensor\n", id);

	return 0;
}

static int sunnytof_configure_regulators(struct sunnytof *sunnytof)
{
	int i;

	for (i = 0; i < SUNNYTOF_NUM_SUPPLIES; i++)
		sunnytof->supplies[i].supply = sunnytof_supply_names[i];

	return devm_regulator_bulk_get(&sunnytof->client->dev,
				       SUNNYTOF_NUM_SUPPLIES,
				       sunnytof->supplies);
}

static int sunnytof_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sunnytof *sunnytof;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	sunnytof = devm_kzalloc(dev, sizeof(*sunnytof), GFP_KERNEL);
	
	dev_info(dev, "devm_kzalloc");
	if (!sunnytof)
	{
		dev_info(dev, "kzalloc feil");
		return -ENOMEM;
	}
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sunnytof->module_index);
	dev_info(dev, "of_property_read_u32 ret = %d  module_index = %d",ret,sunnytof->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sunnytof->module_facing);
	dev_info(dev, "of_property_read_u32 ret = %d module_facing = %s",ret,sunnytof->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sunnytof->module_name);
	dev_info(dev, "of_property_read_u32 ret = %d module_name = %s",ret,sunnytof->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sunnytof->len_name);
	dev_info(dev, "of_property_read_u32 ret = %d len_name = %s",ret,sunnytof->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sunnytof->client = client;
	sunnytof->cur_mode = &supported_modes[0];////

	dev_info(dev, "supported_modes");
	sunnytof->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sunnytof->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sunnytof->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	dev_info(dev, "reset_gpiogpio_get = %d ",GPIOD_OUT_LOW);
	if (IS_ERR(sunnytof->reset_gpio)) {
		dev_warn(dev, "Failed to get reset-gpios\n");
	}

	sunnytof->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	dev_info(dev, "reset_gpiogpio_get = %d ",GPIOD_OUT_LOW);
	if (IS_ERR(sunnytof->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

        sunnytof->pinctrl = devm_pinctrl_get(dev);
        if (!IS_ERR(sunnytof->pinctrl)) {
                sunnytof->pins_default =
                        pinctrl_lookup_state(sunnytof->pinctrl,
                                             OF_CAMERA_PINCTRL_STATE_DEFAULT);
                if (IS_ERR(sunnytof->pins_default))
                        dev_err(dev, "could not get default pinstate\n");

                sunnytof->pins_sleep =
                        pinctrl_lookup_state(sunnytof->pinctrl,
                                             OF_CAMERA_PINCTRL_STATE_SLEEP);
                if (IS_ERR(sunnytof->pins_sleep))
                        dev_err(dev, "could not get sleep pinstate\n");
        } else {
                dev_err(dev, "no pinctrl\n");
        }

	ret = sunnytof_configure_regulators(sunnytof);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sunnytof->mutex);

	sd = &sunnytof->subdev;
	v4l2_i2c_subdev_init(sd, client, &sunnytof_subdev_ops);
	ret = sunnytof_initialize_controls(sunnytof);
	if (ret)
		goto err_destroy_mutex;

	ret = __sunnytof_power_on(sunnytof);
	if (ret)
		goto err_free_handler;

	ret = sunnytof_check_sensor_id(sunnytof, client);
	if (ret)
	{
		dev_info(dev,"goto err_power_off");
		goto err_power_off;
	}
	else 
		dev_info(dev,"check contiune");
	dev_info(dev,"check end -->");

	printk(KERN_ERR "write arry start\n");
        ret = sunnytof_write_array(sunnytof->client, sunnytof->cur_mode->reg_list);
        if (ret){
		printk(KERN_INFO "write array failed\n");
                //return ret;
	}

	printk(KERN_ERR "Write array done\n");

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sunnytof_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	sunnytof->pad.flags = MEDIA_PAD_FL_SOURCE;
//	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
//	ret = media_entity_init(&sd->entity, 1, &sunnytof->pad, 0);
	ret = media_entity_pads_init(&sd->entity, 1, &sunnytof->pad);
	if (ret < 0)
	{
		dev_info(dev,"goto err_power_off2");
		goto err_power_off;
	}
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sunnytof->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sunnytof->module_index, facing,
		 SUNNYTOF_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	dev_info(dev,"end probe");
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	dev_info(dev, "power off end");
	__sunnytof_power_off(sunnytof);
err_free_handler:
	v4l2_ctrl_handler_free(&sunnytof->ctrl_handler);
	dev_info(dev, "free handle end");
err_destroy_mutex:
	mutex_destroy(&sunnytof->mutex);
	dev_info(dev, "destroy mutex");
	return ret;
}

static int sunnytof_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sunnytof *sunnytof = to_sunnytof(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sunnytof->ctrl_handler);
	mutex_destroy(&sunnytof->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sunnytof_power_off(sunnytof);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sunnytof_of_match[] = {
	{ .compatible = "pmd,irs2381" },
	{},
};
MODULE_DEVICE_TABLE(of, sunnytof_of_match);
#endif

static const struct i2c_device_id sunnytof_match_id[] = {
	{ "pmd,irs2381", 0 },
	{ },
};

static struct i2c_driver sunnytof_i2c_driver = {
	.driver = {
		.name = SUNNYTOF_NAME,
		.pm = &sunnytof_pm_ops,
		.of_match_table = of_match_ptr(sunnytof_of_match),
	},
	.probe		= &sunnytof_probe,
	.remove		= &sunnytof_remove,
	.id_table	= sunnytof_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sunnytof_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sunnytof_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("sunny 360 tof sensor driver");
MODULE_LICENSE("GPL v2");

