/*
* Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
* Copyright 2018 NXP
* Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
* Copyright 2022 Semiconductor Components Industries, LLC ("onsemi").
*/
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include "vvsensor.h"
#include "ar0830_regs_4k.h"


#define AR0830_VOLTAGE_ANALOG			2800000
#define AR0830_VOLTAGE_DIGITAL_CORE		1500000
#define AR0830_VOLTAGE_DIGITAL_IO		1800000

#define AR0830_XCLK_MIN 6000000
#define AR0830_XCLK_MAX 48000000

#define AR0830_CHIP_ID                  0x553
#define AR0830_CHIP_VERSION_REG 		0x0016
#define AR0830_RESET_REG                0x0103

#define AR0830_SENS_PAD_SOURCE	0
#define AR0830_SENS_PADS_NUM	1

#define AR0830_RESERVE_ID 0X2770
#define DCG_CONVERSION_GAIN 11

#define AR0830_REG_CTRL_MODE		0x0100
#define AR0830_MODE_SW_STANDBY	0x00
#define AR0830_MODE_STREAMING		0x01

#define AR0830_GAIN_MIN		0x00
#define AR0830_GAIN_MAX		127
#define AR0830_REG_GAIN		0x3062
#define AR0830_REG_GAIN2	0x3062

#define client_to_ar0830(client)\
	container_of(i2c_get_clientdata(client), struct ar0830, subdev)

struct ar0830_capture_properties {
	__u64 max_lane_frequency;
	__u64 max_pixel_frequency;
	__u64 max_data_rate;
};

struct ar0830 {
	struct i2c_client *i2c_client;
	struct regulator *io_regulator;
	struct regulator *core_regulator;
	struct regulator *analog_regulator;
	unsigned int pwn_gpio;
	unsigned int rst_gpio;
	unsigned int mclk;
	unsigned int mclk_source;
	struct clk *sensor_clk;
	unsigned int csi_id;
	struct ar0830_capture_properties ocp;

	struct v4l2_subdev subdev;
	struct media_pad pads[AR0830_SENS_PADS_NUM];

	struct v4l2_mbus_framefmt format;
	vvcam_mode_info_t cur_mode;
	sensor_blc_t blc;
	sensor_white_balance_t wb;
	struct mutex lock;
	u32 stream_status;
	u32 resume_status;
	vvcam_lens_t focus_lens;
};
struct gain_table{
	unsigned int code;
	unsigned int times;
};
unsigned int GAIN_TABLE_SIZE = 128;
struct gain_table ar0830_gain_table[] = {
{0	,1024     },
{1	,1069     },
{2	,1116     },
{3	,1166     },
{4	,1217     },
{5	,1271     },
{6	,1327     },
{7	,1385     },
{8	,1441     },
{9	,1505     },
{10	,1571     },
{11	,1641     },
{12	,1713     },
{13	,1789     },
{14	,1868     },
{15	,1950     },
{16	,2038     },
{17	,2127     },
{18	,2221     },
{19	,2319     },
{20	,2422     },
{21	,2529     },
{22	,2640     },
{23	,2757     },
{24	,2880     },
{25	,3007     },
{26	,3140     },
{27	,3278     },
{28	,3423     },
{29	,3574     },
{30	,3732     },
{31	,3896     },
{32	,4064     },
{33	,4243     },
{34	,4430     },
{35	,4626     },
{36	,4830     },
{37	,5043     },
{38	,5265     },
{39	,5498     },
{40	,5758     },
{41	,6012     },
{42	,6277     },
{43	,6554     },
{44	,6843     },
{45	,7145     },
{46	,7460     },
{47	,7789     },
{48	,8109     },
{49	,8467     },
{50	,8840     },
{51	,9230     },
{52	,9638     },
{53	,10063    },
{54	,10507    },
{55	,10970    },
{56	,11472    },
{57	,11978    },
{58	,12507    },
{59	,13059    },
{60	,13635    },
{61	,14236    },
{62	,14864    },
{63	,15520    },
{64	,16182    },
{65	,16896    },
{66	,17642    },
{67	,18420    },
{68	,19233    },
{69	,20081    },
{70	,20967    },
{71	,21892    },
{72	,22861    },
{73	,23870    },
{74	,24923    },
{75	,26022    },
{76	,27170    },
{77	,28369    },
{78	,29621    },
{79	,30927    },
{80	,32357    },
{81	,33785    },
{82	,35275    },
{83	,36831    },
{84	,38456    },
{85	,40153    },
{86	,41925    },
{87	,43774    },
{88	,45722    },
{89	,47739    },
{90	,49845    },
{91	,52044    },
{92	,54341    },
{93	,56738    },
{94	,59241    },
{95	,61855    },
{96	,64538    },
{97	,67386    },
{98	,70359    },
{99	,73463    },
{100,	76704 },
{101,	80088 },
{102,	83621 },
{103,	87311 },
{104,	91180 },
{105,	95203 },
{106,	99403 },
{107,	103789},
{108,	108368},
{109,	113149},
{110,	118141},
{111,	123353},
{112,	128901},
{113,	134588},
{114,	140525},
{115,	146725},
{116,	153199},
{117,	159958},
{118,	167015},
{119,	174383},
{120,	182008},
{121,	190038},
{122,	198423},
{123,	207177},
{124,	216317},
{125,	225861},
{126,	235826},
{127,	246230},
};

static struct vvcam_mode_info_s par0830_mode_info[] = {
	{
		.index          = 0,
		.size           = {
			.bounds_width  = 3840,
			.bounds_height = 2160,
			.top           = 0,
			.left          = 0,
			.width         = 3840,
			.height        = 2160,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GRBG,
		.ae_info = {
			.def_frm_len_lines     = 0x8A8,
			.curr_frm_len_lines    = 0x8A8 - 1,
			.one_line_exp_time_ns  = 15069,

			.max_integration_line  = 0x8A8 - 1,
			.min_integration_line  = 8,

			.max_again             = 240.45 * (1 << SENSOR_FIX_FRACBITS),//gain times
			.min_again             = 1 * (1 << SENSOR_FIX_FRACBITS),
			.max_dgain             = 1 * (1 << SENSOR_FIX_FRACBITS),
			.min_dgain             = 1 * (1 << SENSOR_FIX_FRACBITS),
			.gain_step             = 1,

			.start_exposure        = 3 * 100 * (1 << SENSOR_FIX_FRACBITS),
			.cur_fps               = 30 * (1 << SENSOR_FIX_FRACBITS),
			.max_fps               = 30 * (1 << SENSOR_FIX_FRACBITS),
			.min_fps               = 5 * (1 << SENSOR_FIX_FRACBITS),
			.min_afps              = 5 * (1 << SENSOR_FIX_FRACBITS),
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = ar0830_init_setting_linear_4k,
		.reg_data_count = ARRAY_SIZE(ar0830_init_setting_linear_4k),
	},

};

int ar0830_get_clk(struct ar0830 *sensor, void *clk)
{
	struct vvcam_clk_s vvcam_clk;
	int ret = 0;
	vvcam_clk.sensor_mclk = clk_get_rate(sensor->sensor_clk);
	vvcam_clk.csi_max_pixel_clk = sensor->ocp.max_pixel_frequency;
	ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
	if (ret != 0)
		ret = -EINVAL;
	return ret;
}

static int ar0830_power_on(struct ar0830 *sensor)
{
	int ret;
	pr_debug("enter %s\n", __func__);

	if (gpio_is_valid(sensor->pwn_gpio))
		gpio_set_value_cansleep(sensor->pwn_gpio, 1);

	ret = clk_prepare_enable(sensor->sensor_clk);
	if (ret < 0)
		pr_err("%s: enable sensor clk fail\n", __func__);

	return ret;
}

static int ar0830_power_off(struct ar0830 *sensor)
{
	pr_debug("enter %s\n", __func__);
	if (gpio_is_valid(sensor->pwn_gpio))
		gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	clk_disable_unprepare(sensor->sensor_clk);

	return 0;
}

static int ar0830_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0830 *sensor = client_to_ar0830(client);

	pr_debug("enter %s\n", __func__);
	if (on)
		ar0830_power_on(sensor);
	else
		ar0830_power_off(sensor);

	return 0;
}
//default 16bits data write by i2c
static s32 ar0830_write_reg(struct ar0830 *sensor, u16 reg, u16 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8Buf[4] = { reg >> 8, reg & 0xff, val >> 8, val & 0xff };

	if (i2c_master_send(sensor->i2c_client, au8Buf, 4) < 0) {
		dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
		return -1;
	}
	return 0;
}

static s32 ar0830_write_reg_special(struct ar0830 *sensor, u16 reg, u16 val, u16 bytes)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8Buf[4] = { reg >> 8, reg & 0xff, val >> 8, val & 0xff };;

	if(bytes == 1){
		au8Buf[2] = val & 0xff;
	}
	
	if (i2c_master_send(sensor->i2c_client, au8Buf, (2+bytes)) < 0) {
		dev_err(dev, "Write reg special error: reg=%x, val=%x\n", reg, val);
		return -1;
	}
	return 0;
}


static s32 ar0830_read_reg(struct ar0830 *sensor, u16 reg, u16 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = { reg >> 8, reg & 0xff };
	u8 au8RdVal[2] = {0};

	if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(sensor->i2c_client, au8RdVal, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x, val=%x %x\n",
                        reg, au8RdVal[0], au8RdVal[1]);
		return -1;
	}

	*val = ((u16)au8RdVal[0] << 8) | (u16)au8RdVal[1];

	return 0;
}


static int ar0830_stream_on(struct ar0830 *sensor)
{
	int ret;
	//u16 val = AR0830_MODE_STREAMING;
	
	ret = ar0830_write_reg_special(sensor, AR0830_REG_CTRL_MODE, AR0830_MODE_STREAMING, AR0830_REG_VALUE_08BIT);

	return ret;
}

static int ar0830_stream_off(struct ar0830 *sensor)
{
	int ret;
	//u16 val = 0;

	ret = ar0830_write_reg_special(sensor, AR0830_REG_CTRL_MODE, AR0830_MODE_SW_STANDBY, AR0830_REG_VALUE_08BIT);

	return ret;
}

static int ar0830_write_reg_arry(struct ar0830 *sensor,
				    struct vvcam_sccb_data_with_length_s *mode_setting,
				    s32 size)
{
	register u16 reg_addr = 0;
	register u16 data = 0;
	register u16 bytes = 0;
	int i, retval = 0;

	for (i = 0; i < size; ++i, ++mode_setting) {
		if (unlikely(mode_setting->addr == REG_DELAY)) {
			usleep_range(mode_setting->data, mode_setting->data * 2);
		} else {
			reg_addr = mode_setting->addr;
			data = mode_setting->data;
			bytes = mode_setting->len;

			retval = ar0830_write_reg_special(sensor, reg_addr, data, bytes);
			if (retval < 0)
				break;
		}
	}

	ar0830_stream_off(sensor);

	return retval;
}

static int ar0830_query_capability(struct ar0830 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strcpy((char *)pcap->driver, "ar0830");
	sprintf((char *)pcap->bus_info, "csi%d",sensor->csi_id);
	if(sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}
	return 0;
}

static int ar0830_query_supports(struct ar0830 *sensor, void* parry)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
	uint32_t support_counts = ARRAY_SIZE(par0830_mode_info);

	ret = copy_to_user(&psensor_mode_arry->count, &support_counts, sizeof(support_counts));
	ret |= copy_to_user(&psensor_mode_arry->modes, par0830_mode_info,
			   sizeof(par0830_mode_info));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int ar0830_get_sensor_id(struct ar0830 *sensor, void* pchip_id)
{
	int ret = 0;
	u16 chip_id;

	ret = ar0830_read_reg(sensor, 0x3000, &chip_id);
	ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int ar0830_get_reserve_id(struct ar0830 *sensor, void* preserve_id)
{
	int ret = 0;
	u16 reserve_id = 0x2770;
	ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int ar0830_get_sensor_mode(struct ar0830 *sensor, void* pmode)
{
	int ret = 0;
	ret = copy_to_user(pmode, &sensor->cur_mode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int ar0830_set_sensor_mode(struct ar0830 *sensor, void* pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;

	ret = copy_from_user(&sensor_mode, pmode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(par0830_mode_info); i++) 
    {
		if (par0830_mode_info[i].index == sensor_mode.index) 
        {
			memcpy(&sensor->cur_mode, &par0830_mode_info[i],sizeof(struct vvcam_mode_info_s));
			return 0;
		}
	}
	return -ENXIO;
}

static int ar0830_set_exp(struct ar0830 *sensor, u32 exp)
{
	int ret = 0;
	ret |= ar0830_write_reg(sensor, 0x0202, exp);

	return ret;
}

static int ar0830_set_gain(struct ar0830 *sensor, u32 gain)
{
	int ret = 0;
	u16 new_gain = 0;
	unsigned int i = 0;

	pr_info("%s : %d\n", __func__, gain);

	for(i = 0; i < GAIN_TABLE_SIZE; i++){
		if(gain <= ar0830_gain_table[i].times){
			new_gain = ar0830_gain_table[i].code;
			break;
		}
	}
	if (i == GAIN_TABLE_SIZE){
		new_gain = AR0830_GAIN_MAX;
	}
	if (new_gain > AR0830_GAIN_MAX) {
		new_gain = AR0830_GAIN_MAX;
	}else if (new_gain < AR0830_GAIN_MIN) {
                new_gain = AR0830_GAIN_MIN;
        }

	ret = ar0830_write_reg(sensor, AR0830_REG_GAIN, new_gain);
    return ret;
}

static int ar0830_set_fps(struct ar0830 *sensor, u32 fps)
{
	u32 vts;
	int ret = 0;

	if (fps > sensor->cur_mode.ae_info.max_fps) {
		fps = sensor->cur_mode.ae_info.max_fps;
	}
	else if (fps < sensor->cur_mode.ae_info.min_fps) {
		fps = sensor->cur_mode.ae_info.min_fps;
	}
	vts = sensor->cur_mode.ae_info.max_fps *
	      sensor->cur_mode.ae_info.def_frm_len_lines / fps;

	ret |= ar0830_write_reg(sensor, 0x0340, vts);
	sensor->cur_mode.ae_info.cur_fps = fps;

	if (sensor->cur_mode.hdr_mode == SENSOR_MODE_LINEAR) {
		sensor->cur_mode.ae_info.max_integration_line = vts - 1;
	} else {
		if (sensor->cur_mode.stitching_mode ==
		    SENSOR_STITCHING_DUAL_DCG){
			sensor->cur_mode.ae_info.max_vsintegration_line = 44;
			sensor->cur_mode.ae_info.max_integration_line = vts -
				4 - sensor->cur_mode.ae_info.max_vsintegration_line;
		} else {
			sensor->cur_mode.ae_info.max_integration_line = vts - 1;
		}
	}
	sensor->cur_mode.ae_info.curr_frm_len_lines = vts;
	return ret;
}

static int ar0830_get_fps(struct ar0830 *sensor, u32 *pfps)
{
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;

}

static int ar0830_set_test_pattern(struct ar0830 *sensor, void * arg)
{

	int ret;
	struct sensor_test_pattern_s test_pattern;

	ret = copy_from_user(&test_pattern, arg, sizeof(test_pattern));
	if (ret != 0)
		return -ENOMEM;
	if (test_pattern.enable) {
		switch (test_pattern.pattern) {
		case 0:
			ret |= ar0830_write_reg(sensor, 0x0600, 0x0001);
			break;
		case 1:
			ret |= ar0830_write_reg(sensor, 0x0600, 0x0002);
			break;
		case 2:
			ret |= ar0830_write_reg(sensor, 0x0600, 0x0003);
			break;
		default:
			ret = -1;
			break;
		}
	}
	return ret;
}

static int ar0830_get_lens(struct ar0830 *sensor, void * arg) {

	vvcam_lens_t *pfocus_lens = (vvcam_lens_t *)arg;

	if (!arg)
		return -ENOMEM;

	if (strlen(sensor->focus_lens.name) == 0)
		return -1;

	return copy_to_user(pfocus_lens, &sensor->focus_lens, sizeof(vvcam_lens_t));
}

static int ar0830_get_format_code(struct ar0830 *sensor, u32 *code)
{
	switch (sensor->cur_mode.bayer_pattern) {
	case BAYER_RGGB:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SRGGB8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SRGGB10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SRGGB12_1X12;
		}
		break;
	case BAYER_GRBG:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SGRBG8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SGRBG10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SGRBG12_1X12;
		}
		break;
	case BAYER_GBRG:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SGBRG8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SGBRG10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SGBRG12_1X12;
		}
		break;
	case BAYER_BGGR:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SBGGR8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SBGGR10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SBGGR12_1X12;
		}
		break;
	default:
		/*nothing need to do*/
		break;
	}
	return 0;
}


static int ar0830_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0830 *sensor = client_to_ar0830(client);
    int ret;
	
	pr_debug("enter %s\n", __func__);
	if (enable) {
        ret = ar0830_stream_on(sensor);
        if (ret < 0)
            return ret;
    } else {
        ret = ar0830_stream_off(sensor);
        if (ret < 0)
            return ret;
    }

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ar0830_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
#else
static int ar0830_enum_mbus_code(struct v4l2_subdev *sd,
			         struct v4l2_subdev_pad_config *cfg,
			         struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0830 *sensor = client_to_ar0830(client);

	u32 cur_code = MEDIA_BUS_FMT_SBGGR12_1X12;

	if (code->index > 0)
		return -EINVAL;
	ar0830_get_format_code(sensor,&cur_code);
	code->code = cur_code;

	return 0;
}


#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ar0830_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
#else
static int ar0830_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)

#endif
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0830 *sensor = client_to_ar0830(client);

	mutex_lock(&sensor->lock);
	pr_info("enter %s\n", __func__);
	if ((fmt->format.width != sensor->cur_mode.size.bounds_width) ||
	    (fmt->format.height != sensor->cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
			__func__,fmt->format.width,fmt->format.height);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}
	
	ret |= ar0830_write_reg_arry(sensor,
		sensor->cur_mode.preg_data,
		sensor->cur_mode.reg_data_count);
	
	if (ret < 0) {
		pr_err("%s:ar0830_write_reg_arry error\n",__func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}
	ar0830_get_format_code(sensor, &fmt->format.code);
	fmt->format.field = V4L2_FIELD_NONE;
	sensor->format = fmt->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int ar0830_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
#else
static int ar0830_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0830 *sensor = client_to_ar0830(client);

	mutex_lock(&sensor->lock);
	fmt->format = sensor->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

static long ar0830_priv_ioctl(struct v4l2_subdev *sd,
                              unsigned int cmd,
                              void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0830 *sensor = client_to_ar0830(client);
	long ret = 0;
	struct vvcam_sccb_data_s sensor_reg;
	uint32_t value = 0;

	mutex_lock(&sensor->lock);
	switch (cmd){
	case VVSENSORIOC_S_POWER:
		ret = 0;
		break;
	case VVSENSORIOC_S_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_G_CLK:
		ret = ar0830_get_clk(sensor,arg);
		break;
	case VVSENSORIOC_RESET:
		ret = 0;
		break;
	case VIDIOC_QUERYCAP:
		ret = ar0830_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = ar0830_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = ar0830_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = ar0830_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = ar0830_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = ar0830_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0830_s_stream(&sensor->subdev, value);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= ar0830_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg, sizeof(struct vvcam_sccb_data_s));
		ret |= ar0830_read_reg(sensor, (u16)sensor_reg.addr, (u16 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg, sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_EXP:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0830_set_exp(sensor, value);
		break;
	case VVSENSORIOC_S_GAIN:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0830_set_gain(sensor, value);
		break;
	case VVSENSORIOC_S_FPS:
		ret = copy_from_user(&value, arg, sizeof(value));
		ret |= ar0830_set_fps(sensor, value);
		break;
	case VVSENSORIOC_G_FPS:
		ret = ar0830_get_fps(sensor, &value);
		ret |= copy_to_user(arg, &value, sizeof(value));
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret= ar0830_set_test_pattern(sensor, arg);
		break;
	case VVSENSORIOC_G_LENS:
		ret = ar0830_get_lens(sensor, arg);
		break;
	default:
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static struct v4l2_subdev_video_ops ar0830_subdev_video_ops = {
	.s_stream = ar0830_s_stream,
};

static const struct v4l2_subdev_pad_ops ar0830_subdev_pad_ops = {
	.enum_mbus_code = ar0830_enum_mbus_code,
	.set_fmt = ar0830_set_fmt,
	.get_fmt = ar0830_get_fmt,
};

static struct v4l2_subdev_core_ops ar0830_subdev_core_ops = {
	.s_power = ar0830_s_power,
	.ioctl = ar0830_priv_ioctl,
};

static struct v4l2_subdev_ops ar0830_subdev_ops = {
	.core  = &ar0830_subdev_core_ops,
	.video = &ar0830_subdev_video_ops,
	.pad   = &ar0830_subdev_pad_ops,
};

static int ar0830_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations ar0830_sd_media_ops = {
	.link_setup = ar0830_link_setup,
};

static int ar0830_regulator_enable(struct ar0830 *sensor)
{
	int ret = 0;
	struct device *dev = &(sensor->i2c_client->dev);

	pr_debug("enter %s\n", __func__);

	if (sensor->io_regulator) {
		regulator_set_voltage(sensor->io_regulator,
				      AR0830_VOLTAGE_DIGITAL_IO,
				      AR0830_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(sensor->io_regulator);
		if (ret < 0) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		}
	}

	if (sensor->analog_regulator) {
		regulator_set_voltage(sensor->analog_regulator,
				      AR0830_VOLTAGE_ANALOG,
				      AR0830_VOLTAGE_ANALOG);
		ret = regulator_enable(sensor->analog_regulator);
		if (ret) {
			dev_err(dev, "set analog voltage failed\n");
			goto err_disable_io;
		}

	}

	if (sensor->core_regulator) {
		regulator_set_voltage(sensor->core_regulator,
				      AR0830_VOLTAGE_DIGITAL_CORE,
				      AR0830_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(sensor->core_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			goto err_disable_analog;
		}
	}

	return 0;

err_disable_analog:
	regulator_disable(sensor->analog_regulator);
err_disable_io:
	regulator_disable(sensor->io_regulator);
	return ret;
}

static void ar0830_regulator_disable(struct ar0830 *sensor)
{
	int ret = 0;
	struct device *dev = &(sensor->i2c_client->dev);

	if (sensor->core_regulator) {
		ret = regulator_disable(sensor->core_regulator);
		if (ret < 0)
			dev_err(dev, "core regulator disable failed\n");
	}

	if (sensor->analog_regulator) {
		ret = regulator_disable(sensor->analog_regulator);
		if (ret < 0)
			dev_err(dev, "analog regulator disable failed\n");
	}

	if (sensor->io_regulator) {
		ret = regulator_disable(sensor->io_regulator);
		if (ret < 0)
			dev_err(dev, "io regulator disable failed\n");
	}
	return ;
}

static int ar0830_set_clk_rate(struct ar0830 *sensor)
{
	int ret;
	unsigned int clk;

	clk = sensor->mclk;
	clk = min_t(u32, clk, (u32)AR0830_XCLK_MAX);
	clk = max_t(u32, clk, (u32)AR0830_XCLK_MIN);
	sensor->mclk = clk;

	pr_debug("   Setting mclk to %d MHz\n",sensor->mclk / 1000000);
	ret = clk_set_rate(sensor->sensor_clk, sensor->mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", sensor->mclk);
	return ret;
}

static void ar0830_reset(struct ar0830 *sensor)
{
	pr_debug("enter %s\n", __func__);
	if (!gpio_is_valid(sensor->rst_gpio))
		return;

	gpio_set_value_cansleep(sensor->rst_gpio, 0);
	msleep(20);

	gpio_set_value_cansleep(sensor->rst_gpio, 1);
	msleep(20);

	return;
}
static int ar0830_retrieve_capture_properties(
			struct ar0830 *sensor,
			struct ar0830_capture_properties* ocp)
{
	struct device *dev = &sensor->i2c_client->dev;
	__u64 mlf = 0;
	__u64 mpf = 0;
	__u64 mdr = 0;

	struct device_node *ep;
	int ret;
	/*Collecting the information about limits of capture path
	* has been centralized to the sensor
	* * also into the sensor endpoint itself.
	*/

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -ENODEV;
	}

	/*ret = fwnode_property_read_u64(of_fwnode_handle(ep),
		"max-lane-frequency", &mlf);
	if (ret || mlf == 0) {
		dev_dbg(dev, "no limit for max-lane-frequency\n");
	}*/
	ret = fwnode_property_read_u64(of_fwnode_handle(ep),
	        "max-pixel-frequency", &mpf);
	if (ret || mpf == 0) {
	        dev_dbg(dev, "no limit for max-pixel-frequency\n");
	}

	/*ret = fwnode_property_read_u64(of_fwnode_handle(ep),
	        "max-data-rate", &mdr);
	if (ret || mdr == 0) {
	        dev_dbg(dev, "no limit for max-data_rate\n");
	}*/

	ocp->max_lane_frequency = mlf;
	ocp->max_pixel_frequency = mpf;
	ocp->max_data_rate = mdr;

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
static int ar0830_probe(struct i2c_client *client)
#else
static int ar0830_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
#endif
{
	int retval;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct ar0830 *sensor;
    u16 chip_id;
	struct device_node *lens_node;

	pr_info("enter %s\n", __func__);

	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	memset(sensor, 0, sizeof(*sensor));

	sensor->i2c_client = client;

	sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(sensor->pwn_gpio))
		dev_warn(dev, "No sensor pwdn pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->pwn_gpio,
						GPIOF_OUT_INIT_HIGH,
						"ar0830_mipi_pwdn");
		if (retval < 0) {
			dev_warn(dev, "Failed to set power pin\n");
			dev_warn(dev, "retval=%d\n", retval);
			return retval;
		}
	}

	sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(sensor->rst_gpio))
		dev_warn(dev, "No sensor reset pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->rst_gpio,
						GPIOF_OUT_INIT_HIGH,
						"ar0830_mipi_reset");
		if (retval < 0) {
			dev_warn(dev, "Failed to set reset pin\n");
			return retval;
		}
	}

	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk", &(sensor->mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
				(u32 *)&(sensor->mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id", &(sensor->csi_id));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	lens_node = of_parse_phandle(dev->of_node, "lens-focus", 0);
	if (lens_node) {
		retval = of_property_read_u32(lens_node, "id", &sensor->focus_lens.id);
		if (retval) {
			dev_err(dev, "lens-focus id missing or invalid\n");
			return retval;
		}
		memcpy(sensor->focus_lens.name, lens_node->name, strlen(lens_node->name));
	}

	retval = ar0830_retrieve_capture_properties(sensor,&sensor->ocp);
	if (retval) {
		dev_warn(dev, "retrive capture properties error\n");
	}

	sensor->io_regulator = devm_regulator_get(dev, "DOVDD");
	if (IS_ERR(sensor->io_regulator)) {
		dev_err(dev, "cannot get io regulator\n");
		return PTR_ERR(sensor->io_regulator);
	}

	sensor->core_regulator = devm_regulator_get(dev, "DVDD");
	if (IS_ERR(sensor->core_regulator)) {
		dev_err(dev, "cannot get core regulator\n");
		return PTR_ERR(sensor->core_regulator);
	}

	sensor->analog_regulator = devm_regulator_get(dev, "AVDD");
	if (IS_ERR(sensor->analog_regulator)) {
		dev_err(dev, "cannot get analog  regulator\n");
		return PTR_ERR(sensor->analog_regulator);
	}

	retval = ar0830_regulator_enable(sensor);
	if (retval) {
		dev_err(dev, "regulator enable failed\n");
		return retval;
	}

	ar0830_set_clk_rate(sensor);
	retval = clk_prepare_enable(sensor->sensor_clk);
	if (retval < 0) {
		dev_err(dev, "%s: enable sensor clk fail\n", __func__);
		goto probe_err_regulator_disable;
	}
	mdelay(2);

	retval = ar0830_power_on(sensor);
	if (retval < 0) {
		dev_err(dev, "%s: sensor power on fail\n", __func__);
		goto probe_err_regulator_disable;
	}

	ar0830_reset(sensor);

    ar0830_read_reg(sensor, AR0830_CHIP_VERSION_REG, &chip_id);
	if (chip_id != AR0830_CHIP_ID) {
		dev_err(dev, "Sensor AR0830 is not found\n");
        goto probe_err_power_off;
    }

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &ar0830_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.ops = &ar0830_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[AR0830_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity,
				AR0830_SENS_PADS_NUM,
				sensor->pads);
	if (retval < 0)
		goto probe_err_power_off;

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
	retval = v4l2_async_register_subdev_sensor(sd);
#else
	retval = v4l2_async_register_subdev_sensor_common(sd);
#endif
	if (retval < 0) {
		dev_err(&client->dev,"%s--Async register failed, ret=%d\n",
			__func__,retval);
		goto probe_err_free_entiny;
	}

	memcpy(&sensor->cur_mode, &par0830_mode_info[0],
			sizeof(struct vvcam_mode_info_s));

	mutex_init(&sensor->lock);

	pr_info("%s camera mipi ar0830, is found\n", __func__);

	return 0;

probe_err_free_entiny:
	media_entity_cleanup(&sd->entity);

probe_err_power_off:
	ar0830_power_off(sensor);

probe_err_regulator_disable:
	ar0830_regulator_disable(sensor);

	return retval;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
static int ar0830_remove(struct i2c_client *client)
#else
static void ar0830_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0830 *sensor = client_to_ar0830(client);

	pr_info("enter %s\n", __func__);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	ar0830_power_off(sensor);
	ar0830_regulator_disable(sensor);
	mutex_destroy(&sensor->lock);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
	return 0;
#else
#endif
}

static int __maybe_unused ar0830_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ar0830 *sensor = client_to_ar0830(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status) {
		ar0830_s_stream(&sensor->subdev,0);
	}

	return 0;
}

static int __maybe_unused ar0830_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ar0830 *sensor = client_to_ar0830(client);

	if (sensor->resume_status) {
		ar0830_s_stream(&sensor->subdev,1);
	}

	return 0;
}

static const struct dev_pm_ops ar0830_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ar0830_suspend, ar0830_resume)
};

static const struct i2c_device_id ar0830_id[] = {
	{"ar0830", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ar0830_id);

static const struct of_device_id ar0830_of_match[] = {
	{ .compatible = "onsemi,ar0830" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ar0830_of_match);

static struct i2c_driver ar0830_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "ar0830",
		.pm = &ar0830_pm_ops,
		.of_match_table	= ar0830_of_match,
	},
	.probe  = ar0830_probe,
	.remove = ar0830_remove,
	.id_table = ar0830_id,
};


module_i2c_driver(ar0830_i2c_driver);
MODULE_DESCRIPTION("AR0830 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
