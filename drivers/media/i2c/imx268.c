/*
 * imx268.c - imx268 sensor driver
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>

#include "../platform/tegra/camera/camera_gpio.h"
#include "imx268_mode_tbls.h"

/* imx268 - sensor i2c register addresses */
#define IMX268_COARSE_INTEG_TIME_ADDR_MSB	0x0202
#define IMX268_COARSE_INTEG_TIME_ADDR_LSB	0x0203
#define IMX268_GAIN_ADDR_MSB			0x0204
#define IMX268_GAIN_ADDR_LSB			0x0205
#define IMX268_FRAME_LENGTH_ADDR_MSB		0x0340
#define IMX268_FRAME_LENGTH_ADDR_LSB		0x0341

static const struct of_device_id imx268_of_match[] = {
	{ .compatible = "nvidia,imx268", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx268_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	/* TEGRA_CAMERA_CID_GROUP_HOLD, */
	/* TEGRA_CAMERA_CID_EEPROM_DATA, */
	/* TEGRA_CAMERA_CID_FUSE_ID, */
};

struct imx268 {
	struct i2c_client *i2c_client;
	struct v4l2_subdev *subdev;
	u32 frame_length;
	struct camera_common_data *s_data;
	struct tegracam_device *tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline void imx268_get_coarse_time_regs(imx268_reg *regs,
	u32 coarse_time)
{
	regs->addr = IMX268_COARSE_INTEG_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0xff;
	(regs + 1)->addr = IMX268_COARSE_INTEG_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0xff;
}

static inline void imx268_get_gain_regs(imx268_reg *regs, s16 gain)
{
	regs->addr = IMX268_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x01;
	(regs + 1)->addr = IMX268_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static inline void imx268_get_frame_length_regs(imx268_reg *regs,
	u32 frame_length)
{
	regs->addr = IMX268_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 8) & 0xff;
	(regs + 1)->addr = IMX268_FRAME_LENGTH_ADDR_LSB;
	(regs + 1)->val = (frame_length) & 0xff;
}

static inline int imx268_read_reg(struct camera_common_data *s_data,
	u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static inline int imx268_write_reg(struct camera_common_data *s_data,
	u16 addr, u8 val)
{
	int err;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

static inline int imx268_write_table(struct imx268 *priv,
	const imx268_reg table[])
{
	return regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
		IMX268_TABLE_WAIT_MS, IMX268_TABLE_END);
}

static int imx268_set_group_hold(struct camera_common_data *s_data, bool val)
{
	struct device *dev = s_data->dev;
	dev_err(dev, "%s: unsupported, should not be called", __func__);
	return 0;
}

static int imx268_set_exposure(struct camera_common_data *s_data, s64 val)
{
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
		&(s_data->sensor_props.sensor_modes[s_data->mode_prop_idx]);
	imx268_reg reg_list[2];
	int err = 0;
	u32 coarse_time;
	int i;

	coarse_time = mode->signal_properties.pixel_clock.val *
		val / mode->image_properties.line_length /
		mode->control_properties.exposure_factor;

	/* @todo: min and max coarse_time bounds */

	dev_dbg(dev, "%s: coarse_time: %d\n", __func__, coarse_time);

	imx268_get_coarse_time_regs(reg_list, coarse_time);

	for (i = 0; i < 2; i++) {
		err = imx268_write_reg(s_data, reg_list[i].addr,
			reg_list[i].val);
		if (err) {
			dev_dbg(dev,
				"%s: coarse_time control error\n", __func__);
			return err;
		}
	}

	return err;
}

static int imx268_set_gain(struct camera_common_data *s_data, s64 val)
{
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
		&(s_data->sensor_props.sensor_modes[s_data->mode_prop_idx]);
	imx268_reg reg_list[2];
	int err = 0;
	s16 gain;
	int i;

	if (val < mode->control_properties.min_gain_val)
		val = mode->control_properties.min_gain_val;
	else if (val > mode->control_properties.max_gain_val)
		val = mode->control_properties.max_gain_val;

	/* translate value */
	gain = (s16)((512 * mode->control_properties.gain_factor) / val);
	gain = 512 - gain;

	if (gain < 0)
		gain = 0;

	dev_dbg(dev, "%s: gain: %d, times: %lld\n", __func__, gain, val);

	imx268_get_gain_regs(reg_list, gain);

	for (i = 0; i < 2; i++) {
		err = imx268_write_reg(s_data, reg_list[i].addr,
			reg_list[i].val);
		if (err) {
			dev_dbg(dev, "%s: gain control error\n", __func__);
			return err;
		}
	}

	return err;
}

static int imx268_set_frame_rate(struct camera_common_data *s_data, s64 val)
{
	struct imx268 *priv = (struct imx268 *)s_data->priv;
	struct device *dev = s_data->dev;
	const struct sensor_mode_properties *mode =
		&(s_data->sensor_props.sensor_modes[s_data->mode_prop_idx]);
	imx268_reg reg_list[2];
	int err = 0;
	u32 frame_length;
	int i;

	frame_length = (u32)(mode->signal_properties.pixel_clock.val *
		((u64)mode->control_properties.framerate_factor) /
		mode->image_properties.line_length / val);

	dev_dbg(dev, "%s: frame_length: %d\n", __func__, frame_length);

	imx268_get_frame_length_regs(reg_list, frame_length);

	for (i = 0; i < 2; i++) {
		err = imx268_write_reg(s_data, reg_list[i].addr,
			reg_list[i].val);
		if (err) {
			dev_dbg(dev,
				"%s: frame_length control error\n", __func__);
			return err;
		}
	}

	priv->frame_length = frame_length;

	return 0;
}

static int imx268_fill_string_ctrl(struct camera_common_data *s_data,
				struct v4l2_ctrl *ctrl)
{
	struct device *dev = s_data->dev;
	dev_err(dev, "%s: unsupported, should not be called", __func__);
	return 0;
}

static struct tegracam_ctrl_ops imx268_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.string_ctrl_size = { 0, 0 }, /* unsupported */
	.set_gain = imx268_set_gain,
	.set_exposure = imx268_set_exposure,
	.set_frame_rate = imx268_set_frame_rate,
	.set_group_hold = imx268_set_group_hold, /* unsupported */
	.fill_string_ctrl = imx268_fill_string_ctrl, /* unsupported */
};

static inline int imx268_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s: power on failed\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 0);

	usleep_range(15, 20);

	if (pw->avdd)
		err = regulator_enable(pw->avdd);
	if (err)
		goto imx268_avdd_fail;

	if (pw->iovdd)
		err = regulator_enable(pw->iovdd);
	if (err)
		goto imx268_iovdd_fail;

	if (pw->dvdd)
		err = regulator_enable(pw->dvdd);
	if (err)
		goto imx268_dvdd_fail;

	usleep_range(5000, 5010);

	if (pw->reset_gpio)
		gpio_set_value(pw->reset_gpio, 1);

	usleep_range(12000, 12010);

	pw->state = SWITCH_ON;

	return 0;

imx268_dvdd_fail:
	regulator_disable(pw->iovdd);

imx268_iovdd_fail:
	regulator_disable(pw->avdd);

imx268_avdd_fail:
	dev_err(dev, "%s: power on failed\n", __func__);

	return err;
}

static int imx268_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		dev_dbg(dev, "%s: b1\n", __func__);
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s: power off failed\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio)
			gpio_set_value(pw->reset_gpio, 0);

		usleep_range(15, 20);

		if (pw->avdd)
			regulator_disable(pw->avdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->dvdd)
			regulator_disable(pw->dvdd);
	}

	pw->state = SWITCH_OFF;

	return 0;
}

static int imx268_power_put(struct camera_common_data *s_data)
{
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (pdata && pdata->use_cam_gpio) {
		cam_gpio_deregister(s_data->dev, pw->pwdn_gpio);
	} else {
		gpio_free(pw->pwdn_gpio);
		gpio_free(pw->reset_gpio);
	}

	return 0;
}

static int imx268_power_get(struct camera_common_data *s_data)
{
	struct device *dev = s_data->dev;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0, ret = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	mclk_name = pdata->mclk_name ? pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent)) {
			dev_err(dev, "unable to get parent clock %s",
				parentclk_name);
		} else {
			clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	err |= camera_common_regulator_get(dev,
		&pw->avdd, pdata->regulators.avdd);
	/* IO 1.8v */
	err |= camera_common_regulator_get(dev,
		&pw->iovdd, pdata->regulators.iovdd);
	/* dig 1.2v */
	err |= camera_common_regulator_get(dev,
		&pw->dvdd, pdata->regulators.dvdd);

	if (!err)
		pw->reset_gpio = pdata->reset_gpio;

	ret = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (ret < 0)
		dev_dbg(dev, "%s can't request reset_gpio %d\n", __func__, ret);

	pw->state = SWITCH_OFF;

	return err;
}

static struct camera_common_pdata *imx268_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err = 0;
	struct camera_common_pdata *ret = NULL;

	if (!np)
		return NULL;

	match = of_match_device(imx268_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
		sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	board_priv_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (board_priv_pdata->reset_gpio < 0) {
		if (board_priv_pdata->reset_gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found %d\n", err);
		goto error;
	}

	err = of_property_read_string(np, "avdd-reg",
		&board_priv_pdata->regulators.avdd);
	if (err) {
		dev_err(dev, "avdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(np, "iovdd-reg",
		&board_priv_pdata->regulators.iovdd);
	if (err) {
		dev_err(dev, "iovdd-reg not in DT\n");
		goto error;
	}
	err = of_property_read_string(np, "dvdd-reg",
		&board_priv_pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "dvdd-reg not in DT\n");
		goto error;
	}
	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static struct camera_common_sensor_ops imx268_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx268_frmfmt),
	.frmfmt_table = imx268_frmfmt,
	.power_on = imx268_power_on,
	.power_off = imx268_power_off,
	.write_reg = imx268_write_reg,
	.read_reg = imx268_read_reg,
	.parse_dt = imx268_parse_dt,
	.power_get = imx268_power_get,
	.power_put = imx268_power_put,
};

static int imx268_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx268 *priv = (struct imx268 *)s_data->priv;
	struct v4l2_ext_controls ctrls;
	struct v4l2_ext_control control[3];
	int err;

	dev_dbg(&client->dev, "%s++ enable %d\n", __func__, enable);

	if (!enable) {
		err = imx268_write_table(priv,
			mode_table[IMX268_MODE_STREAM_STOP]);
		if (err)
			return err;

		/*
		 * Wait for one frame to make sure sensor is set to
		 * software standby in V-blank
		 *
		 * delay = frame length rows * Tline (10 us)
		 */
		usleep_range(priv->frame_length * 10,
			priv->frame_length * 10 + 1000);

		return 0;
	}

	err = imx268_write_table(priv, mode_table[IMX268_MODE_COMMON]);
	if (err)
		goto exit;

	err = imx268_write_table(priv, mode_table[s_data->mode]);
	if (err)
		goto exit;

	if (s_data->override_enable) {
		/*
		 * write list of override regs for the asking frame length,
		 * coarse integration time, and gain. Failures to write
		 * overrides are non-fatal
		 */
		memset(&ctrls, 0, sizeof(ctrls));
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
		ctrls.which = V4L2_CTRL_ID2WHICH(TEGRA_CAMERA_CID_GAIN);
#else
		ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(TEGRA_CAMERA_CID_GAIN);
#endif
		ctrls.count = 3;
		ctrls.controls = control;

		control[0].id = TEGRA_CAMERA_CID_GAIN;
		control[1].id = TEGRA_CAMERA_CID_FRAME_RATE;
		control[2].id = TEGRA_CAMERA_CID_EXPOSURE;

		err = v4l2_g_ext_ctrls(s_data->ctrl_handler, &ctrls);
		if (err == 0) {
			err |= imx268_set_gain(s_data, control[0].value64);
			if (err)
				dev_err(&client->dev,
					"%s: error gain override\n", __func__);

			err |= imx268_set_frame_rate(s_data,
					control[1].value64);
			if (err)
				dev_err(&client->dev,
					"%s: error frame length override\n",
					__func__);

			err |= imx268_set_exposure(s_data, control[2].value64);
			if (err)
				dev_err(&client->dev,
					"%s: error exposure override\n",
					__func__);

		} else {
			dev_err(&client->dev, "%s: failed to get overrides\n",
				__func__);
		}
	}

	err = imx268_write_table(priv, mode_table[IMX268_MODE_STREAM_START]);
	if (err)
		goto exit;

	return 0;
exit:
	dev_err(&client->dev, "%s: error setting stream\n", __func__);
	return err;
}

static int imx268_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct camera_common_power_rail *pw = s_data->power;

	*status = pw->state == SWITCH_ON;

	return 0;
}

static struct v4l2_subdev_video_ops imx268_subdev_video_ops = {
	.s_stream	= imx268_s_stream,
	.g_mbus_config	= camera_common_g_mbus_config,
	.g_input_status = imx268_g_input_status,
};

static struct v4l2_subdev_core_ops imx268_subdev_core_ops = {
	.s_power	= camera_common_s_power,
};

static int imx268_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int imx268_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	int ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		ret = camera_common_try_fmt(sd, &format->format);
	else
		ret = camera_common_s_fmt(sd, &format->format);

	return ret;
}

static struct v4l2_subdev_pad_ops imx268_subdev_pad_ops = {
	.set_fmt = imx268_set_fmt,
	.get_fmt = imx268_get_fmt,
	.enum_mbus_code = camera_common_enum_mbus_code,
	.enum_frame_size = camera_common_enum_framesizes,
	.enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops imx268_subdev_ops = {
	.core = &imx268_subdev_core_ops,
	.video = &imx268_subdev_video_ops,
	.pad = &imx268_subdev_pad_ops,
};

static int imx268_board_setup(struct imx268 *priv)
{
	int err = 0;
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	u8 reg_val;

	dev_dbg(dev, "%s++\n", __func__);

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev, "Error %d turning on mclk\n", err);
		return err;
	}

	err = imx268_power_on(s_data);
	if (err) {
		dev_err(dev, "Error %d during power on sensor\n", err);
		return err;
	}

	/* Probe FRM_CNT register */
	err = imx268_read_reg(s_data, 0x05, &reg_val);
	if (err) {
		dev_err(dev, "Error %d during i2c read probe\n", err);
		return err;
	}

	imx268_power_off(s_data);
	camera_common_mclk_disable(s_data);

	return err;
}

static int imx268_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx268_subdev_internal_ops = {
	.open = imx268_open,
};

static const struct media_entity_operations imx268_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx268_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx268 *priv;
	int err;

	dev_dbg(dev, "[imx268]: probing v4l2 sensor at addr 0x%0x.\n",
		client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct imx268), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx268", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx268_common_ops;
	tc_dev->v4l2sd_ops = &imx268_subdev_ops;
	tc_dev->v4l2sd_internal_ops = &imx268_subdev_internal_ops;
	tc_dev->media_ops = &imx268_media_ops;
	tc_dev->tcctrl_ops = &imx268_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx268_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_dbg(dev, "detected imx268 sensor\n");

	return 0;
}

static int imx268_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx268 *priv = (struct imx268 *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx268_id[] = {
	{ "imx268", 0 },
	{ }
};

static struct i2c_driver imx268_i2c_driver = {
	.driver = {
		.name = "imx268",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx268_of_match),
	},
	.probe = imx268_probe,
	.remove = imx268_remove,
	.id_table = imx268_id,
};

module_i2c_driver(imx268_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX268");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
