#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/gmsl-common.h>
#include <media/gmsl-link.h>

#include <media/tegracam_core.h>
#include "ar0233_mdoe_tbls.h"

#include "serializer.h"
#include "deserializer.h"

#define DEBUG_LOG_LEVEL 1

#if DEBUG_LOG_LEVEL == 1
#define AR0233_LOG(fmt, args...) printk(KERN_INFO "[AR0233] " fmt, ##args)
#else
#define AR0233_LOG(fmt, args...)
#endif

#define AR0233_ERR(fmt, args...) printk(KERN_ERR "[AR0233] " fmt, ##args)

struct ar0233 {
	struct i2c_client	*i2c_client;
	const struct i2c_device_id *id;

	//use for v4l2 driver
	struct v4l2_subdev	*subdev;

    //use for unqiue for Nvidia Tegra camera driver
    struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;

    //use for GMSL
	struct device		*ser_dev;
	struct device		*dser_dev;
	struct device		*ser_prim_dev;
    struct serializer *priv_prim_ser;   //use for broadcast channel
    struct serializer *priv_ser;
    struct deserializer *priv_deser;
		//use for GMSL link
    struct gmsl_link_ctx	g_ctx;
};

const struct of_device_id ar0233_of_match[] = {
	{ .compatible = "nvidia,ar0233",},
	{ },
};

static const struct i2c_device_id ar0233_id[] = {
	{ "ar0233", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ar0233_id);

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_RBTREE,
};

static int ar0233_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int ar0233_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int ar0233_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int ar0233_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	return 0;
}

static struct tegracam_ctrl_ops ar0233_ctrl_ops = {
	 .numctrls = ARRAY_SIZE(ctrl_cid_list),
	 .ctrl_cid_list = ctrl_cid_list,
	 .set_gain = ar0233_set_gain,
	 .set_exposure = ar0233_set_exposure,
	 .set_exposure_short = ar0233_set_exposure,
	 .set_frame_rate = ar0233_set_frame_rate,
	 .set_group_hold = ar0233_set_group_hold,
};

static int ar0233_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	AR0233_LOG("%s:\n", __func__);
	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops ar0233_subdev_internal_ops = {
	 .open = ar0233_open,
};

static int ar0233_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;

	AR0233_LOG("ar0233 power on\n");

	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			AR0233_ERR("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	pw->state = SWITCH_ON;

	return 0;
}

static int ar0233_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;

	AR0233_LOG("ar0233 power off\n");

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			AR0233_ERR("%s failed.\n", __func__);
		return err;
	}

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static struct camera_common_pdata *ar0233_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *node = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	int err;

	if (!node)
		return NULL;

	match = of_match_device(ar0233_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev, sizeof(*board_priv_pdata), GFP_KERNEL);

	err = of_property_read_string(node, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		AR0233_ERR("mclk not in DT\n");

	return board_priv_pdata;
}

static int ar0233_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = pdata->mclk_name ? pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		AR0233_ERR("unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(dev, parentclk_name);
		if (IS_ERR(parent))
			AR0233_ERR("unable to get parent clcok %s",parentclk_name);
		else
			clk_set_parent(pw->mclk, parent);
	}

	pw->state = SWITCH_OFF;

	return err;
}

static int ar0233_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static int ar0233_set_mode(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	const struct of_device_id *match;

	match = of_match_device(ar0233_of_match, dev);
	if (!match) {
		AR0233_ERR("Failed to find matching dt id\n");
		return -EINVAL;
	}

	AR0233_LOG("mode:%d",s_data->mode_prop_idx);

	return 0;
}

static int ar0233_start_streaming(struct tegracam_device *tc_dev)
{
	return 0;
}

static int ar0233_stop_streaming(struct tegracam_device *tc_dev)
{
	return 0;
}

static struct camera_common_sensor_ops ar0233_common_ops = {
	 .numfrmfmts = ARRAY_SIZE(ar0233_frmfmt),
	 .frmfmt_table = ar0233_frmfmt,
	 .power_on = ar0233_power_on,
	 .power_off = ar0233_power_off,
	 .parse_dt = ar0233_parse_dt,
	 .power_get = ar0233_power_get,
	 .power_put = ar0233_power_put,
	 .set_mode = ar0233_set_mode,
	 .start_streaming = ar0233_start_streaming,
	 .stop_streaming = ar0233_stop_streaming,
};

static int ar0233_board_setup(struct ar0233 *priv)
{
    struct tegracam_device *tc_dev = priv->tc_dev;
    struct device *dev = tc_dev->dev;
    struct device_node *node = dev->of_node;
	//GMSL
	struct device_node *ser_prim_node;
	struct device_node *ser_node;
	struct device_node *dser_node;
	struct i2c_client *ser_prim_i2c = NULL;
	struct i2c_client *ser_i2c = NULL;
	struct i2c_client *dser_i2c = NULL;
	struct device_node *gmsl_link;
	const char *str_value;
	const char *str_array[GMSL_DEV_MAX_NUM_DATA_STREAMS];
	int value = 0xFFFF;
	int  i;
    int err;

    AR0233_LOG("prase dts\n");

    err = of_property_read_u32(node, "def-addr", &priv->g_ctx.sdev_def);
    if (err < 0) {
		AR0233_ERR("reg not found\n");
		goto error;
	}

    err = of_property_read_u32(node, "reg", &priv->g_ctx.sdev_reg);
    if (err < 0) {
		AR0233_ERR("reg not found\n");
		goto error;
	}

/***************parse primitive device tree***********************/
	ser_prim_node = of_parse_phandle(node, "nvidia,gmsl-ser-prim-device", 0);
	if (ser_prim_node == NULL) {
		AR0233_ERR("missing %s handle\n", "nvidia,gmsl-ser-device");
		goto error;
	}

	ser_prim_i2c = of_find_i2c_device_by_node(ser_prim_node);
	of_node_put(ser_prim_node);

	if (ser_prim_i2c == NULL) {
		AR0233_ERR("missing serializer dev handle\n");
		goto error;
	}
	if (ser_prim_i2c->dev.driver == NULL) {
		AR0233_ERR("missing serializer driver\n");
		goto error;
	}

	priv->ser_prim_dev = &ser_prim_i2c->dev;
	priv->priv_prim_ser = dev_get_drvdata(priv->ser_prim_dev);
/*******************************************************************/


/***************parse serializer device tree***********************/
	ser_node = of_parse_phandle(node, "nvidia,gmsl-ser-device", 0);
	if (ser_node == NULL) {
		AR0233_ERR("missing %s handle\n", "nvidia,gmsl-ser-device");
		goto error;
	}

    err = of_property_read_u32(ser_node, "reg", &priv->g_ctx.ser_reg);
    if (err < 0) {
		AR0233_ERR("reg not found\n");
		goto error;
	}

	ser_i2c = of_find_i2c_device_by_node(ser_node);
	of_node_put(ser_node);

	if (ser_i2c == NULL) {
		AR0233_ERR("missing serializer dev handle\n");
		goto error;
	}
	if (ser_i2c->dev.driver == NULL) {
		AR0233_ERR("missing serializer driver\n");
		goto error;
	}

	priv->ser_dev = &ser_i2c->dev;
	priv->priv_ser = dev_get_drvdata(priv->ser_dev);
/*******************************************************************/

/***************parse deserializer device tree***********************/
	dser_node = of_parse_phandle(node, "nvidia,gmsl-dser-device", 0);
	if (dser_node == NULL) {
		AR0233_ERR("missing %s handle\n", "nvidia,gmsl-dser-device");
		goto error;
	}

	dser_i2c = of_find_i2c_device_by_node(dser_node);
	of_node_put(dser_node);

	if (dser_i2c == NULL) {
		AR0233_ERR("missing deserializer dev handle\n");
		goto error;
	}
	if (dser_i2c->dev.driver == NULL) {
		AR0233_ERR("missing deserializer driver\n");
		goto error;
	}

	priv->dser_dev = &dser_i2c->dev;
	priv->priv_deser = dev_get_drvdata(priv->dser_dev);
/********************************************************************/

/***************parse GMSL link device tree**************************/
	gmsl_link = of_get_child_by_name(node, "gmsl-link");
	if (gmsl_link == NULL) {
		AR0233_ERR("missing gmsl-link device node\n");
		err = -EINVAL;
		goto error;
	}

	err = of_property_read_string(gmsl_link, "src-csi-port", &str_value);
	if (err < 0) {
		AR0233_ERR("No src-csi-port found\n");
		goto error;
	}
	priv->g_ctx.src_csi_port = 
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	err = of_property_read_string(gmsl_link, "dst-csi-port", &str_value);
	if (err < 0) {
		AR0233_ERR("No dst-csi-port found\n");
		goto error;
	}
	//to do:there is only for 2x4 lane mode. In this mode, four mipi out phys are formed two ports.
	priv->g_ctx.dst_csi_port =
		(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

	err = of_property_read_string(gmsl_link, "csi-mode", &str_value);
	if (err < 0) {
		AR0233_ERR("No csi-mode found\n");
		goto error;
	}
	if (!strcmp(str_value, "1x4")) {
		priv->g_ctx.csi_mode = GMSL_CSI_1X4_MODE;
	} else if (!strcmp(str_value, "2x4")) {
		priv->g_ctx.csi_mode = GMSL_CSI_2X4_MODE;
	} else if (!strcmp(str_value, "4x2")) {
		priv->g_ctx.csi_mode = GMSL_CSI_4X2_MODE;
	} else if (!strcmp(str_value, "2x2")) {
		priv->g_ctx.csi_mode = GMSL_CSI_2X2_MODE;
	} else {
		AR0233_ERR("invalid csi mode\n");
		goto error;
	}

	err = of_property_read_string(gmsl_link, "serdes-csi-link", &str_value);
	if (err < 0) {
		AR0233_ERR("No serdes-csi-link found\n");
		goto error;
	}

	priv->g_ctx.serdes_csi_link =
		(!strcmp(str_value, "a")) ?
			GMSL_SERDES_CSI_LINK_A : GMSL_SERDES_CSI_LINK_B;

	err = of_property_read_u32(gmsl_link, "st-vc", &value);
	if (err < 0) {
		AR0233_ERR("No st-vc info\n");
		goto error;
	} 
	priv->g_ctx.st_vc = value;

	err = of_property_read_u32(gmsl_link, "vc-id", &value);
	if (err < 0) {
		AR0233_ERR("No vc-id info\n");
		goto error;
	}
	priv->g_ctx.dst_vc = value;

	err = of_property_read_u32(gmsl_link, "num-lanes", &value);
	if (err < 0) {
		AR0233_ERR("No num-lanes info\n");
		goto error;
	}
	priv->g_ctx.num_csi_lanes = value;

	priv->g_ctx.num_streams =
			of_property_count_strings(gmsl_link, "streams");
	if (priv->g_ctx.num_streams <= 0) {
		AR0233_ERR("No streams found\n");
		err = -EINVAL;
		goto error;
	}
	for (i = 0; i < priv->g_ctx.num_streams; i++) {
		of_property_read_string_index(gmsl_link, "streams", i,
						&str_array[i]);
		if (!str_array[i]) {
			AR0233_ERR("invalid stream info\n");
			goto error;
		}
		if (!strcmp(str_array[i], "raw12")) {
			priv->g_ctx.streams[i].st_data_type =
							GMSL_CSI_DT_RAW_12;
		} else if (!strcmp(str_array[i], "embed")) {
			priv->g_ctx.streams[i].st_data_type =
							GMSL_CSI_DT_EMBED;
		} else if (!strcmp(str_array[i], "ued-u1")) {
			priv->g_ctx.streams[i].st_data_type =
							GMSL_CSI_DT_UED_U1;
		} else if (!strcmp(str_array[i], "yuv-8")) {
			priv->g_ctx.streams[i].st_data_type =
							GMSL_CSI_DT_YUV422_8;
		}  
		else {
			AR0233_ERR("invalid stream data type\n");
			goto error;
		}
	}

	priv->g_ctx.s_dev = dev;

	AR0233_LOG("ar0233_board_setup successful\n");

	return 0;
/********************************************************************/
error:
	AR0233_ERR("board setup failed\n");
	return err;
}

static int ar0233_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct tegracam_device *tc_dev;
	struct ar0233 *priv;
	int err;

    AR0233_LOG("probing ar0233 sensor.\n");

    if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;
    
    priv = devm_kzalloc(dev, sizeof(struct ar0233), GFP_KERNEL);
	if (!priv) {
		AR0233_ERR("unable to allocate memory!\n");
		return -ENOMEM;
	}

	AR0233_LOG("alloc tegracam_device buff.\n");
    tc_dev = devm_kzalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

    priv->i2c_client = client;
	tc_dev->client = client;
    tc_dev->dev = dev;
    strncpy(tc_dev->name, "ar0233", sizeof(tc_dev->name));
    tc_dev->dev_regmap_config = &sensor_regmap_config;
    tc_dev->sensor_ops = &ar0233_common_ops;
    tc_dev->v4l2sd_internal_ops = &ar0233_subdev_internal_ops;
    tc_dev->tcctrl_ops = &ar0233_ctrl_ops;

	AR0233_LOG("register tegra camera driver.\n");
    err = tegracam_device_register(tc_dev);
	if (err) {
		AR0233_ERR("tegra camera driver registration failed\n");
		return err;
	}

    priv->tc_dev = tc_dev;
    priv->s_data = tc_dev->s_data;
    priv->subdev = &tc_dev->s_data->subdev;
	AR0233_LOG("set tegra camera private data.\n");
    tegracam_set_privdata(tc_dev, (void*)priv);
	AR0233_LOG("set tegra camera private data down.\n");

    err = ar0233_board_setup(priv);
	if (err) {
		AR0233_ERR("board setup failed\n");
		return err;
	}

/***********************GSML init and link****************************************/
	/***********************************************************************
	 * AR0233-------->serializer-------->deserializer-------->tegra camera
	 * So, we need to init deserializer first, then serializer.
	 ***********************************************************************/
	err = init_deserializer(priv->dser_dev, &priv->g_ctx);
	if (err) {
		AR0233_ERR("init deserialzer failed\n");
		return err;
	}

	err = init_serializer(priv->ser_prim_dev, &priv->g_ctx);
	if (err) {
		AR0233_ERR("init serialzer failed\n");
		return err;
	}

	err = setup_deserializer_link(priv->dser_dev, &priv->g_ctx);
	if (err) {
		AR0233_ERR("deserializer set up link failed\n");
		return err;
	}
/*********************************************************************************/
	return 0;
}

static int ar0233_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0233 *priv = (struct ar0233 *)s_data->priv;

	//ar0233_gmsl_serdes_reset(priv);

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}


static struct i2c_driver ar0233_i2c_driver = {
	.driver = {
		.name = "ar0233",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ar0233_of_match),
	},
	.probe = ar0233_probe,
	.remove = ar0233_remove,
	.id_table = ar0233_id,
};

static int __init ar0233_init(void)
{
	return i2c_add_driver(&ar0233_i2c_driver);
}

static void __exit ar0233_exit(void)
{
	i2c_del_driver(&ar0233_i2c_driver);
}

module_init(ar0233_init);
module_exit(ar0233_exit);

MODULE_DESCRIPTION("Media Controller driver for ON Semiconductor AR0233");
MODULE_AUTHOR("Alex_min <624843267@qq.com>");
MODULE_LICENSE("GPL v2");
