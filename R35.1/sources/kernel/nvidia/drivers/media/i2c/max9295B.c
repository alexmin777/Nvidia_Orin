#include <media/camera_common.h>
#include <linux/module.h>
#include <media/max9295B.h>

#define DEBUG_LOG_LEVEL 1

#if DEBUG_LOG_LEVEL > 0
#define MAX9295_LOGE(fmt, args...) printk(KERN_ERR "[MAX9255] " fmt, ##args)
#else
#define MAX9295_LOGE(fmt, args...)
#endif

struct max9295 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	//primary data
	__u32 def_addr;
};

static struct max9295 *prim_priv__;

static  struct regmap_config max9295_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

// static int max9295_write_reg(struct device *dev, u16 addr, u8 val)
// {
// 	struct max9295 *priv = dev_get_drvdata(dev);
// 	int err;

// 	err = regmap_write(priv->regmap, addr, val);
// 	if (err)
// 		MAX9295_LOGE("%s:i2c write failed, 0x%x = %x\n",
// 			__func__, addr, val);

// 	/* delay before next i2c command as required for SERDES link */
// 	usleep_range(100, 110);

// 	return err;
// }

static int max9295_read_reg(struct device *dev, u16 addr, u8 *val)
{
	struct max9295 *priv = dev_get_drvdata(dev);
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->regmap, addr, &reg_val);
	if (err)
		MAX9295_LOGE("%s:i2c write failed, 0x%x = %x\n",
			__func__, addr, reg_val);

	/* delay before next i2c command as required for SERDES link */
	usleep_range(100, 110);
	*val = reg_val & 0xff;

	return err;
}

static u8 check_ID(struct i2c_client *client)
{
	u8 val = 0;
	struct device *dev = &client->dev;

	max9295_read_reg(dev, MAX9295_ID, &val);

	return val;
}

int max9295_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct max9295 *priv;
	int ret = 0;
	struct device_node *np = client->dev.of_node;
	u8 chip_id = 0;

	MAX9295_LOGE("max9295 start probe!\n");

	priv = devm_kzalloc(&client->dev, sizeof(struct max9295), GFP_KERNEL);
	if (!priv) {
		MAX9295_LOGE("max9295 kzalloc failed!\n");
		return -ENOMEM;
	}

	priv->i2c_client = client;
	priv->regmap = devm_regmap_init_i2c(priv->i2c_client, &max9295_regmap_config);
	if (IS_ERR(priv->regmap)) {
		MAX9295_LOGE("regmap init failed: %ld\n", PTR_ERR(priv->regmap));
		return -ENODEV;
	}

	if (of_get_property(np, "is-prim-ser", NULL)) {
		if (prim_priv__) {
			MAX9295_LOGE("max9295 prim serializer already exist!\n");
			return -EEXIST;
		}

		ret = of_property_read_u32(np, "reg", &priv->def_addr);
		if (ret < 0) {
			MAX9295_LOGE("max9295 read reg failed!\n");
			return -EINVAL;
		}

		prim_priv__ = priv;
	}

	dev_set_drvdata(&client->dev, priv);

	chip_id = check_ID(priv->i2c_client);
	MAX9295_LOGE("max9295 ID = 0x%x\n", chip_id);

	MAX9295_LOGE("max9295 probe successful!\n");

	return ret;
}

static int max9295_remove(struct i2c_client *client)
{
	struct max9295 *priv;

	if (client != NULL) {
		priv = dev_get_drvdata(&client->dev);
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max9295_id[] = {
	{ "max9295", 0 },
	{ },
};

static const struct of_device_id max9295_of_match[] = {
	{ .compatible = "maxim,max9295", },
	{ },
};
MODULE_DEVICE_TABLE(of, max9295_of_match);
MODULE_DEVICE_TABLE(i2c, max9295_id);

static struct i2c_driver max9295_i2c_driver = {
	.driver = {
		.name = "max9295",
		.owner = THIS_MODULE,
	},
	.probe = max9295_probe,
	.remove = max9295_remove,
	.id_table = max9295_id,
};


static int __init max9295_init(void)
{
	return i2c_add_driver(&max9295_i2c_driver);
}

static void __exit max9295_exit(void)
{
	i2c_del_driver(&max9295_i2c_driver);
}

module_init(max9295_init);
module_exit(max9295_exit);

MODULE_DESCRIPTION("GMSL Serializer driver max9295B");
MODULE_AUTHOR("alex_min <624843267@qq.com.com>");
MODULE_LICENSE("GPL v2");
