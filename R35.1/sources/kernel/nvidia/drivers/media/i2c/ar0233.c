#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/gmsl-common.h>

#include "ar0233.h"

static int ar0233_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

}

static int ar0233_remove(struct i2c_client *client)
{

}
static const struct i2c_device_id ar0233_id[] = {
	{ "ar0233", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ar0233_id);

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
MODULE_AUTHOR("alex.min <624843267@qq.com>");
MODULE_LICENSE("GPL v2");