#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>


//#include <linux/unistd.h>

struct simcom_power {
	int simcom_power_en ;
	int simcom_power_key ;
	int simcom_force_boot ;

	struct device_node *node ;
};

struct simcom_power *simcom ;


static int simcom_power_parse_dt(struct simcom_power *spdev)
{
	int err = 0 ;
	struct device_node *node = spdev->node;

	if(node == NULL)
	{
		return -EINVAL ;
	}
	
	spdev->simcom_power_en = of_get_named_gpio(node, "power-en", 0);
	if (spdev->simcom_power_en < 0) {
		err = spdev->simcom_power_en ;
		printk("simcom power-en not found %d\n", err);
		return err;
	}

	spdev->simcom_power_key = of_get_named_gpio(node, "power-key", 0);
	if (spdev->simcom_power_key < 0) {
		err = spdev->simcom_power_key ;
		printk("simcom power-key not found %d\n", err);
		return err;
	}

	spdev->simcom_force_boot = of_get_named_gpio(node, "force-boot", 0);
	if (spdev->simcom_force_boot < 0) {
		err = spdev->simcom_force_boot ;
		printk("simcom force-boot not found %d\n", err);
		return err;
	}

		
	return 0 ;
}

static int simcom_power_set_up(struct simcom_power *spdev)
{	
	int ret = 0 ;
	
	if(spdev->simcom_power_en)
	{
		ret = gpio_direction_output(spdev->simcom_power_en, 0);
		if(ret < 0)
		{
			printk("simcom gpio_direction_output failed! %d\n", ret);
			return ret ;
		}
	}
	msleep(10);
	
	if(spdev->simcom_power_key)
	{
		ret = gpio_direction_output(spdev->simcom_power_key, 0);
		if(ret < 0)
		{
			printk("simcom gpio_direction_output failed! %d\n", ret);
			return ret ;
		}
	}
	msleep(10);
	
	if(spdev->simcom_force_boot)
	{
		ret = gpio_direction_output(spdev->simcom_force_boot, 0);
		if(ret < 0)
		{
			printk("simcom gpio_direction_output failed! %d\n", ret);
			return ret ;
		}
		//gpio_set_value(spdev->simcom_force_boot, 0);
	}
	msleep(10);

	if(spdev->simcom_power_en)
		gpio_direction_output(spdev->simcom_power_en, 1);
	msleep(20);
	
	if(spdev->simcom_power_key)
		gpio_direction_output(spdev->simcom_power_key, 1);
	msleep(200);
	
	if(spdev->simcom_power_key)
		gpio_direction_output(spdev->simcom_power_key, 0);
	
	printk("simcom power on sequence Success!\n") ;
	
	return 0 ;
}

static int simcom_power_probe(struct platform_device *pdev)
{
	struct simcom_power *priv ;
	
	int ret = 0;
	printk("simcom simcom_power_probe\n") ;
	
	priv = devm_kzalloc(&pdev->dev, sizeof(struct simcom_power), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->node =  pdev->dev.of_node ;
	if(priv->node == NULL)
	{	
		printk("simcom priv->node null\n") ;
		return -ENXIO ;
	}
		
	ret = simcom_power_parse_dt(priv) ;
	if(ret < 0)
		return ret ;

	ret = simcom_power_set_up(priv) ;
	if(ret < 0)
		return ret ;

	simcom = priv ;

	printk("simcom Operation Success!\n") ;

	return 0;
}

static int simcom_power_remove(struct platform_device *pdev)
{
	int ret = 0 ;
	
	if(simcom == NULL)
		return 0 ;

	#if 0
	if(simcom->simcom_power_key)
		gpio_direction_output(simcom->simcom_power_key, 1);
	msleep(3500);
	
	if(simcom->simcom_power_key)
		gpio_direction_output(simcom->simcom_power_key, 0);
	#endif 
	ret = gpio_direction_output(simcom->simcom_power_en, 0);
	if(ret < 0)
	{
		printk("simcom gpio_direction_output failed! %d\n", ret);
		return ret ;
	}
	
	kfree(simcom) ;

	return 0 ;
}


const struct of_device_id simcom_power_of_match[] = {
	{ .compatible = "simcom,poweron", },
	{ },
};


static struct platform_driver simcom_power_driver = {
	.driver	= {
		.name	= "simcom7800_power",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(simcom_power_of_match),
	},
	.probe	= simcom_power_probe,
	.remove = simcom_power_remove,
};



module_platform_driver(simcom_power_driver);

MODULE_DESCRIPTION("LED driver for Simcom7800 power on");
MODULE_AUTHOR("Kexiong Deng");
MODULE_LICENSE("GPL");


