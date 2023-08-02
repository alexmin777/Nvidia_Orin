#include "serializer.h"
//define for use which serializer
#define SER_MAX9295B
#ifdef SER_MAX9295B 
#include "max9295B.h"
#endif

#define DEBUG_LOG_LEVEL 1

#if DEBUG_LOG_LEVEL > 0
#define SER_LOG(fmt, args...) printk(KERN_INFO "[Serializer] " fmt, ##args)
#else
#define SER_LOG(fmt, args...)
#endif

#define SER_ERR(fmt, args...) printk(KERN_ERR "[Serializer] " fmt, ##args)


int set_dev_proxy_addr(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;

#ifdef SER_MAX9295B 
    err = max9295_set_proxy_addr(dev, g_ctx);
#endif

    return 0;
}

int check_ser_chip_info(struct device *dev)
{
    int err = 0;

#ifdef SER_MAX9295B 
    err = max9295_check_chip_ID(dev);
#endif

    return err;
}

int power_on_serializer(struct device *dev)
{
    int err = 0;

#ifdef SER_MAX9295B 
    max9295_power_on(dev);
#endif

    return err;
}

int init_serializer(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;

    err = power_on_serializer(dev);
	if (err) {
		SER_ERR("power failed\n");
		goto done;
	}

    err = check_ser_chip_info(dev);
	if (err) {
		SER_ERR("Check serializer chip ID failed\n");
		goto done;
	}

    err = set_dev_proxy_addr(dev, g_ctx);
	if (err) {
		SER_ERR("Change serializer address failed\n");
		goto done;
	}

done:
    return err;
}