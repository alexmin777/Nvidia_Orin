#include "deserializer.h"
//define for use which serializer
#define DER_MAXIM96712
#ifdef DER_MAXIM96712 
#include "maxim96712.h"
#endif

#define DEBUG_LOG_LEVEL 1

#if DEBUG_LOG_LEVEL > 0
#define DES_LOG(fmt, args...) printk(KERN_INFO "[deserializer] " fmt, ##args)
#else
#define DES_LOG(fmt, args...)
#endif

#define DES_ERR(fmt, args...) printk(KERN_ERR "[deserializer] " fmt, ##args)


//DEFINE_MUTEX(g_deserializer_mutex);
/****************************init deserializer********************************************/
int power_on_deserializer(struct device *dev)
{
#ifdef DER_MAXIM96712 
    maxim96712_power_on(dev);
#endif

    return 0;
}

int check_des_chip_info(struct device *dev)
{
    int err = 0;

#ifdef DER_MAXIM96712 
    err = maxim96712_dev_info(dev);
#endif

    return err;
}

int init_deserializer(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;

    err = power_on_deserializer(dev);
	if (err) {
		DES_ERR("power failed\n");
		goto done;
	}

    err = check_des_chip_info(dev);
	if (err) {
		DES_ERR("check deserializer chip failed\n");
		goto done;
	}

done:
    return err;
}
/********************************************************************************************/

/**********************************set up link***********************************************/
void check_port_link_status(struct device *dev, int *link_status)
{
#ifdef DER_MAXIM96712 
    maxim96712_get_enabled_links(dev, link_status);
#endif
}

int deserilaizer_enable_link(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;

#ifdef DER_MAXIM96712 
    err = maxim96712_enable_link(dev, g_ctx);
#endif

    return err;  
}

int deserilaizer_set_link_mode(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;

#ifdef DER_MAXIM96712 
    err = maxim96712_set_link_mode(dev, g_ctx);
#endif

    return err;
}

int deserilaizer_one_shot_reset(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;
#ifdef DER_MAXIM96712 
    err = maxim96712_one_shot_reset(dev, g_ctx);
#endif

    return err;
}

int deserilaizer_check_link_lock(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;
#ifdef DER_MAXIM96712 
    err = maxim96712_check_link_lock(dev, g_ctx);
#endif

    return err;
}

int setup_deserializer_link(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    int err = 0;
    int link_mask = 0;
    int link_status = 0;
    static bool first_init = true;

    if (false == first_init) {
        //Get current linked info
        check_port_link_status(dev, &link_mask);
        DES_LOG("Current link status: 0x%x\n", link_mask);

        //Confirm whether is linked
        link_status = (g_ctx->serdes_csi_link) & 0xff;
        if (link_status & link_mask) {
            DES_LOG("Have been linked\n");
            goto done;
        }

        first_init = false;
    }

    err = deserilaizer_enable_link(dev, g_ctx);
	if (err) {
		DES_ERR("Enable deserializer link failed\n");
		goto done;
	}

    err = deserilaizer_set_link_mode(dev, g_ctx);
	if (err) {
		DES_ERR("Set deserializer link mode failed\n");
		goto done;
	}

    err = deserilaizer_one_shot_reset(dev, g_ctx);
	if (err) {
		DES_ERR("Deserializer one-shot reset failed\n");
		goto done;
	}

    err = deserilaizer_check_link_lock(dev, g_ctx);
	if (err) {
		DES_ERR("Deserializer link locked failed\n");
		goto done;
	}

done:
    return err;
}
/**********************************************************************************************/