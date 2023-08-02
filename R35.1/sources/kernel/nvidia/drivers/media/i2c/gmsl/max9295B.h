#ifndef __MAX9295_H__
#define __MAX9295_H__
#include <media/gmsl-link.h>

//register address
#define MAX9295_ID                  0x0D
#define MAX9295_DEV_ADDR            0x00

//register value
#define CHIP_MAX9295B               0x93

void max9295_power_on(struct device *dev);
int max9295_check_chip_ID(struct device *dev);
int max9295_set_proxy_addr(struct device *dev, struct gmsl_link_ctx *g_ctx);
#endif  /* __MAX9295_H__ */
