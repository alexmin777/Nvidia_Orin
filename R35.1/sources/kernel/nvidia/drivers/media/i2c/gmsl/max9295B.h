#ifndef __MAX9295_H__
#define __MAX9295_H__
#include <media/gmsl-link.h>

#define MAX9295_ID                  0x000D

//register address
#define MAX9295_DEV_ADDR            0x0000
#define MAX9295_REG2                0x0002
#define MAX9295_GPIO0_A             0x02BE
#define MAX9295_GPIO0_B             0x02BF
#define MAX9295_FRONTTOP_0          0x0308
#define MAX9295_FRONTTOP_9          0x0311
#define MAX9295_FRONTTOP_6          0x0316
#define MAX9295_MIPI_RX0            0x0330
#define MAX9295_MIPI_RX1            0x0331
#define MAX9295_MIPI_RX2            0x0332
#define MAX9295_MIPI_RX3            0x0333

//register value
#define CHIP_MAX9295B               0x93

void max9295_power_on(struct device *dev);
int max9295_check_chip_ID(struct device *dev);
int max9295_set_proxy_addr(struct device *dev, struct gmsl_link_ctx *g_ctx);
int max9295_reset_cam(struct device *dev);
int max9295_mipi_rx_phy_set(struct device *dev, struct gmsl_link_ctx *g_ctx);
int max9295_mipi_rx_port_set(struct device *dev, struct gmsl_link_ctx *g_ctx);
int max9295_dt_route_to_pipeline(struct device *dev, struct gmsl_link_ctx *g_ctx);
#endif  /* __MAX9295_H__ */
