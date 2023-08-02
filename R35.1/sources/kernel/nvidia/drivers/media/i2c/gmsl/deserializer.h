#ifndef _DESERIALIZER_H_
#define _DESERIALIZER_H_

#include <linux/regmap.h>
#include <media/gmsl-link.h>

#include "gmsl_link.h"


typedef enum {
    PHY_MODE_INVALID,
    PHY_MODE_DPHY,
    PHY_MODE_CPHY,
} PHYMode;

typedef struct {
    struct i2c_client *i2c_client;
    struct regmap *i2cProgrammer;
} deserializer;

/************************define for function*************************************************/
/**
 * @brief  init deserializer device.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev            The deserializer device handle.
 * @param  [in]  g_ctx          The @ref gmsl_link_ctx structure handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int init_deserializer(struct device *dev, struct gmsl_link_ctx *g_ctx);

/**
 * @brief  set up  deserializer link.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev            The deserializer device handle.
 * @param  [in]  g_ctx          The @ref gmsl_link_ctx structure handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int setup_deserializer_link(struct device *dev, struct gmsl_link_ctx *g_ctx);
/**********************************************************************************************/
#endif