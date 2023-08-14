#ifndef _SERIALIZER_H_
#define _SERIALIZER_H_

#include <linux/regmap.h>
#include <media/gmsl-link.h>

typedef struct {
    struct i2c_client *i2c_client;
    struct regmap *i2cProgrammer;
} serializer;


/************************define for function*************************************************/
/**
 * @brief  init serializer device.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev            The serializer device handle.
 * @param  [in]  g_ctx          The @ref gmsl_link_ctx structure handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int init_serializer(struct device *dev, struct gmsl_link_ctx *g_ctx);

/**
 * @brief  set serializer control.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev            The serializer device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int set_serializer_ctl(struct device *dev, struct gmsl_link_ctx *g_ctx);

int set_serializer_cam(struct device *dev);
/**********************************************************************************************/

#endif