#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/camera_common.h>
#include <linux/module.h>
#include <media/gmsl-common.h>

#include "maxim96712.h"

#define DEBUG_LOG_LEVEL 1

#if DEBUG_LOG_LEVEL > 0
#define MAX96712_LOG(fmt, args...) printk(KERN_INFO "[MAX96712] " fmt, ##args)
#else
#define MAX96712_LOG(fmt, args...)
#endif

#define MAX96712_ERR(fmt, args...) printk(KERN_ERR "[MAX96712] " fmt, ##args)

DEFINE_MUTEX(__maxim96712_mutex);


static bool IsGMSL2Mode(const GMSLMode mode)
{
    if ((GMSL2_MODE_6GBPS == mode) ||   \
        (GMSL2_MODE_3GBPS == mode)) {
        return true;
    } else {
        return false;
    }
}
/******************************************use for i2c R/W************************************************************/
static NvMediaStatus ReadUint8(struct regmap *i2cProgrammer, uint16_t addr, uint8_t *val)
{
    int err = 0;
	u32 reg_val = 0;

	err = regmap_read(i2cProgrammer, addr, &reg_val);
    if (err!=0)
    {
        MAX96712_ERR("i2c Read error! addr:%04x=",addr);
        return WICRI_STATUS_BAD_PARAMETER;
    }
	*val = reg_val & 0xFF;

    return WICRI_STATUS_OK;
}

static NvMediaStatus WriteUint8(struct regmap *i2cProgrammer, uint16_t addr, uint8_t val)
{
    int err = 0;
	err = regmap_write(i2cProgrammer, addr, val);
    MAX96712_LOG("0x%04x,0x%02x ",addr,val);
    if (err != 0)
    {
        MAX96712_ERR("i2c Write error! addr:%04x val:%02x",addr,val);
        return WICRI_STATUS_BAD_PARAMETER;
    }
    usleep_range(500, 510);

    return WICRI_STATUS_OK;
}

/* Access register fields belong to a single register.
 * REG_READ_MODE: Register is read and specified field vals are unpacked into regBitFieldArg array.
 * REG_WRITE_MODE: Specified field vals from regBitFieldArg array are packed and written to register.
 * REG_READ_MOD_WRITE_MODE: Register is read, specified field vals in regBitFieldArg are modified
 *                          and written to register */
static NvMediaStatus sSingleRegAccessRegFieldQ(max96712 *priv, RegBitFieldAccessMode mode)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    RegBitFieldQ *regBit = &(priv->regBitFieldQ);
    uint8_t numFields = regBit->numRegBitFieldArgs;
    uint16_t regAddr = 0u;
    const RegBitFieldProp *regBitProp = NULL;
    uint8_t fieldMask = 0u;
    uint8_t regData = 0u;
    uint8_t i = 0u;
    uint8_t loop = 0u;

    if (numFields == 0u) {
        MAX96712_LOG("There are no signle register to operate");
        goto done;
    }

    regBitProp = &regBitFieldProps[regBit->name[0]];
    regAddr = regBitProp->regAddr;

    /* Check if msbPos and lsbPos are valid. */
    for (i = 0u; i < numFields; i++) {
        regBitProp = &regBitFieldProps[regBit->name[i]];
        if (regBitProp->lsbPos > regBitProp->msbPos) {
            MAX96712_ERR("Bad parameter\n");
            status = WICRI_STATUS_BAD_PARAMETER;
            goto done;
        }
    }

    if (mode == REG_READ_MOD_WRITE_MODE) {
        for (loop = 0u; loop < 10u; loop++) {
            status = ReadUint8(priv->i2cProgrammer, regAddr, &regData);
            if (status == WICRI_STATUS_OK)
                break;
            usleep_range(10, 10);
        }

        if (status != WICRI_STATUS_OK) {
            MAX96712_ERR("Register I2C read failed with status regaddr:0x%04x status:%d ", regAddr, status);
            goto done;
        }
     }

    for (i = 0u; i < numFields; i++) {
        regBitProp = &regBitFieldProps[regBit->name[i]];
        fieldMask = (1u << (regBitProp->msbPos + 1u)) - (1u << regBitProp->lsbPos);
        /* Pack fieldVals for write*/
        regData &= ~fieldMask;
        regData |= ((regBit->val[i] << regBitProp->lsbPos) & fieldMask);

    }

    for (loop = 0u; loop < 10u; loop++) {
        status = WriteUint8(priv->i2cProgrammer, regAddr, regData);
        if (status == WICRI_STATUS_OK)
            break;
        usleep_range(10, 10);
    }

    if (regBitProp->delayNS != 0)
        usleep_range(regBitProp->delayNS, regBitProp->delayNS);
    else
        usleep_range(20, 20);

    if (status != WICRI_STATUS_OK)
        MAX96712_ERR("Register I2C write failed with status regaddr:0x%04x status:%d", regAddr, status);

done:
    return status;
}

/*
 * Check whether all register fields belong to the same register.
 */
static bool IsSingleRegister(RegBitFieldQ *regBit, uint8_t numFields)
{
    bool status = true;
    const RegBitFieldProp *regBitProp = NULL;
    uint16_t regAddr = 0u;
    uint8_t i;

    regBitProp = &regBitFieldProps[regBit->name[0]];
    regAddr = regBitProp->regAddr;

    for (i = 0u; i < numFields; i++) {
        regBitProp = &regBitFieldProps[regBit->name[i]];
        if (regBitProp->regAddr != regAddr) {
            status = false;
            goto done;
        }
    }

done:
    return status;
}

/* Access register fields.
 * REG_READ_MODE: Register is read and specified field vals are unpacked into regBitFieldArg array.
 * REG_WRITE_MODE: Specified field vals from regBitFieldArg array are packed and written to register.
 * REG_READ_MOD_WRITE_MODE: Register is read, specified field vals in regBitFieldArg are modified
 *                          and written to register */
static NvMediaStatus sAccessRegFieldQ(max96712 *priv, RegBitFieldAccessMode mode)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    RegBitFieldQ *regBit = &(priv->regBitFieldQ);
    uint8_t numFields = regBit->numRegBitFieldArgs;
    uint16_t regAddr = regBitFieldProps[regBit->name[0]].regAddr;
    const RegBitFieldProp *regBitProp = NULL;
    uint8_t fieldMask = 0u;
    uint8_t regData = 0;
    uint8_t i = 0u;
    uint8_t loop = 0u;

    if (numFields == 0u) {
        MAX96712_LOG("There are no register to operate\n");
        return status;
    }

    /*
     * use sSingleRegAccessRegFieldQ() if all register fields belong to
     * a single register
     */
    if (IsSingleRegister(regBit, numFields) && \
        ((mode == REG_WRITE_MODE) || (mode == REG_READ_MOD_WRITE_MODE))) {
        return sSingleRegAccessRegFieldQ(priv, mode);
    }

    /* Check if all the supplied fields belongs to same register addr.
     * Check if msbPos and lsbPos are valid. */
    for (i = 0u; i < numFields; i++) {
        regAddr = regBitFieldProps[regBit->name[i]].regAddr;
        regBitProp = &regBitFieldProps[regBit->name[i]];
        if ((regAddr != regBitProp->regAddr) ||
            (regBitProp->lsbPos > regBitProp->msbPos)) {
            MAX96712_ERR("Bad parameter: invalid register bit specification");
            return WICRI_STATUS_BAD_PARAMETER;
        }

        if ((mode == REG_READ_MODE) || (mode == REG_READ_MOD_WRITE_MODE)) {
            for (loop = 0u; loop < 10u; loop++) {
                status = ReadUint8(priv->i2cProgrammer, regAddr, &regData);
                if (status == WICRI_STATUS_OK) {
                    break;
                }
                usleep_range(10, 10);
            }

            if (status != WICRI_STATUS_OK) {
                MAX96712_ERR("I2C register read failed with status regaddr:0x%04x status:%d", regAddr, status);
                return status;
            }

            usleep_range(20, 20);
        }
        regBitProp = &regBitFieldProps[regBit->name[i]];
        fieldMask = (1u << (regBitProp->msbPos + 1u)) - (1u << regBitProp->lsbPos);
        if (mode == REG_READ_MODE) {
            /* Unpack fieldVals */
            regBit->val[i] = ((regData & fieldMask) >> (regBitProp->lsbPos));
        } else {
            /* Pack fieldVals for write*/
            regData &= ~fieldMask;
            regData |= ((regBit->val[i] << regBitProp->lsbPos) & fieldMask);
        }

        if (mode != REG_READ_MODE) {
            for (loop = 0u; loop < 10u; loop++) {
                status = WriteUint8(priv->i2cProgrammer, regAddr, regData);
                if (status == WICRI_STATUS_OK)
                    break;
                usleep_range(10, 10);
            }

            if (status != WICRI_STATUS_OK) {
                MAX96712_ERR("I2C register write failed with status regaddr:0x%04x status:%d", regAddr, status);
                return status;
            }
            if (regBitProp->delayNS != 0)
                usleep_range(regBitProp->delayNS, regBitProp->delayNS);
            else
                usleep_range(20, 20);
        }
    }

    return status;
}

static NvMediaStatus AddToRegFieldQ(max96712 *priv, RegBitField name, uint8_t val)
{
    
    uint8_t index = priv->regBitFieldQ.numRegBitFieldArgs;

    if (index == MAX96712_REG_MAX_FIELDS_PER_REG) {
        MAX96712_ERR("RegFieldQ full. Failed to add %d", (uint32_t)name);
        return WICRI_STATUS_ERROR;
    }

    if (name >= REG_FIELD_MAX) {
        MAX96712_ERR("RegFieldQ name over max. Failed to add %d", (uint32_t)name);
        return WICRI_STATUS_ERROR;
    }

    // MAX96712_LOG("MAX96712: Adding regField = %u, val = %02x to index %u in RegFieldQ\n",
    //         name,
    //         val,
    //         index);

    priv->regBitFieldQ.name[index] = name;
    priv->regBitFieldQ.val[index] = val;
    priv->regBitFieldQ.numRegBitFieldArgs = index + 1u;
    return WICRI_STATUS_OK;
}

static void ClearRegFieldQ(max96712 *priv)
{
    priv->regBitFieldQ.numRegBitFieldArgs = 0u;
}

static uint8_t ReadFromRegFieldQ(max96712 *priv, uint8_t index)
{
    uint8_t val = 0u;

    if (index >= priv->regBitFieldQ.numRegBitFieldArgs) {
        MAX96712_ERR("Bad parameter. Invalid index %d", (uint32_t)index);
        return 0u;
    }

    val = priv->regBitFieldQ.val[index];

    //MAX96712_LOG(" Read index %u from RegFieldQ. Val = %02x", index, val);

    return val;
}
/************************************************************************************************************/


/*******************************function used in this file***************************************************/

/************************************************************************************************************/


/*******************************function export to other files***********************************************/
void maxim96712_power_on(struct device *dev)
{
	max96712 *priv = dev_get_drvdata(dev);

	mutex_lock(&__maxim96712_mutex);
	if (false == priv->power_status) {
		if (priv->reset_gpio)
			gpio_set_value(priv->reset_gpio, 0);
        msleep(50);
		/*exit reset mode: XCLR */
		if (priv->reset_gpio) {
			gpio_set_value(priv->reset_gpio, 0);
			msleep(50);
			gpio_set_value(priv->reset_gpio, 1);
			msleep(50);
		}

		/* delay to settle reset */
		msleep(20);
	}

	priv->power_status = true;

	mutex_unlock(&__maxim96712_mutex);
}
EXPORT_SYMBOL(maxim96712_power_on);

int maxim96712_dev_info(struct device *dev)
{
	max96712 *priv = dev_get_drvdata(dev);
	NvMediaStatus status = WICRI_STATUS_ERROR;
	uint8_t devID = 0u;

	ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DEV_ID, 0, REG_READ_MODE);
	devID = ReadFromRegFieldQ(priv, 0u);

    if (MAX96712_DEV_ID != devID) {
        MAX96712_ERR("MAX96712: Device ID mismatch devid:%d", (uint32_t)devID);
        status = WICRI_STATUS_ERROR;
        goto done;
    }

done:
	return status;
}
EXPORT_SYMBOL(maxim96712_dev_info);


int maxim96712_get_enabled_links(struct device *dev, int *link_status)
{
    max96712 *priv = dev_get_drvdata(dev);
    int status = WICRI_STATUS_ERROR;
    int i;

    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_EN_A + i, 0u, REG_READ_MODE);
        *link_status |= (ReadFromRegFieldQ(priv, 0u) << i);
        MAX96712_LOG("Link %d status = %d", i, *link_status);
    }

    return status;
}
EXPORT_SYMBOL(maxim96712_get_enabled_links);

int maxim96712_enable_link(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    max96712 *priv = dev_get_drvdata(dev);
    NvMediaStatus status = WICRI_STATUS_ERROR;
    int serdes_csi_link = g_ctx->serdes_csi_link;
    int i;

    //All defined pairs links are enabled
    ClearRegFieldQ(priv);
    for (i = 0 ; i < MAX96712_MAX_NUM_LINK; i++) {
        if (GMSL_MODE_UNUSED == priv->gmslMode[i]) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_EN_A + i, 0u);
        } else {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_EN_A + i,  
                        MAX96712_IS_GMSL_LINK_SET(serdes_csi_link, i) ? 1u : 0u);
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    return status;
}
EXPORT_SYMBOL(maxim96712_enable_link);

int maxim96712_set_link_mode(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    max96712 *priv = dev_get_drvdata(dev);
    NvMediaStatus status = WICRI_STATUS_ERROR;
    int serdes_csi_link = g_ctx->serdes_csi_link;
    int i;

    // Set the link GMSL1 or GMSL2 mode
    ClearRegFieldQ(priv);
    for (i = 0 ; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(serdes_csi_link, i)) {
            if (IsGMSL2Mode(priv->gmslMode[i])) {
                ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_GMSL2_A + i,   
                                           (GMSL1_MODE == priv->gmslMode[i]) ? 0u : 1u);                
            }
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    //Set Link speed
    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(serdes_csi_link, i)) {
            MAX96712_LOG("SetLinkMode: Link is set and now setting the speed for %i", i);
            /*MAX96712_GMSL1_MODE       : 1
              MAX96712_GMSL2_MODE_6GBPS : 2
              MAX96712_GMSL2_MODE_3GBPS : 1*/
            if (GMSL_MODE_UNUSED != priv->gmslMode[i]) {
                ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_RX_RATE_PHY_A + i,  
                                          (GMSL2_MODE_6GBPS == priv->gmslMode[i]) ? 2u : 1u);
            }
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    return status;
}
EXPORT_SYMBOL(maxim96712_set_link_mode);

int maxim96712_one_shot_reset(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    max96712 *priv = dev_get_drvdata(dev);
    NvMediaStatus status = WICRI_STATUS_ERROR;
    int link = g_ctx->serdes_csi_link;
    int i;

    //Check whether link prots exist
    for (i = 0U; i < MAX96712_MAX_NUM_LINK; i++) {
        if (GMSL_MODE_UNUSED == priv->gmslMode[i]) {
            link &= ~(1 << i);
        }
    }

    if (MAX96712_LINK_NONE != link) {
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_RESET_ONESHOT, (uint8_t)link, REG_WRITE_MODE);
        //need 100ms delay
        usleep_range(100000, 100000);
    }

    return status;
}
EXPORT_SYMBOL(maxim96712_one_shot_reset);

int maxim96712_check_link_lock(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    max96712 *priv = dev_get_drvdata(dev);
    NvMediaStatus status = WICRI_STATUS_ERROR;
    int i, link_index;
    bool success = false;

    /* From Max96712 programming guide V1.1, typical link rebuilding time is 25 ~ 100ms
    * check the link lock in 100ms periodically
    * TODO : Intermittently the link lock takes more than 100ms. Check it with MAXIM */
    for (link_index = 0; link_index < MAX96712_MAX_NUM_LINK; link_index++) {
        if (MAX96712_IS_GMSL_LINK_SET(g_ctx->serdes_csi_link, link_index)) {
            MAX96712_LOG("%s %d\n", __func__, __LINE__);
            for (i = 0; i < 50; i++) {
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL2_LOCK_A + link_index,   \
                                                0, REG_READ_MODE);

                if (LINK_LOCKED == ReadFromRegFieldQ(priv, 0))  {
                    MAX96712_LOG("Link %u: GMSL2 link lock after %u ms", link_index, (i * 10u));
                    success = true;
                    break;
                }
                usleep_range(10000, 10000);
            }

            if (!success) {
                MAX96712_LOG("Link %u: GMSL2 link lock failed", link_index);
                status = WICRI_STATUS_ERROR;
                return status;
            }
        }
    }

    return status;
}
EXPORT_SYMBOL(maxim96712_check_link_lock);

int maxim96712_video_pipe_selection(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    max96712 *priv = dev_get_drvdata(dev);
    NvMediaStatus status = WICRI_STATUS_ERROR;
    //LinkPipelinePort ser_pipe;
    int i;

    //PipeY --> LineA --> Pipe0
    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(g_ctx->serdes_csi_link, i)) {
            if (IsGMSL2Mode(priv->gmslMode[i])) {
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_SEL_0 + i,    \
                                            4u * i + 1, \
                                            REG_READ_MOD_WRITE_MODE);   \
            }            
        }
    }

    //Turn on pipes
    ClearRegFieldQ(priv);
    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(g_ctx->serdes_csi_link, i))
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_EN_0 + i, 1u);
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    return status;
}
EXPORT_SYMBOL(maxim96712_video_pipe_selection);

int maxim96712_MIPI_phy_set(struct device *dev, u8 speed)
{
    max96712 *priv = dev_get_drvdata(dev);
    NvMediaStatus status = WICRI_STATUS_ERROR;
    PHYMode phyMode = priv->phyMode;
    u8 csi_mode = priv->csi_mode;
    u8 prebegin = 0, post = 0;
    u8 val;
    u16 addr;
    int i;

    if ((PHY_MODE_DPHY != phyMode) && (PHY_MODE_CPHY != phyMode)) {
        MAX96712_ERR("Invalid MIPI output phy mode\n");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if ((speed < 1) || (speed > 25)) {
        MAX96712_ERR("Invalid MIPI speed\n");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    //Set mipi out config
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_MIPI_OUT_CFG, csi_mode, REG_READ_MOD_WRITE_MODE);

    /* Set prebegin phase, post length and prepare for CPHY mode
     * This is a requirement for CPHY periodic calibration */
    if (PHY_MODE_CPHY == phyMode) {
        if (speed == 17) {
            /* TODO : This is a temporal solution to support the previous platform
             * This will be updated once CPHY calibration logic in RCE updated
             */
            /* t3_prebegin = (63 + 1) * 7 = 448 UI
             * Bit[6:2] = t3_post = (31 + 1) * 7 = 224 UI
             * Bit[1:0] = t3_prepare = 86.7ns
             */
            prebegin = 0x3F;
            post = 0x7F;
        } else {
            /* t3_prebegin = (19 + 1) * 7 = 140 UI
             * Bit[6:2] = t3_post = (31 + 1) * 7 = 224 UI
             * Bit[1:0] = t3_prepare = 40ns
             */
            prebegin = 0x13;
            post = 0x7c;
        }
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_T_T3_PREBEGIN, prebegin, REG_READ_MOD_WRITE_MODE);
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_T_T3_POST_PREP, post, REG_READ_MOD_WRITE_MODE);
    }

    /* Mapping data lanes Port A */
    val = (csi_mode == MAX96712_CSI_MODE_4X2) ? 0x44 : 0xE4;
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_MIPI_PHY_MAP_A, val, REG_READ_MOD_WRITE_MODE);

    /* Mapping data lanes Port B */
    val = (csi_mode == MAX96712_CSI_MODE_4X2) ? 0x44 : 0xE4;
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_MIPI_PHY_MAP_B, val, REG_READ_MOD_WRITE_MODE);

    /* Set CSI2 lane count per Phy */
    for (i = 0; i < MAX96712_MAX_NUM_PHY; i++) {
        val = (priv->phylines[i] -1) << 6;
        val |= (phyMode == PHY_MODE_CPHY) ? (1 << 5) : 0;
        addr = 0x090A + (i * 0x40);
        MAX96712_ERR("0x090A use other interface\n");
        WriteUint8(priv->i2cProgrammer, addr, val);
    }

    /* Put all Phys in standby mode */
    val = 0xF0;
    WriteUint8(priv->i2cProgrammer, 0x08a2, val);
    //ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_MIPI_PHY_EN, val, REG_READ_MOD_WRITE_MODE);


    WriteUint8(priv->i2cProgrammer, 0x1C00, 0xF4);
    /* Set MIPI speed */
    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        //val |= ((1 << 5) | speed);
        val = 0x28;
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_PHY0_SPEED + i, val, REG_READ_MOD_WRITE_MODE);
    }
    WriteUint8(priv->i2cProgrammer, 0x1C00, 0xF5);

    /* Deskew is enabled if MIPI speed is faster than or equal to 1.5GHz */
    if ((PHY_MODE_DPHY == phyMode) && (speed >= 15)) {
        addr = 0x0903;
        val = 0x97; /* enable the initial deskew with 8 * 32K UI */
        for (i = 0; i < MAX96712_MAX_NUM_PHY; i++) {
            addr = (addr & 0xff00) + ((addr + 0x40) & 0xff) ;
            WriteUint8(priv->i2cProgrammer, addr, val);
        }
    }
    
    return status;
}
EXPORT_SYMBOL(maxim96712_MIPI_phy_set);

int maxim96712_video_map_to_mipi_ctl(struct device *dev, struct gmsl_link_ctx *g_ctx)
{
    max96712 *priv = dev_get_drvdata(dev);
    NvMediaStatus status = WICRI_STATUS_ERROR;

    //test
    WriteUint8(priv->i2cProgrammer, 0x090B, 0x07);
    WriteUint8(priv->i2cProgrammer, 0x092D, 0x15);

    WriteUint8(priv->i2cProgrammer, 0x090D, 0x1E);
    WriteUint8(priv->i2cProgrammer, 0x090E, 0x1E);

    WriteUint8(priv->i2cProgrammer, 0x090F, 0x00);
    WriteUint8(priv->i2cProgrammer, 0x0910, 0x00);

    WriteUint8(priv->i2cProgrammer, 0x0911, 0x01);
    WriteUint8(priv->i2cProgrammer, 0x0912, 0x01);

    status = 0;

    return status;
}
EXPORT_SYMBOL(maxim96712_video_map_to_mipi_ctl);


void test_default(struct device *dev)
{
    max96712 *priv = dev_get_drvdata(dev);

    // WriteUint8(priv->i2cProgrammer, 0x06, 0x11);
    // WriteUint8(priv->i2cProgrammer, 0x10, 0x22);
    // WriteUint8(priv->i2cProgrammer, 0x18, 0x01);
    usleep_range(10000, 10000);
    //mipi set
    WriteUint8(priv->i2cProgrammer, 0x8a0, 0x04);
    WriteUint8(priv->i2cProgrammer, 0x8a2, 0xf0);
    WriteUint8(priv->i2cProgrammer, 0x8a3, 0xe4);
    WriteUint8(priv->i2cProgrammer, 0x8a4, 0xe4);
    WriteUint8(priv->i2cProgrammer, 0x90a, 0xc0);
    WriteUint8(priv->i2cProgrammer, 0x94a, 0xc0);
    WriteUint8(priv->i2cProgrammer, 0x98a, 0xc0);
    WriteUint8(priv->i2cProgrammer, 0x9ca, 0xc0);
    WriteUint8(priv->i2cProgrammer, 0x1c00, 0xf4);
    WriteUint8(priv->i2cProgrammer, 0x1d00, 0xf4);
    WriteUint8(priv->i2cProgrammer, 0x1e00, 0xf4);
    WriteUint8(priv->i2cProgrammer, 0x1f00, 0xf4);
    WriteUint8(priv->i2cProgrammer, 0x415, 0x28);
    WriteUint8(priv->i2cProgrammer, 0x418, 0x28);
    WriteUint8(priv->i2cProgrammer, 0x41b, 0x28);
    WriteUint8(priv->i2cProgrammer, 0x41e, 0x28);
    WriteUint8(priv->i2cProgrammer, 0x1c00, 0xf5);
    WriteUint8(priv->i2cProgrammer, 0x1d00, 0xf5);
    WriteUint8(priv->i2cProgrammer, 0x1e00, 0xf5);
    WriteUint8(priv->i2cProgrammer, 0x1f00, 0xf5); 

}
EXPORT_SYMBOL(test_default);
/************************************************************************************************************/
static const struct i2c_device_id max96712_id[] = {
	{ "max96712", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max96712_id);

const struct of_device_id max96712_of_match[] = {
	{ .compatible = "maxim,max96712", },
	{ },
};

static struct regmap_config max96712_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

int maxim96712_set_i2c_ctl_port(max96712 *priv)
{
    int status = 0;
    I2CPortMAX96712 i2cPort = priv->i2cPort;
    int i;

    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(priv->linkMask, i)) {
            if (GMSL1_MODE == priv->gmslMode[i]) {
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_I2C_PORT_GMSL1_PHY_A + i,    \
                                (i2cPort == MAX96712_I2CPORT_0) ? 0u : 1u,  \
                                REG_READ_MOD_WRITE_MODE);
            } else if (IsGMSL2Mode(priv->gmslMode[i])) {
                /* Disable connection from both port 0/1 */
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DIS_REM_CC_A + i,
                                             0x3u,
                                             REG_READ_MOD_WRITE_MODE);

                /* Select port 0 or 1 over the link */
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SEC_XOVER_SEL_PHY_A + i,
                                             (MAX96712_I2CPORT_0 == i2cPort) ? 0u : 1u,
                                             REG_READ_MOD_WRITE_MODE);

                /* Enable connection from port 0 or 1 */
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DIS_REM_CC_A + i,
                                             (MAX96712_I2CPORT_0 == i2cPort) ? 2u : 1u,
                                             REG_READ_MOD_WRITE_MODE);
            }
        }

        /* Update I2C slave timeout */
        if (i2cPort == MAX96712_I2CPORT_0) {
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SLV_TO_P0_A + i,
                                         0x5, /* 16 ms timeout. This value is less than I2C_INTREG_SLV_0_TO */
                                         REG_READ_MOD_WRITE_MODE);
        } else if (i2cPort == MAX96712_I2CPORT_1)  {
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SLV_TO_P1_A + i,
                                         0x5, /* 16 ms timeout. This value is less than I2C_INTREG_SLV_1_TO */
                                         REG_READ_MOD_WRITE_MODE);
        } else {
            return -EINVAL;
        }
    }

    return status;
}

int read_property_u32(struct device_node *node, const char *name, u32 *value)
{
	const char *str;
	int err = 0;

	err = of_property_read_string(node, name, &str);
	if (err)
		return -ENODATA;

	err = kstrtou32(str, 10, value);
	if (err)
		return -EFAULT;

	return 0;
}

int max96712_parse_dt(struct i2c_client *client, max96712 *priv)
{
	struct device_node *node = client->dev.of_node;
	const struct of_device_id *match;
	const char *str_value;
    u32 value;
	const char *gmsl_mode[MAX96712_MAX_NUM_LINK];
	int err;
	int i;

	if (!node)
		return -EINVAL;
	
	match = of_match_device(max96712_of_match, &client->dev);
	if (!match) {
		MAX96712_ERR("Failed to find matching dt id\n");
		return -EFAULT;
	}

	/*protected resource*/
	mutex_lock(&priv->lock);
/***********************set deserializer csi output mode**************************/
	err = of_property_read_string(node, "csi-mode", &str_value);
	if (err < 0) {
		MAX96712_ERR("csi-mode property not found\n");
		return err;
	}
	if (!strcmp(str_value, "2x4")) {
		priv->csi_mode = MAX96712_CSI_MODE_2X4;
        for (i = 0; i < MAX96712_MAX_NUM_PHY; i++) {
            priv->phylines[i] = 4;
        }
	} else if (!strcmp(str_value, "4x2")) {
		priv->csi_mode = MAX96712_CSI_MODE_4X2;
        for (i = 0; i < MAX96712_MAX_NUM_PHY; i++) {
            priv->phylines[i] = 2;
        }
	} else if (!strcmp(str_value, "1x4a_22")) {
		priv->csi_mode = MAX96712_CSI_MODE_1X4A_22;
        for (i = 0; i < MAX96712_MAX_NUM_PHY; i++) {
            priv->phylines[i] = 4;
        }
	} else if (!strcmp(str_value, "1x4b_22")) {
		priv->csi_mode = MAX96712_CSI_MODE_1X4B_22;
        for (i = 0; i < MAX96712_MAX_NUM_PHY; i++) {
            priv->phylines[i] = 2;
        }
	}
	else {
		MAX96712_ERR("invalid csi mode\n");
		return -EINVAL;
	}
/*********************************************************************************/

/***********************set deserializer link speed*******************************/
    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
		of_property_read_string_index(node, "gmsl-mode", i, &gmsl_mode[i]);
		if (!gmsl_mode[i]) {
			MAX96712_ERR("invalid GMSL mode info\n");
		}
		if (!strcmp(gmsl_mode[i], "gmsl2-6gbps")) {
			priv->gmslMode[i] = GMSL2_MODE_6GBPS;
		} else if (!strcmp(gmsl_mode[i], "gmsl2-3gbps")) {
			priv->gmslMode[i] = GMSL2_MODE_3GBPS;
		} else if (!strcmp(gmsl_mode[i], "gmsl1")) {
			priv->gmslMode[i] = GMSL1_MODE;
		} else {
			MAX96712_ERR( "No support's or None GMSL mode\n");
            priv->linkMask ^= (1 << i); 
			priv->gmslMode[i] = GMSL_MODE_UNUSED;
		}
	}
/*********************************************************************************/

/***********************set deserializer phy mode*********************************/
	err = of_property_read_string(node, "phy-mode", &str_value);
	if (err) {
		MAX96712_ERR("%s: use default phy mode DPHY\n", __func__);
		priv->phyMode = PHY_MODE_DPHY;
	} else {
		if (strcmp(str_value, "CPHY") == 0)
			priv->phyMode = PHY_MODE_CPHY;
		else if (strcmp(str_value, "DPHY") == 0)
			priv->phyMode = PHY_MODE_DPHY;
		else {
			MAX96712_ERR("%s: Invalid Phy mode\n", __func__);
			return -EINVAL;
		}
	}
/*********************************************************************************/

/***********************set i2c control port**************************************/
	err = read_property_u32(node, "i2cport", &value);
	if (err < 0) {
		MAX96712_ERR("i2cport property not found\n");
		return err;
	}
    switch (value)
    {
    case 0:
        priv->i2cPort = MAX96712_I2CPORT_0;
        break;
    case 1:
        priv->i2cPort = MAX96712_I2CPORT_1;
        break;
    case 2:
        priv->i2cPort = MAX96712_I2CPORT_2;
        break;
    default:
        MAX96712_ERR("not match i2cPort");
        break;
    }
/*********************************************************************************/

/***********************select CSI control port***********************************/
    err = read_property_u32(node, "txport", &value);
	if (err < 0) {
		MAX96712_ERR("txport property not found\n");
		return err;
	}
    switch (value)
    {
    case 0:
        priv->txPort  = MIPI_CSI_Controller_0;
        break;
    case 1:
        priv->txPort  = MIPI_CSI_Controller_1;
        break;
    case 2:
        priv->txPort  = MIPI_CSI_Controller_2;
        break;
    case 3:
        priv->txPort  = MIPI_CSI_Controller_3;
        break;
    default:
        MAX96712_ERR("not match txPort");
        break;
    }
/*********************************************************************************/

/***********************reset gpio************************************************/
	priv->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (priv->reset_gpio < 0) {
		MAX96712_ERR("reset-gpios not found %d\n", err);
		return err;
	}
/*********************************************************************************/

	mutex_unlock(&priv->lock);

	return err;	
}

int max96712_chip_init(max96712 *priv)
{
	NvMediaStatus status = WICRI_STATUS_ERROR;
	uint8_t devID = 0u;

	ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DEV_ID, 0, REG_READ_MODE);
	devID = ReadFromRegFieldQ(priv, 0u);

    if (MAX96712_DEV_ID != devID) {
        MAX96712_ERR("MAX96712: Device ID mismatch devid:%d", (uint32_t)devID);
        status = WICRI_STATUS_ERROR;
        goto done;
    }
done:
	return status;
}

static int max96712_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	max96712 *priv;
	int err;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		MAX96712_ERR("unable to allocate memory!\n");
		return -ENOMEM;
	}
	priv->i2c_client = client;
	priv->i2cProgrammer = devm_regmap_init_i2c(priv->i2c_client, &max96712_regmap_config);
	if (IS_ERR(priv->i2cProgrammer)) {
		MAX96712_ERR( "regmap init failed: %ld\n", PTR_ERR(priv->i2cProgrammer));
		return -ENODEV;
	}
	
	mutex_init(&priv->lock);

	priv->linkMask = (0xff & (1 << MAX96712_MAX_NUM_LINK)) - 1;
	err = max96712_parse_dt(client, priv);
	if (err) {
		MAX96712_ERR("unable to parse dt\n");
		return -EFAULT;
	}

    err = max96712_chip_init(priv);
    if (err) {
        MAX96712_ERR("unable to init max96712\n");
        return -EFAULT;
    }

    err = maxim96712_set_i2c_ctl_port(priv);
    if (err) {
        MAX96712_ERR("set i2c control port failed\n");
        return -EFAULT;
    }    

	//set default power down
	priv->power_status = false;

	dev_set_drvdata(&client->dev, priv);

	MAX96712_LOG("%s:  success\n", __func__);

	return err;
}

static int max96712_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver max96712_i2c_driver = {
	.driver = {
		.name = "max96712",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max96712_of_match),
	},
	.probe = max96712_probe,
	.remove = max96712_remove,
	.id_table = max96712_id,
};

static int __init max96712_init(void)
{
	return i2c_add_driver(&max96712_i2c_driver);
}

static void __exit max96712_exit(void)
{
	i2c_del_driver(&max96712_i2c_driver);
}

module_init(max96712_init);
module_exit(max96712_exit);

MODULE_DESCRIPTION("GMSL Deserializer driver max96712");
MODULE_AUTHOR("Alex_min <624843267@qq.com");
MODULE_LICENSE("GPL v2");
