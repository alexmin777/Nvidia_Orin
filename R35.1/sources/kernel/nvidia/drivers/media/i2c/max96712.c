#define DEBUG
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/camera_common.h>
#include <linux/module.h>
#include <media/gmsl-common.h>
#include <media/max96712.h>
#include <media/max96712_priv.h>
#include "max96712_pg_setting.h"


static bool
IsGMSL2Mode(const GMSLModeMAX96712 mode)
{
    if ((mode == MAX96712_GMSL2_MODE_6GBPS) ||
        (mode == MAX96712_GMSL2_MODE_3GBPS)) {
        return true;
    } else {
        return false;
    }
}

static void
us_sleep(u_int64_t us)
{
    usleep_range(us,us);
}

static NvMediaStatus
AddToRegFieldQ(
    max96712 *priv,
    RegBitField name,
    uint8_t val)
{
    
    uint8_t index = priv->regBitFieldQ.numRegBitFieldArgs;

    if (index == MAX96712_REG_MAX_FIELDS_PER_REG) {
        LOG_ERROR("MAX96712: RegFieldQ full. Failed to add %d", (uint32_t)name);
        return WICRI_STATUS_ERROR;
    }

    if (name >= REG_FIELD_MAX) {
        LOG_ERROR("MAX96712: RegFieldQ name over max. Failed to add %d", (uint32_t)name);
        return WICRI_STATUS_ERROR;
    }

    LOG_INFO("MAX96712: Adding regField = %u, val = %u to index %u in RegFieldQ\n",
            name,
            val,
            index);

    priv->regBitFieldQ.name[index] = name;
    priv->regBitFieldQ.val[index] = val;
    priv->regBitFieldQ.numRegBitFieldArgs = index + 1u;
    return WICRI_STATUS_OK;
}

static NvMediaStatus
ReadUint8(struct regmap *i2cProgrammer,
				uint16_t addr, uint8_t *val)
{
    int err = 0;
	u32 reg_val = 0;

	err = regmap_read(i2cProgrammer, addr, &reg_val);
    if (err!=0)
    {
        LOG_ERROR("i2c Read error! addr:%04x=",addr);
        return WICRI_STATUS_BAD_PARAMETER;
    }
	*val = reg_val & 0xFF;
    return WICRI_STATUS_OK;
}

static NvMediaStatus
WriteUint8(struct regmap *i2cProgrammer,
				uint16_t addr, uint8_t val)
{
    int err = 0;
	err = regmap_write(i2cProgrammer, addr, val);
    LOG_INFO("MAX96712:0x%04x,0x%02x ",addr,val);
    if (err!=0)
    {
        LOG_ERROR("i2c Write error! addr:%04x val:%02x",addr,val);
        return WICRI_STATUS_BAD_PARAMETER;
    }
    us_sleep(500);
    return WICRI_STATUS_OK;
}

static NvMediaStatus
WriteArray(struct regmap *i2cProgrammer,
DevBlkCDII2CRegList *regList)
{
    int i;
    int status=WICRI_STATUS_OK;
    for ( i = 0; i < regList->numRegs; i++)
    {
        status = WriteUint8(i2cProgrammer,regList->regs[i].address,regList->regs[i].data);
        if (status != WICRI_STATUS_OK)
        {
            LOG_ERROR("write reglist Failed!");
            return status;
        }
        if (regList->regs[i].delayUsec!=0)
        {
            us_sleep(regList->regs[i].delayUsec);
        }
        us_sleep(100);
    }
    
    return status;
}

static void
ClearRegFieldQ(
    max96712 *priv)
{
    LOG_INFO("MAX96712: Clearing RegFieldQ");
    priv->regBitFieldQ.numRegBitFieldArgs = 0u;
}

static uint8_t
ReadFromRegFieldQ(
    max96712 *priv,
    uint8_t index)
{
    
    uint8_t val = 0u;

    if (index >= priv->regBitFieldQ.numRegBitFieldArgs) {
        LOG_ERROR("MAX96712: Bad parameter. Invalid index %d", (uint32_t)index);
        return 0u;
    }

    val = priv->regBitFieldQ.val[index];

    LOG_INFO("MAX96712: Read index %u from RegFieldQ. Val = %u", index, val);
    return val;
}

/* Access register fields belong to a single register.
 * REG_READ_MODE: Register is read and specified field vals are unpacked into regBitFieldArg array.
 * REG_WRITE_MODE: Specified field vals from regBitFieldArg array are packed and written to register.
 * REG_READ_MOD_WRITE_MODE: Register is read, specified field vals in regBitFieldArg are modified
 *                          and written to register */
static NvMediaStatus
sSingleRegAccessRegFieldQ(
    max96712 *priv,
    RegBitFieldAccessMode mode)
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
        LOG_INFO("MAX96712: Skipping sAccessRegFieldQ");
        goto done;
    }

    regBitProp = &regBitFieldProps[regBit->name[0]];
    regAddr = regBitProp->regAddr;

    /* Check if msbPos and lsbPos are valid. */
    for (i = 0u; i < numFields; i++) {
        regBitProp = &regBitFieldProps[regBit->name[i]];
        if (regBitProp->lsbPos > regBitProp->msbPos) {
            LOG_ERROR("MAX96712: Bad parameter");
            status = WICRI_STATUS_BAD_PARAMETER;
            goto done;
        }
    }

    if (mode == REG_READ_MOD_WRITE_MODE) {
        for (loop = 0u; loop < 10u; loop++) {
            status = ReadUint8(priv->i2cProgrammer,
                                               regAddr,
                                               &regData);
            if (status == WICRI_STATUS_OK) {
                break;
            }
            us_sleep(10);
        }

        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX96712: Register I2C read failed with status regaddr:0x%04x status:%d ", regAddr, status);
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
        status = WriteUint8(priv->i2cProgrammer,
                                            regAddr,
                                            regData);
        if (status == WICRI_STATUS_OK) {
            break;
        }
        us_sleep(10);
    }

    if (regBitProp->delayNS != 0) {
        us_sleep(regBitProp->delayNS);
    } else {
        us_sleep(20);
    }

    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: Register I2C write failed with status regaddr:0x%04x status:%d", regAddr, status);
    }

done:
    return status;
}


/*
 * Check whether all register fields belong to the same register.
 */
static bool
IsSingleRegister(
    RegBitFieldQ *regBit,
    uint8_t numFields)
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
static NvMediaStatus
sAccessRegFieldQ(
    max96712 *priv,
    RegBitFieldAccessMode mode)
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
        LOG_INFO("MAX96712: Skipping sAccessRegFieldQ");
        return status;
    }

    /*
     * use sSingleRegAccessRegFieldQ() if all register fields belong to
     * a single register
     */
    if (IsSingleRegister(regBit, numFields) &&
        ((mode == REG_WRITE_MODE) ||
        (mode == REG_READ_MOD_WRITE_MODE))) {
        return sSingleRegAccessRegFieldQ(priv, mode);
    }

    /* Check if all the supplied fields belongs to same register addr.
     * Check if msbPos and lsbPos are valid. */
    for (i = 0u; i < numFields; i++) {
        regAddr = regBitFieldProps[regBit->name[i]].regAddr;
        regBitProp = &regBitFieldProps[regBit->name[i]];
        if ((regAddr != regBitProp->regAddr) ||
            (regBitProp->lsbPos > regBitProp->msbPos)) {
            LOG_ERROR("MAX96712: Bad parameter: invalid register bit specification");
            return WICRI_STATUS_BAD_PARAMETER;
        }

        if ((mode == REG_READ_MODE) || (mode == REG_READ_MOD_WRITE_MODE)) {
            for (loop = 0u; loop < 10u; loop++) {
                status = ReadUint8(priv->i2cProgrammer,
                                                   regAddr,
                                                   &regData);
                if (status == WICRI_STATUS_OK) {
                    break;
                }
                us_sleep(10);
            }

            if (status != WICRI_STATUS_OK) {
                LOG_ERROR("MAX96712: I2C register read failed with status regaddr:0x%04x status:%d", regAddr, status);
                return status;
            }

            us_sleep(20);
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
                status = WriteUint8(priv->i2cProgrammer,
                                                    regAddr,
                                                    regData);
                if (status == WICRI_STATUS_OK) {
                    break;
                }
                us_sleep(10);
            }

            if (status != WICRI_STATUS_OK) {
                LOG_ERROR("MAX96712: I2C register write failed with status regaddr:0x%04x status:%d", regAddr, status);
                return status;
            }
            if (regBitProp->delayNS != 0) {
                us_sleep(regBitProp->delayNS);
            } else {
                us_sleep(20);
            }
        }
    }

    return status;
}

NvMediaStatus
MAX96712OneShotReset(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint8_t i = 0U;

    for (i = 0U; i < MAX96712_MAX_NUM_LINK; i++) {
        if (priv->ctx.gmslMode[i] == MAX96712_GMSL_MODE_UNUSED) {
            link &= ~(1 << i);
        }
    }

    if (link != MAX96712_LINK_NONE) {
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_RESET_ONESHOT, (uint8_t)link, REG_WRITE_MODE);

        us_sleep(100000);
    }

    return status;
}

static NvMediaStatus
EnableSpecificLinks(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint8_t i = 0u;

    /* Disable the link lock error report to avoid the false alarm */
    ClearRegFieldQ(priv);
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_ENABLE_LOCK,
                                 0u,
                                 REG_READ_MOD_WRITE_MODE);

    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (priv->ctx.gmslMode[i] == MAX96712_GMSL_MODE_UNUSED) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_EN_A + i, 0u);
        } else {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_EN_A + i,
                                      MAX96712_IS_GMSL_LINK_SET(link, i) ? 1u : 0u);
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    /* Make sure the link is locked properly before enabling the link lock signal */
    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            if (priv->ctx.gmslMode[i] == MAX96712_GMSL1_MODE) {
                status = MAX96712CheckLink(priv, link, MAX96712_LINK_LOCK_GMSL1_CONFIG, true);
            } else {
                status = MAX96712CheckLink(priv, link, MAX96712_LINK_LOCK_GMSL2, true);
            }
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }
    }

    /* Enable the link lock error report */
    if (link != MAX96712_LINK_NONE) {
        ClearRegFieldQ(priv);
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_ENABLE_LOCK,
                                     1u,
                                     REG_READ_MOD_WRITE_MODE);
    }

    return status;
}

static NvMediaStatus
SetLinkMode(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint8_t i = 0u;

    /* Set GMSL mode */
    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            LOG_INFO("SetLinkMode: Setting Link for %d", i);
            if (priv->ctx.gmslMode[i] != MAX96712_GMSL_MODE_UNUSED) {
                ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_GMSL2_A + i,
                                          (priv->ctx.gmslMode[i] == MAX96712_GMSL1_MODE) ? 0u : 1u);
            }
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    /* Set Link speed */
    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            LOG_INFO("SetLinkMode: Link is set and now setting the speed for %i", i);
            /*MAX96712_GMSL1_MODE     : 1
            MAX96712_GMSL2_MODE_6GBPS : 2
            MAX96712_GMSL2_MODE_3GBPS : 1*/
            if (priv->ctx.gmslMode[i] != MAX96712_GMSL_MODE_UNUSED) {
                ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_RX_RATE_PHY_A + i,
                                          (priv->ctx.gmslMode[i] == MAX96712_GMSL2_MODE_6GBPS) ? 2u : 1u);
            }
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    return status;
}

static NvMediaStatus
EnablePeriodicAEQ(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ClearRegFieldQ(priv);
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_AEQ_PHY_A + i,
                                      1u);
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_PERIODIC_AEQ_PHY_A + i,
                                      1u);
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_EOM_PER_THR_PHY_A + i,
                                      0x10);
            ACCESS_REG_FIELD_RET_ERR(REG_WRITE_MODE);

            LOG_INFO("MAX96712: Enable periodic AEQ on Link %d\n", i);
            us_sleep(10000);
        }
    }

    return status;
}

static NvMediaStatus
SetDefaultGMSL1HIMEnabled(
    max96712 *priv,
    LinkMAX96712 link,
    uint8_t step)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CReg max96712_defaults_HIM_step0_regs[] = {
        /* GMSL1 - Turn on HIM */
        {0x0B06, 0xEF},
        /* GMSL1 - Enable reverse channel cfg and turn on local I2C ack */
        {0x0B0D, 0x81},
    };
    DevBlkCDII2CRegList max96712_defaults_HIM_step0 = {
        .regs = max96712_defaults_HIM_step0_regs,
        .numRegs = I2C_ARRAY_SIZE(max96712_defaults_HIM_step0_regs),
    };
    DevBlkCDII2CReg max96712_defaults_HIM_step1_regs[] = {
        /* GMSL1 - Turn off HIM */
        {0x0B06, 0x6F},
        /* GMSL1 - Enable manual override of reverse channel pulse length */
        {0x14C5, 0xAA},
        /* GMSL1 - Enable manual override of reverse channel rise fall time setting */
        {0x14C4, 0x80},
        /* GMSL1 - Tx amplitude manual override */
        {0x1495, 0xC8},
    };
    DevBlkCDII2CRegList max96712_defaults_HIM_step1 = {
        .regs = max96712_defaults_HIM_step1_regs,
        .numRegs = I2C_ARRAY_SIZE(max96712_defaults_HIM_step1_regs),
    };
    DevBlkCDII2CReg max96712_defaults_HIM_step2_regs[] = {
        /* Enable HIM */
        {0x0B06, 0xEF},
        /* Manual override of reverse channel pulse length */
        {0x14C5, 0x40},
        /* Manual override of reverse channel rise fall time setting */
        {0x14C4, 0x40},
        /* TxAmp manual override */
        {0x1495, 0x69},
    };
    DevBlkCDII2CRegList max96712_defaults_HIM_step2 = {
        .regs = max96712_defaults_HIM_step2_regs,
        .numRegs = I2C_ARRAY_SIZE(max96712_defaults_HIM_step2_regs),
    };
    DevBlkCDII2CRegList *stepHIM = NULL;
    uint8_t i = 0u;

    if (step > 2u) {
        LOG_ERROR("MAX96712: Bad parameter. Step must be either 0, 1 or 2.");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            /* Update register offset */
            if (step == 0) {
                max96712_defaults_HIM_step0_regs[0].address += (i << 8);
                max96712_defaults_HIM_step0_regs[1].address += (i << 8);
            } else if (step == 1) {
                max96712_defaults_HIM_step1_regs[0].address += (i << 8);
                max96712_defaults_HIM_step1_regs[1].address += (i << 8);
                max96712_defaults_HIM_step1_regs[2].address += (i << 8);
                max96712_defaults_HIM_step1_regs[3].address += (i << 8);
            } else {
                max96712_defaults_HIM_step2_regs[0].address += (i << 8);
                max96712_defaults_HIM_step2_regs[1].address += (i << 8);
                max96712_defaults_HIM_step2_regs[2].address += (i << 8);
                max96712_defaults_HIM_step2_regs[3].address += (i << 8);
            }

            stepHIM = (step == 0) ? &max96712_defaults_HIM_step0 :
                      ((step == 1) ? &max96712_defaults_HIM_step1 : &max96712_defaults_HIM_step2);

            status = WriteArray(priv->i2cProgrammer, stepHIM);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }
    }

    return status;
}

static NvMediaStatus
EnablePacketBasedControlChannel(
    max96712 *priv,
    LinkMAX96712 link,
    bool enable)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CReg ctrlChannelReg = {0x0B08, 0x25};
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ctrlChannelReg.address += (i << 8);

            if (!enable) {
                ctrlChannelReg.data = 0x21;
            }

            status = WriteUint8(priv->i2cProgrammer,
                                ctrlChannelReg.address,
                                ctrlChannelReg.data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
            us_sleep(10000);
        }
    }

    return status;
}

static NvMediaStatus
EnableDoublePixelMode(
    max96712 *priv,
    LinkMAX96712 link,
    DataTypeMAX96712 dataType,
    const bool embDataType,
    bool isSharedPipeline)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    TxPortMAX96712 txPort;
    DevBlkCDII2CReg disPktDetectorReg = {0x0100, 0x13};
    DevBlkCDII2CReg altModeArrReg = {0x0933, 0x01}; /* ALT_MEM_MAP12 = 1 on Ctrl 0 */
    uint8_t i = 0u;

    
    txPort = priv->ctx.txPort;

    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_BPP8DBL_4 + i,
                                      1u);
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_BPP8DBL_MODE_4 + i,
                                      1u);
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    altModeArrReg.address += (txPort - MAX96712_TXPORT_PHY_C) * 0x40;

    ClearRegFieldQ(priv);
    if ((dataType == MAX96712_DATA_TYPE_RAW12) &&
                     isSharedPipeline) {
        if (embDataType) {
            // In cases where EMB and Pix data share the same pipeline enable ALT2_MEM_MAP8
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_ALT2_MEM_MAP8_PHY0 + txPort, 1u);
        } else {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_ALT_MEM_MAP12_PHY0 + txPort, 1u);
        }
    } else {
        if (dataType == MAX96712_DATA_TYPE_RAW12) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_ALT_MEM_MAP12_PHY0 + txPort, 1u);
        } else if (dataType == MAX96712_DATA_TYPE_RAW10) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_ALT_MEM_MAP10_PHY0 + txPort, 1u);
        }

        if (embDataType == true) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_ALT_MEM_MAP8_PHY0 + txPort, 1u);
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            disPktDetectorReg.address =
                ((disPktDetectorReg.address & 0xFF00U) + (0x12U * i));
            status = WriteUint8(priv->i2cProgrammer,
                                                disPktDetectorReg.address,
                                                disPktDetectorReg.data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }

            disPktDetectorReg.address =
                (disPktDetectorReg.address & 0xFF00U) +
                (0x12U * (i + 4U)) +
                ((i != 0U) ? 0x6U : 0U);
            status = WriteUint8(priv->i2cProgrammer,
                                                disPktDetectorReg.address,
                                                disPktDetectorReg.data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }
    }

    return status;
}

/*
 * Bug 2182451: The below errors were observed intermittently in GMSL2 6Gbps link speed.
 *              To resolve it, adjust the Tx amplitude and timing parameters
 * CSI error(short or long line) is seen
 * Decoding error is seen on the deserializer
 * Link margin becomes bad
 */
static NvMediaStatus
ConfigTxAmpTiming(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    GMSLModeMAX96712 gmslMode = MAX96712_GMSL_MODE_INVALID;
    
    RevisionMAX96712 rev = priv->ctx.revision;
    DevBlkCDII2CReg adjTxAmpAndTimingArrRegs[] = {
        {0x1458, 0x28, 0x2701}, /* vth1 : Error channel power-down then delay 1ms*/
        {0x1459, 0x68, 0x2701},/* vth0 : + 104 * 4.7mV = 488.8 mV  then delay 1ms*/
        {0x143E, 0xB3, 0x2701},/* Error channel phase secondary timing adjustment  then delay 1ms*/
        {0x143F, 0x72, 0x2701}, /* Error channel phase primary timing adjustment  then delay 1ms*/
        {0x1495, 0xD2, 0x2701}, /* Reverse channel Tx amplitude to 180 mV  then delay 1ms*/
    };
    DevBlkCDII2CRegList adjTxAmpAndTimingArr = {
        .regs = adjTxAmpAndTimingArrRegs,
        .numRegs = I2C_ARRAY_SIZE(adjTxAmpAndTimingArrRegs),
    };
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            gmslMode = priv->ctx.gmslMode[i];

            if (!IsGMSL2Mode(gmslMode)) {
                LOG_INFO("MAX96712: Link %d: Tx amplitude is only required in GMSL2 mode\n", i);
                continue;
            }

            adjTxAmpAndTimingArrRegs[0].address += (i << 8);
            adjTxAmpAndTimingArrRegs[1].address += (i << 8);
            adjTxAmpAndTimingArrRegs[2].address += (i << 8);
            adjTxAmpAndTimingArrRegs[3].address += (i << 8);
            adjTxAmpAndTimingArrRegs[4].address += (i << 8);
            status = WriteArray(priv->i2cProgrammer, &adjTxAmpAndTimingArr);
            if (status != WICRI_STATUS_OK) {
                LOG_INFO("MAX96712: Link %d: Failed to updte Tx amplitude\n", i);
                return status;
            }
            (void)rev;
            LOG_INFO("MAX96712 Rev %d: Link %d: ", rev, i);
            LOG_INFO("Tx amplitude 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", adjTxAmpAndTimingArrRegs[0].data,
                                                                             adjTxAmpAndTimingArrRegs[1].data,
                                                                             adjTxAmpAndTimingArrRegs[2].data,
                                                                             adjTxAmpAndTimingArrRegs[3].data,
                                                                             adjTxAmpAndTimingArrRegs[4].data);
        }
    }

    return WICRI_STATUS_OK;
}

static NvMediaStatus
UpdateVGAHighGain(
    max96712 *priv,
    LinkMAX96712 link,
    bool enable)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    GMSLModeMAX96712 gmslMode = MAX96712_GMSL_MODE_INVALID;
    
    DevBlkCDII2CReg VGAHighGainReg = {0x1418, 0x03};
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            gmslMode = priv->ctx.gmslMode[i];

            if (!IsGMSL2Mode(gmslMode)) {
                LOG_INFO("MAX96712: Link %d: VGAHighGain is valid in ONLY GMSL2 mode\n", i);
                continue;
            }
            VGAHighGainReg.address += (i << 8);
            VGAHighGainReg.data = (enable) ? 0x07 : 0x03;

            status = WriteUint8(priv->i2cProgrammer,
                                                VGAHighGainReg.address,
                                                VGAHighGainReg.data);
            if (status != WICRI_STATUS_OK) {
                LOG_INFO("MAX96712: Link %d: Failed to set VGAHighGain\n", i);
                return status;
            }
        }
    }

    return WICRI_STATUS_OK;
}

static NvMediaStatus
OverrideDataType(
    max96712 *priv,
    LinkMAX96712 link,
    LinkPipelineMapMAX96712 *linkPipelineMap)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint8_t bpp = 0u;
    uint8_t dataFormat = 0u;
    uint8_t i = 0u;

    /* Override is enabled only for pipes 0-3 */
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i) &&
            linkPipelineMap[i].isDTOverride) {
            switch (linkPipelineMap[i].dataType) {
                case MAX96712_DATA_TYPE_RAW10:
                    bpp = 0xA;         /* 10 bits per pixel */
                    dataFormat = 0x2B; /* raw10 */
                    break;
                case MAX96712_DATA_TYPE_RAW12:
                    bpp = 0xC;         /* 12 bits per pixel */
                    dataFormat = 0x2C; /* raw12 */
                    break;
                case MAX96712_DATA_TYPE_RAW16:
                    bpp = 0x10;        /* 16 bits per pixel */
                    dataFormat = 0x2E; /* raw16 */
                    break;
                case MAX96712_DATA_TYPE_RGB:
                    bpp = 0x18;        /* 24 bits per pixel */
                    dataFormat = 0x24; /* RGB */
                    break;
                case MAX96712_DATA_TYPE_YUV_8:
                    bpp = 0x10;        /* 16 bits per pixel */
                    dataFormat = 0x1E; /* YUV */
                    break;
                default:
                    LOG_ERROR("MAX96712: Bad parameter: Invalid data type");
                    return WICRI_STATUS_BAD_PARAMETER;
            }

            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_BPP_0 + i,
                                         bpp,
                                         REG_READ_MOD_WRITE_MODE);

            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_0 + i,
                                         dataFormat,
                                         REG_READ_MOD_WRITE_MODE);


            if (i == 1u) {
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_1_H,
                                             (dataFormat >> 4u),
                                             REG_READ_MOD_WRITE_MODE);
            } else if (i == 2u) {
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_BPP_2_H,
                                             (bpp >> 2u),
                                             REG_READ_MOD_WRITE_MODE);

                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_2_H,
                                             (dataFormat >> 2u),
                                             REG_READ_MOD_WRITE_MODE);
            }

            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_OVR_0_EN + i,
                                         1u,
                                         REG_READ_MOD_WRITE_MODE);

            if (priv->ctx.tpgEnabled &&
                priv->ctx.pipelineEnabled & (0x10 << i)) {
                /* Override BPP, DT for the pipeline 4 ~ 7 */
                if (i == 0U) {
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_BPP_4,
                                                 bpp,
                                                 REG_READ_MOD_WRITE_MODE);

                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_4,
                                                 dataFormat,
                                                 REG_READ_MOD_WRITE_MODE);
                } else if (i == 1U) {
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_BPP_5,
                                                 bpp,
                                                 REG_READ_MOD_WRITE_MODE);

                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_5_H,
                                                 (dataFormat >> 4U),
                                                 REG_READ_MOD_WRITE_MODE);

                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_5_L,
                                                 (dataFormat & 0xF),
                                                 REG_READ_MOD_WRITE_MODE);
                } else if (i == 2U) {
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_BPP_6_H,
                                                 (bpp >> 2U),
                                                 REG_READ_MOD_WRITE_MODE);

                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_BPP_6_L,
                                                 (bpp & 0x3),
                                                 REG_READ_MOD_WRITE_MODE);

                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_6_H,
                                                 (dataFormat >> 2U),
                                                 REG_READ_MOD_WRITE_MODE);

                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_6_L,
                                                 (dataFormat & 0x3),
                                                 REG_READ_MOD_WRITE_MODE);
                } else {
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_BPP_7,
                                                 bpp,
                                                 REG_READ_MOD_WRITE_MODE);

                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_DT_7,
                                                 dataFormat,
                                                 REG_READ_MOD_WRITE_MODE);
                }

                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_OVR_0_EN + i,
                                             0u,
                                             REG_READ_MOD_WRITE_MODE);
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SOFT_OVR_4_EN + i,
                                             1u,
                                             REG_READ_MOD_WRITE_MODE);
            }
        }
    }

    return status;
}

static NvMediaStatus
VideoPipelineSel(
    max96712 *priv,
    LinkMAX96712 link,
    LinkPipelineMapMAX96712 *linkPipelineMap)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            if (IsGMSL2Mode(priv->ctx.gmslMode[i])) {
                if (linkPipelineMap[i].isSinglePipeline) {
                    /* in case of single pipe Z from ser, select that for pipe in deser */
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_SEL_0 + i,
                                                 (4u * i) + 2u,
                                                 REG_READ_MOD_WRITE_MODE);
                } else {
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_SEL_0 + i,
                                                 4u * i,
                                                 REG_READ_MOD_WRITE_MODE);

                    if (linkPipelineMap[i].isEmbDataType) {
                        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_SEL_4 + i,
                                                     (4u * i) + 1u,
                                                     REG_READ_MOD_WRITE_MODE);
                    }
                }
            }
        }
    }

    /* Enable Pipelines*/
    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_EN_0 + i,
                                      1u);
            if (linkPipelineMap[i].isEmbDataType && !linkPipelineMap[i].isSinglePipeline) {
                ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_EN_4 + i,
                                          1u);
            }
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    return status;
}

static NvMediaStatus
SetPipelineMap(
    max96712 *priv,
    LinkMAX96712 link,
    LinkPipelineMapMAX96712 *linkPipelineMap)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    bool isEmbDataType = false;
    uint8_t vcID = 0u;
    uint8_t dataTypeVal = 0u;
    uint8_t i = 0u;
    uint8_t j = 0u;
    TxPortMAX96712 txPort;
    /* Two pipelines are one set to process raw12 and emb */
    DevBlkCDII2CReg mappingRawRegs[] = {
        /* Send RAW12 FS and FE from X to Controller 1 */
        {0x090B, 0x07},
        {0x092D, 0x00},
        /* For the following MSB 2 bits = VC, LSB 6 bits = DT */
        {0x090D, 0x2C},
        {0x090E, 0x2C},
        {0x090F, 0x00},
        {0x0910, 0x00},
        {0x0911, 0x01},
        {0x0912, 0x01},
    };
    DevBlkCDII2CRegList mappingRaw = {
        .regs = mappingRawRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingRawRegs),
    };
    DevBlkCDII2CReg mappingEmbRegs[] = {
        /* Send EMB8 from Y to Controller 1 with VC unchanged */
        {0x0A0B, 0x07},
        {0x0A2D, 0x00},
        /* For the following MSB 2 bits = VC, LSB 6 bits = DT */
        {0x0A0D, 0x12},
        {0x0A0E, 0x12},
        {0x0A0F, 0x00},
        {0x0A10, 0x00},
        {0x0A11, 0x01},
        {0x0A12, 0x01},
    };
    DevBlkCDII2CRegList mappingEmb = {
        .regs = mappingEmbRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingEmbRegs),
    };
    DevBlkCDII2CReg mappingEmbPipeZRegs[] = {
        /* Send EMB data from pipe Z to controller 1 */
        {0x0913, 0x12},
        {0x0914, 0x12},
    };
    DevBlkCDII2CRegList mappingEmbPipeZ = {
        .regs = mappingEmbPipeZRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingEmbPipeZRegs),
    };

    txPort = priv->ctx.txPort;
    if (linkPipelineMap == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null link pipeline map");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            isEmbDataType = linkPipelineMap[i].isEmbDataType;
            vcID = linkPipelineMap[i].vcID;

            /* Update Tx Port */
            if (linkPipelineMap[i].isSinglePipeline) {
                if (isEmbDataType) {
                    /*Enable 4 mappings FS, FE, PIX, EMB */
                    mappingRawRegs[0].data = 0x0F;
                    /* Map all 4 to controller specified by txPort */
                    mappingRawRegs[1].data = (txPort << 6u) | (txPort << 4u) | (txPort << 2u) | txPort;
                } else {
                    /*Enable 3 mappings FS, FE, PIX */
                    mappingRawRegs[0].data = 0x07;
                    /* Map all 3 to controller specified by txPort */
                    mappingRawRegs[1].data = (txPort << 4u) | (txPort << 2u) | txPort;
                }
            } else {
                mappingRawRegs[1].data = (txPort << 4u) | (txPort << 2u) | txPort;
                mappingEmbRegs[1].data = (txPort << 4u) | (txPort << 2u) | txPort;
            }

            if (isEmbDataType && !IsGMSL2Mode(priv->ctx.gmslMode[i])) {
                LOG_ERROR("MAX96712: Emb data type is valid for GMSL2 only");
                return WICRI_STATUS_ERROR;
            }

            if (isEmbDataType && linkPipelineMap[i].isDTOverride) {
                LOG_ERROR("MAX96712: Emb data type is not supported with dt override enabled");
                return WICRI_STATUS_ERROR;
            }

            switch (linkPipelineMap[i].dataType) {
                case MAX96712_DATA_TYPE_RAW10:
                    dataTypeVal = 0x2B;
                    break;
                case MAX96712_DATA_TYPE_RAW12:
                    dataTypeVal = 0x2C;
                    break;
                case MAX96712_DATA_TYPE_RAW16:
                    dataTypeVal = 0x2E;
                    break;
                case MAX96712_DATA_TYPE_RGB:
                    dataTypeVal = 0x24;
                    break;
                case MAX96712_DATA_TYPE_YUV_8:
                    dataTypeVal = 0x1E;
                    break;
                default:
                    LOG_ERROR("MAX96712: Bad parameter: Invalid data type");
                    return WICRI_STATUS_BAD_PARAMETER;
            }

            /* update offset */
            /* 4 mapping for data and 4 mapping for emb */
            mappingRawRegs[2].data = dataTypeVal;
            mappingRawRegs[3].data = (vcID << 6u) | dataTypeVal;

            mappingEmbPipeZRegs[1].data = (vcID << 6u) | 0x12;

            if (linkPipelineMap[i].isSinglePipeline) {
                /* If this is a single pipeline only map raw, no need to map emb data */
                mappingRawRegs[5].data = (vcID << 6u) | 0x0;
                mappingRawRegs[7].data = (vcID << 6u) | 0x1;
            } else {
                /* Change FS packet's DT to reserved for RAW pipeline if emb data is used */
                mappingRawRegs[5].data = (vcID << 6u) | (isEmbDataType ? 2u : 0u);
                /* Change FE packet's DT to reserved for RAW pipeline if emb data is used */
                mappingRawRegs[7].data = (vcID << 6u) | (isEmbDataType ? 3u : 1u);
                mappingEmbRegs[3].data = (vcID << 6u) | 0x12;
                mappingEmbRegs[5].data = (vcID << 6u) | 0x0;
                mappingEmbRegs[7].data = (vcID << 6u) | 0x1;
            }

            status = WriteArray(priv->i2cProgrammer, &mappingRaw);
            if (status != WICRI_STATUS_OK) {
                return status;
            }

            if (!linkPipelineMap[i].isSinglePipeline) {
                if (isEmbDataType) {
                    status = WriteArray(priv->i2cProgrammer, &mappingEmb);
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                }
            } else {
                if (isEmbDataType) {
                    status = WriteArray(priv->i2cProgrammer, &mappingEmbPipeZ);
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                }
            }
        }

        /* Update the reg addr for the next link */
        for (j = 0u; j < 8u; j++) {
            mappingRawRegs[j].address += 0x40;
            mappingEmbRegs[j].address += 0x40;
        }

        /* Update the reg addr for the next link */
        for (j = 0u; j < 2u; j++) {
            mappingEmbPipeZRegs[j].address += 0x40;
        }
    }

    return VideoPipelineSel(priv,
                            link,
                            linkPipelineMap);
}

static NvMediaStatus
SetYuvPipelineMap(
    max96712 *priv,
    LinkMAX96712 link,
    LinkPipelineMapMAX96712 *linkPipelineMap)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    bool isEmbDataType = false;
    uint8_t vcID = 0u;
    uint8_t dataTypeVal = 0u;
    uint8_t i = 0u;
    uint8_t j = 0u;
    TxPortMAX96712 txPort;
    DevBlkCDII2CReg mappingRawRegs[] = {
        /* Send Yuv FS and FE from Y to Controller 1 */
        {0x090B, 0x07},
        {0x092D, 0x00},
        /* For the following MSB 2 bits = VC, LSB 6 bits = DT */
        {0x090D, 0x1E},
        {0x090E, 0x1E},
        {0x090F, 0x00},
        {0x0910, 0x00},
        {0x0911, 0x01},
        {0x0912, 0x01},
    };
    DevBlkCDII2CRegList mappingRaw = {
        .regs = mappingRawRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingRawRegs),
    };

    txPort = priv->ctx.txPort;
    if (linkPipelineMap == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null link pipeline map");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            vcID = linkPipelineMap[i].vcID;

            /* Update Tx Port */
            if (linkPipelineMap[i].isSinglePipeline) {
                if (isEmbDataType) {
                    /*Enable 4 mappings FS, FE, PIX, EMB */
                    mappingRawRegs[0].data = 0x0F;
                    /* Map all 4 to controller specified by txPort */
                    mappingRawRegs[1].data = (txPort << 6u) | (txPort << 4u) | (txPort << 2u) | txPort;
                } else {
                    /*Enable 3 mappings FS, FE, PIX */
                    mappingRawRegs[0].data = 0x07;
                    /* Map all 3 to controller specified by txPort */
                    mappingRawRegs[1].data = (txPort << 4u) | (txPort << 2u) | txPort;
                }
            } else {
                mappingRawRegs[1].data = (txPort << 4u) | (txPort << 2u) | txPort;
            }

            switch (linkPipelineMap[i].dataType) {
                case MAX96712_DATA_TYPE_YUV_8:
                    dataTypeVal = 0x1E;
                    break;
                default:
                    LOG_ERROR("MAX96712: Bad parameter: Invalid data type");
                    return WICRI_STATUS_BAD_PARAMETER;
            }

            /* update offset */
            /* 4 mapping for data and 4 mapping for emb */
            mappingRawRegs[2].data = dataTypeVal;
            mappingRawRegs[3].data = (vcID << 6u) | dataTypeVal;


            /* If this is a single pipeline only map raw, no need to map emb data */
            mappingRawRegs[5].data = (vcID << 6u) | 0x0;
            mappingRawRegs[7].data = (vcID << 6u) | 0x1;


            status = WriteArray(priv->i2cProgrammer, &mappingRaw);
            if (status != WICRI_STATUS_OK) {
                return status;
            }

        }

        /* Update the reg addr for the next link */
        for (j = 0u; j < 8u; j++) {
            mappingRawRegs[j].address += 0x40;
        }

    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            if (IsGMSL2Mode(priv->ctx.gmslMode[i])) {
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_SEL_0 + i,
                                            4u * i + 1,
                                            REG_READ_MOD_WRITE_MODE);
            }
        }
    }

    /* Enable Pipelines*/
    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_EN_0 + i,
                                    1u);
        }
    }
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MOD_WRITE_MODE);

    return status;
}

static NvMediaStatus
SetPipelineMapTPG(
    max96712 *priv,
    uint8_t linkIndex,
    LinkPipelineMapMAX96712 *linkPipelineMap)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    TxPortMAX96712 txPort;
    bool PGGen0 = true;
    uint8_t vcID = 0u, dataTypeVal = 0u;
    uint8_t i = 0u;
    /* Two pipelines are one set to process raw12 and emb */
    DevBlkCDII2CReg mappingRawRegs[] = {
        /* Send RAW12 FS and FE from X to Controller 1 */
        {0x090B, 0x07},
        {0x092D, 0x00},
        /* For the following MSB 2 bits = VC, LSB 6 bits = DT */
        {0x090D, 0x2C},
        {0x090E, 0x2C},
        {0x090F, 0x00},
        {0x0910, 0x00},
        {0x0911, 0x01},
        {0x0912, 0x01},
    };
    DevBlkCDII2CRegList mappingRaw = {
        .regs = mappingRawRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingRawRegs),
    };

    if (linkPipelineMap == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null link pipeline map");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    vcID = linkPipelineMap[linkIndex].vcID;
    txPort = priv->ctx.txPort;
    /* Update Tx Port */
    mappingRawRegs[1].data = (txPort << 4u) | (txPort << 2u) | txPort;

    switch (linkPipelineMap[linkIndex].dataType) {
        case MAX96712_DATA_TYPE_RAW10:
            dataTypeVal = 0x2B;
            break;
        case MAX96712_DATA_TYPE_RAW12:
            dataTypeVal = 0x2C;
            break;
        case MAX96712_DATA_TYPE_RAW16:
            dataTypeVal = 0x2E;
            break;
        case MAX96712_DATA_TYPE_RGB:
            dataTypeVal = 0x24;
            break;
        case MAX96712_DATA_TYPE_YUV_8:
            dataTypeVal = 0x1E;
            break;
        default:
            LOG_ERROR("MAX96712: Bad parameter: Invalid data type");
            return WICRI_STATUS_BAD_PARAMETER;
    }

    if (priv->ctx.pipelineEnabled & (0x1 << linkIndex)) {
        PGGen0 = true;
    } else if (priv->ctx.pipelineEnabled & (0x10 << linkIndex)) {
        PGGen0 = false;
    } else {
        LOG_ERROR("MAX96712: No pipeline enabled for the link %d", (uint32_t)linkIndex);
        LOG_INFO("No pipeline enabled for the link %d\n", linkIndex);
        LOG_INFO("          Please make sure if WRITE_PARAM_CMD_MAX96712_SET_PG calling\n");
        LOG_INFO("          before WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_TPG\n");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    /* update offset */
    for (i = 0u; i < 8u; i++) {
        mappingRawRegs[i].address += (linkIndex * 0x40) + ((PGGen0 == false) ? 0x100 : 0x0);
    }

    /* 4 mapping for the pixel data */
    mappingRawRegs[2].data = dataTypeVal;
    mappingRawRegs[3].data = (vcID << 6u) | dataTypeVal;
    /* Change FS packet's DT to reserved for RAW pipeline */
    mappingRawRegs[5].data = (vcID << 6u) | 0u;
    /* Change FE packet's DT to reserved for RAW pipeline */
    mappingRawRegs[7].data = (vcID << 6u) | 1u;

    status = WriteArray(priv->i2cProgrammer, &mappingRaw);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    return status;
}

static NvMediaStatus
ConfigPGSettings(
    max96712 *priv,
    uint32_t width,
    uint32_t height,
    uint32_t frameRate,
    uint8_t linkIndex)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CRegList *PGarray = NULL;
    DevBlkCDII2CReg *regsPG = NULL;
    uint8_t i = 0U, j = 0U, pclk = 0U;
    PGModeMAX96712 pgMode = MAX96712_PG_NUM;

    priv->ctx.tpgEnabled = true;

    if (i >= MAX96712_MAX_NUM_PG) {
        return WICRI_STATUS_ERROR;
    }

    if ((width == 1920U) && (height == 1236U) && (frameRate == 30)) {
        pgMode = MAX96712_PG_1920_1236_30FPS;
        PGarray = &configPGArrCmd[pgMode];
        regsPG = PGArr1920x1236_30FPS_PATGEN0;
        pclk = MAX96712_PG_PCLK_150MHX;
    } else if ((width == 1920U) && (height == 1236U) && (frameRate == 60)) {
        pgMode = MAX96712_PG_1920_1236_60FPS;
        PGarray = &configPGArrCmd[pgMode];
        regsPG = PGArr1920x1236_60FPS_PATGEN0;
        pclk = MAX96712_PG_PCLK_375MHX;
    } else if ((width == 3848U) && (height == 2168U) && (frameRate == 30)) {
        pgMode = MAX96712_PG_3848_2168_30FPS;
        PGarray = &configPGArrCmd[pgMode];
        regsPG = PGArr3848x2168_30FPS_PATGEN0;
        pclk = MAX96712_PG_PCLK_375MHX;
    } else if ((width == 3848U) && (height == 2174U) && (frameRate == 30)) {
        pgMode = MAX96712_PG_3848_2174_30FPS;
        PGarray = &configPGArrCmd[pgMode];
        regsPG = PGArr3848x2174_30FPS_PATGEN0;
        pclk = MAX96712_PG_PCLK_375MHX;
    } else if ((width == 2880U) && (height == 1860U) && (frameRate == 30)) {
        pgMode = MAX96712_PG_2880_1860_30FPS;
        PGarray = &configPGArrCmd[pgMode];
        regsPG = PGArr2880x1860_30FPS_PATGEN0;
        pclk = MAX96712_PG_PCLK_375MHX;
    } else {
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0U; i < MAX96712_MAX_NUM_PG; i++) {
        if (priv->ctx.pgMode[i] == pgMode) {
            break;
        }
        if (priv->ctx.pgMode[i] == MAX96712_PG_NUM) {
            priv->ctx.pgMode[i] = pgMode;
            break;
        }
    }

    priv->ctx.pipelineEnabled |= ((1 << linkIndex) << (i * 4));

    if (i == 1U) { /* For 2nd PG, need to update the register offset */
        /* PG setting */
        for (j = 0U; j < 38U; j++) {
            regsPG[j].address += 0x30;
        }
    }

    status = WriteArray(priv->i2cProgrammer,
                                        PGarray);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    if (pclk == MAX96712_PG_PCLK_150MHX) {
        status = WriteArray(priv->i2cProgrammer,
                                            &configPGPCLK150MHZ[i]);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_FORCE_CSI_OUT_EN,
                                 1u,
                                 REG_READ_MOD_WRITE_MODE);

    return status;
}

static NvMediaStatus
MapUnusedPipe(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t i = 0U;
    uint8_t j = 0U;
    DevBlkCDII2CReg mappingRawRegs[] = {
        /* Send RAW12 FS and FE from X to Controller 1 */
        {0x090B, 0x07},
        {0x092D, 0x3F},
        /* For the following MSB 2 bits = VC, LSB 6 bits = DT */
        {0x090D, 0x24},
        {0x090E, 0x3F},
        {0x090F, 0x00},
        {0x0910, 0x02},
        {0x0911, 0x01},
        {0x0912, 0x03},
    };
    DevBlkCDII2CRegList mappingRaw = {
        .regs = mappingRawRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingRawRegs),
    };
    priv->ctx.tpgEnabled = true;
    /* When enabling TPG on Max96712, 1st TPG output is going to pipeline 0 ~ 3,
     * 2nd TPG output is going to pipeline 4 ~ 7.
     * And pipeline 0/4 is going to controller 0, pipeline 1/5 is going to controller 1
     * pipeline 2/6 is going to controller 2, pipeline 3/7 is going to controller 3 by default.
     * Since there is no way to disable TPG and TPG is behind the pipeline,
     * undesired pipeline output has to be mapped to unused controller.
     */
    for (i = 0U; i < MAX96712_NUM_VIDEO_PIPELINES; i++) {
        if (!(priv->ctx.pipelineEnabled & (0x1 << i))) {
            if (priv->ctx.mipiOutMode == MAX96712_MIPI_OUT_4x2) {
                mappingRawRegs[1].data = 0x3F; /* controller 1 */
            } else if (priv->ctx.mipiOutMode == MAX96712_MIPI_OUT_2x4) {
                /* 2x4 mode*/
                mappingRawRegs[1].data = 0x3F; /* controller 0 */
            } else {
                return WICRI_STATUS_BAD_PARAMETER;
            }

            status = WriteArray(priv->i2cProgrammer, &mappingRaw);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }

        for (j = 0U; j < 8U; j++) {
            mappingRawRegs[j].address += 0x40;
        }
    }

    return status;
}

static NvMediaStatus
EnablePG(
    max96712 *priv,bool enable)
{
    
    DevBlkCDII2CReg enablePGArrCmdReg = {0x1050, 0xFb};
    NvMediaStatus status;
    priv->ctx.tpgEnabled = true;

    if (priv->ctx.pipelineEnabled & 0xF0) {
        enablePGArrCmdReg.address += 0x30;
    }
    if (enable)
    {
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_CSI_MIPI_OUT_EN,
                                    1,
                                    REG_READ_MOD_WRITE_MODE);
        if (status != WICRI_STATUS_OK)
        {
            LOG_ERROR("EnablePG: Set CSI mipi out Failed!");
            return status;
        }
        status = WriteUint8(priv->i2cProgrammer,
                                enablePGArrCmdReg.address,
                                enablePGArrCmdReg.data);
        
    }else{
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_CSI_MIPI_OUT_EN,
                            0,
                            REG_READ_MOD_WRITE_MODE);
    }
    return status;
}

static NvMediaStatus
SetTxSRCId(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CReg txSrcIdReg = {0x0503, 0x00};
    uint8_t i = 0u;

    if (MAX96712_IS_MULTIPLE_GMSL_LINK_SET(link)) {
        LOG_ERROR("MAX96712: Bad param: Multiple links specified");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            txSrcIdReg.address += (i << 4); /* update the address */
            txSrcIdReg.data = i; /* 0 - link 0, 1 - link 1, so on */

            status = WriteUint8(priv->i2cProgrammer,
                                                txSrcIdReg.address,
                                                txSrcIdReg.data);
            break;
        }
    }

    return status;
}

static NvMediaStatus
DisableAutoAck(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CReg autoAckReg = {0x0B0D, 0x00};
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            autoAckReg.address += (i << 8); /* update the address */

            status = WriteUint8(priv->i2cProgrammer,
                                                autoAckReg.address,
                                                autoAckReg.data);
            us_sleep(25000);
        }
    }

    return status;
}

static NvMediaStatus
EnableERRB(
    max96712 *priv,
    bool enable)
{
    NvMediaStatus status = WICRI_STATUS_OK;

    ClearRegFieldQ(priv);
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_ENABLE_ERRB,
                                 (enable ? 1u : 0u),
                                 REG_READ_MOD_WRITE_MODE);

    return status;
}

static NvMediaStatus
EnableBackTop(
    max96712 *priv,
    bool enable)
{
    NvMediaStatus status = WICRI_STATUS_OK;

    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_BACKTOP_EN,
                                 (enable ? 1u : 0u),
                                 REG_READ_MOD_WRITE_MODE);

    return status;
}

static NvMediaStatus
TriggerDeskew(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint8_t i = 0u;
    uint8_t phy_num = 0u;
    uint8_t temp;
    DevBlkCDII2CReg deskewReg = {0x0903, 0x00};

    /* Trigger the initial deskew patterns two times
     * to make sure Rx device recevies the patterns */
    for (i = 0u; i < 2u; i++) {
        for (phy_num = 0u; phy_num < MAX96712_MAX_NUM_PHY; phy_num++) {
            /* Update the register offset */
            deskewReg.address = (deskewReg.address & 0xFF00U) +
                                (0x40U * phy_num) + 0x03U;
            status = ReadUint8(priv->i2cProgrammer,
                                               deskewReg.address,
                                               &temp);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
            deskewReg.data = temp;

            deskewReg.data ^= (1 << 5);
            status = WriteUint8(priv->i2cProgrammer,
                                                deskewReg.address,
                                                deskewReg.data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }
        us_sleep((i == 0u) ? 10000 : 0);
    }

    return status;
}

static NvMediaStatus
EnableExtraSMs(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;

    ClearRegFieldQ(priv);
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR4,
                                 0xF3u,
                                 REG_READ_MOD_WRITE_MODE);

    ClearRegFieldQ(priv);
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR6,
                                 0xFDu, /* TODO : Enable the remote error */
                                 REG_READ_MOD_WRITE_MODE);

    ClearRegFieldQ(priv);
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR10,
                                 0xFFu,
                                 REG_READ_MOD_WRITE_MODE);

    ClearRegFieldQ(priv);
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR12_MEM_ERR_OEN,
                                 0x1u,
                                 REG_READ_MOD_WRITE_MODE);

    ClearRegFieldQ(priv);
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_MASKED_OEN,
                                 0x3Fu,
                                 REG_READ_MOD_WRITE_MODE);

    return status;
}

static NvMediaStatus
SetI2CPort(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    I2CPortMAX96712 i2cPort = priv->ctx.i2cPort;
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(priv->ctx.linkMask, i)) {
            if (priv->ctx.gmslMode[i] == MAX96712_GMSL1_MODE) {
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_I2C_PORT_GMSL1_PHY_A + i,
                                             (i2cPort == MAX96712_I2CPORT_0) ? 0u : 1u,
                                             REG_READ_MOD_WRITE_MODE);
            } else if (IsGMSL2Mode(priv->ctx.gmslMode[i])) {
                /* Disable connection from both port 0/1 */
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DIS_REM_CC_A + i,
                                             0x3u,
                                             REG_READ_MOD_WRITE_MODE);

                /* Select port 0 or 1 over the link */
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_SEC_XOVER_SEL_PHY_A + i,
                                             (i2cPort == MAX96712_I2CPORT_0) ? 0u : 1u,
                                             REG_READ_MOD_WRITE_MODE);

                /* Enable connection from port 0 or 1 */
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DIS_REM_CC_A + i,
                                             (i2cPort == MAX96712_I2CPORT_0) ? 2u : 1u,
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
            return WICRI_STATUS_BAD_PARAMETER;
        }
    }

    return status;
}

static NvMediaStatus
SetFSYNCMode(
    max96712 *priv,
    FSyncModeMAX96712 FSyncMode,
    uint32_t pclk,
    uint32_t fps,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    GMSLModeMAX96712 gmslMode = MAX96712_GMSL_MODE_INVALID;
    DevBlkCDII2CReg fsyncPeriodRegs[] = {
        {0x04A7, 0x00}, /* Calculate FRSYNC period H. don't move position */
        {0x04A6, 0x00}, /* Calculate FRSYNC period M. don't move position */
        {0x04A5, 0x00}, /* Calculate FRSYNC period L. don't move position */
    };
    DevBlkCDII2CRegList fsyncPeriod = {
        .regs = fsyncPeriodRegs,
        .numRegs = I2C_ARRAY_SIZE(fsyncPeriodRegs),
    };
    DevBlkCDII2CReg setManualFsyncModeRegs[] = {
        {0x04A2, 0xE0}, /* video link for fsync generation */
        {0x04AA, 0x00}, /* Disable overlap window */
        {0x04AB, 0x00}, /* Disable overlap window */
        {0x04A8, 0x00}, /* Disable error threshold */
        {0x04A9, 0x00}, /* Disable error threshold */
        {0x04AF, 0x1F}, /* Set FSYNC to GMSL1 type */
        {0x04A0, 0x10}, /* Set FSYNC to manual mode */
    };
    DevBlkCDII2CRegList setManualFsyncMode = {
        .regs = setManualFsyncModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setManualFsyncModeRegs),
    };
    DevBlkCDII2CReg setAutoFsyncModeRegs[] = {
        {0x04A2, 0xE1}, /* video link for fsync generation */
        {0x04AA, 0x00}, /* Disable overlap window */
        {0x04AB, 0x00}, /* Disable overlap window */
        {0x04A8, 0x00}, /* Disable error threshold */
        {0x04A9, 0x00}, /* Disable error threshold */
        {0x04B1, 0x78}, /* GPIO ID setup to output FSYNC. For Auto mode, select ID=0xF */
        {0x04A0, 0x12}, /* Set FSYNC to auto mode */
    };
    DevBlkCDII2CRegList setAutoFsyncMode = {
        .regs = setAutoFsyncModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setAutoFsyncModeRegs),
    };
    DevBlkCDII2CReg setOSCManualFsyncModeRegs[] = {
        {0x04AF, 0x40, 0x2710}, /* Set FSYNC to GMSL1 type then delay 10ms*/
        {0x04A0, 0x00, 0x2710}, /* Set FSYNC to manual mode then delay 10ms*/
        {0x04A2, 0x00, 0x2710}, /* Turn off auto master link selection then delay 10ms*/
        {0x04AA, 0x00, 0x2710}, /* Disable overlap window then delay 10ms*/
        {0x04AB, 0x00, 0x2710}, /* Disable overlap window then delay 10ms*/
        {0x04A8, 0x00, 0x2710}, /* Disable error threshold then delay 10ms*/
        {0x04A9, 0x00, 0x2710}, /* Disable error threshold then delay 10ms*/
    };
    DevBlkCDII2CRegList setOSCManualFsyncMode = {
        .regs = setOSCManualFsyncModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setOSCManualFsyncModeRegs),
    };
    DevBlkCDII2CReg setExtFsyncModeReg = {0x04A0, 0x08};
    DevBlkCDII2CReg setTxIDIntReg = {0x04B1, MAX96712_GPIO_20 << 3}; /* GPIO ID 20 */
    DevBlkCDII2CReg setGpio2Mode = {0x0306, 0x83};
    DevBlkCDII2CReg setGMSL2PerLinkExtFsyncModeRegs[4] = {
        {0x0307, 0xA0 | MAX96712_GPIO_2},
        {0x033D, 0x20 | MAX96712_GPIO_2},
        {0x0374, 0x20 | MAX96712_GPIO_2},
        {0x03AA, 0x20 | MAX96712_GPIO_2},
    };
    DevBlkCDII2CReg enableGpiGpoReg = {0x0B08, 0x00};
    uint8_t i = 0u;

    /* TODO: Handle GMSL1 + GMSL2 case */
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            gmslMode = priv->ctx.gmslMode[i];
            break;
        }
    }

    if (FSyncMode == MAX96712_FSYNC_MANUAL) {
        /* Calculate FRSYNC period in manual mode based on PCLK */
        if (priv->ctx.manualFSyncFPS != 0u) {
            if (priv->ctx.manualFSyncFPS != fps) {
                /* Requested a new manual fsync frequency*/
                LOG_ERROR("MAX96712: 2 different manual fsync frequencies requested");
                return WICRI_STATUS_NOT_SUPPORTED;
            }
        } else {
            /* calculate frsync high period */
            fsyncPeriodRegs[0].data = (uint8_t)((gmslMode ==
                                      (MAX96712_GMSL1_MODE)) ?
                                      (((pclk / fps) >> 16U) &
                                        0xFFU) : 0x25U);
            /* calculate frsync middle period */
            fsyncPeriodRegs[1].data = (uint8_t)((gmslMode ==
                                      (MAX96712_GMSL1_MODE)) ?
                                      (((pclk / fps) >> 8U) &
                                        0xFFU) : 0x4CU);
            /* calculate frsync low period */
            fsyncPeriodRegs[2].data = (uint8_t)((gmslMode ==
                                      (MAX96712_GMSL1_MODE)) ?
                                      ((pclk / fps) & 0xFFU) : 0x9CU);

            status = WriteArray(priv->i2cProgrammer, &fsyncPeriod);
            if (status != WICRI_STATUS_OK) {
                return status;
            }

            if (IsGMSL2Mode(gmslMode)) {
                setManualFsyncModeRegs[6].data = 0x90; /* Set FSYNC to GMSL2 type */
            }

            status = WriteArray(priv->i2cProgrammer, &setManualFsyncMode);
            if (status != WICRI_STATUS_OK) {
                return status;
            }

            us_sleep(10000);

            priv->ctx.manualFSyncFPS = fps;
        }

        if (gmslMode == MAX96712_GMSL1_MODE) {
                enableGpiGpoReg.data = 0x35;

                for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
                    if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
                        enableGpiGpoReg.address += (i << 8);

                        status = WriteUint8(priv->i2cProgrammer,
                                                            enableGpiGpoReg.address,
                                                            enableGpiGpoReg.data);
                        if (status != WICRI_STATUS_OK) {
                            return status;
                        }
                        us_sleep(10000);
                    }
                }
        }
    } else if (FSyncMode == MAX96712_FSYNC_OSC_MANUAL) {
        /* Calculate FRSYNC period in manual with OSC mode */
        if (priv->ctx.manualFSyncFPS != 0u) {
            if (priv->ctx.manualFSyncFPS != fps) {
                /* Requested a new manual fsync frequency*/
                LOG_ERROR("MAX96712: 2 different manual osc fsync frequencies requested");
                return WICRI_STATUS_NOT_SUPPORTED;
            }
        }

        /* MAXIM doesn't recommend to use auto or semi-auto mode for the safety concern.
         * If the master link is lost, the frame sync will be lost for other links in both modes.
         * Instead the manual mode with OSC in MAX96712 is recommended.
         */
        if (IsGMSL2Mode(gmslMode)) {
            setOSCManualFsyncModeRegs[0].data |= (1 << 7); /* Set FSYNC to GMSL2 type */
        }

        status = WriteArray(priv->i2cProgrammer, &setOSCManualFsyncMode);
        if (status != WICRI_STATUS_OK) {
            return status;
        }

        if (IsGMSL2Mode(gmslMode)) {
            status = WriteUint8(priv->i2cProgrammer,
                                                setTxIDIntReg.address,
                                                setTxIDIntReg.data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }

        /* calculate frsync high period
         */
        fsyncPeriodRegs[0].data =
           (uint16_t)(((MAX96712_OSC_MHZ * 1000U * 1000U / fps) >> 16U) &
                        0xFFU);
        /* calculate frsync middle period */
        fsyncPeriodRegs[1].data =
           (uint16_t)(((MAX96712_OSC_MHZ * 1000U * 1000U / fps) >> 8U) &
                        0xFFU);
        /* calculate frsync low period */
        fsyncPeriodRegs[2].data =
           (uint16_t)((MAX96712_OSC_MHZ * 1000U * 1000U / fps) & 0xFFU);

        status = WriteArray(priv->i2cProgrammer, &fsyncPeriod);
        if (status != WICRI_STATUS_OK) {
            return status;
        }

        priv->ctx.manualFSyncFPS = fps;

        if (gmslMode == MAX96712_GMSL1_MODE) {
            enableGpiGpoReg.data = 0x35;

            for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
                if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
                    enableGpiGpoReg.address += (i << 8);

                    status = WriteUint8(priv->i2cProgrammer,
                                                        enableGpiGpoReg.address,
                                                        enableGpiGpoReg.data);
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                    us_sleep(10000);
                }
            }
        }
    } else if (FSyncMode == MAX96712_FSYNC_EXTERNAL) {
        for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
            if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
                if (gmslMode == MAX96712_GMSL1_MODE) {
                    status = WriteUint8(priv->i2cProgrammer,
                                                        setExtFsyncModeReg.address,
                                                        setExtFsyncModeReg.data);
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                    enableGpiGpoReg.data = 0x65;
                    enableGpiGpoReg.address += (i << 8);

                    status = WriteUint8(priv->i2cProgrammer,
                                                        enableGpiGpoReg.address,
                                                        enableGpiGpoReg.data);
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                    us_sleep(10000);
                } else {
                    status = WriteUint8(priv->i2cProgrammer,
                                                        setGpio2Mode.address,
                                                        setGpio2Mode.data);
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }

                    status = WriteUint8(priv->i2cProgrammer,
                                                        setGMSL2PerLinkExtFsyncModeRegs[i].address,
                                                        setGMSL2PerLinkExtFsyncModeRegs[i].data);
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                    us_sleep(10000);
                }
            }
        }
    } else if (FSyncMode == MAX96712_FSYNC_AUTO) {
        status = WriteArray(priv->i2cProgrammer, &setAutoFsyncMode);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    } else {
        LOG_ERROR("MAX96712: Invalid param: FSyncMode");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    return status;
}

static NvMediaStatus
ReadCtrlChnlCRCErr(
    max96712 *priv,
    LinkMAX96712 link,
    uint8_t *errVal)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t i = 0u;

    if (errVal == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null error value");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_CC_CRC_ERRCNT_A + i,
                                         0u,
                                         REG_READ_MODE);
            *errVal = ReadFromRegFieldQ(priv, 0u);
        }
    }

    return WICRI_STATUS_OK;
}

static NvMediaStatus
GetEnabledLinks(
    max96712 *priv,
    LinkMAX96712 *link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t i = 0u;

    *link = 0u;
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        ClearRegFieldQ(priv);
        ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_LINK_EN_A + i,
                                  0u);
        ACCESS_REG_FIELD_RET_ERR(REG_READ_MODE);
        *link |= (ReadFromRegFieldQ(priv, 0u) << i);
    }

    return status;
}

static NvMediaStatus
ClearErrb(
    max96712 *priv,
    LinkMAX96712 *link,
    uint8_t *errVal)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    ErrorStatusMAX96712 errorStatus;
    
    uint8_t regVal = 0u;

    if (priv->ctx.tpgEnabled == true) {
        return WICRI_STATUS_OK;
    }

    ClearRegFieldQ(priv);
    ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_ERRB,
                              0u);
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MODE);
    if (ReadFromRegFieldQ(priv, 0u) == 1u) {
        LOG_ERROR("MAX96712: MAX96712 ERRB was Set");
        status = MAX96712GetErrorStatus(priv,
                                        sizeof(errorStatus),
                                        &errorStatus);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    ClearRegFieldQ(priv);
    ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_OVERFLOW_FIRST4,
                              0u);
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MODE);
    regVal = ReadFromRegFieldQ(priv, 0u) ;
    if (regVal != 0u) {
        LOG_ERROR("MAX96712: MAX96712 Overflow in pipeline 0~3 reported regval:0x%02x", (uint32_t)regVal);
        us_sleep(10000);
        return WICRI_STATUS_ERROR;
    }

    ClearRegFieldQ(priv);
    ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_OVERFLOW_LAST4,
                              0u);
    ACCESS_REG_FIELD_RET_ERR(REG_READ_MODE);
    regVal = ReadFromRegFieldQ(priv, 0u) ;
    if (regVal != 0u) {
        LOG_ERROR("MAX96712: MAX96712 Overflow in pipeline 4~7 reported regval:0x%02x", (uint32_t)regVal);
        us_sleep(10000);
        return WICRI_STATUS_ERROR;
    }

    return status;
}

static NvMediaStatus
EnableReplication(
    max96712 *priv,
    bool enable)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    TxPortMAX96712 port = MAX96712_TXPORT_PHY_C;
    MipiOutModeMAX96712 mipiOutMode = MAX96712_MIPI_OUT_INVALID;
    PHYModeMAX96712 phyMode = MAX96712_PHY_MODE_INVALID;
    RevisionMAX96712 revision = MAX96712_REV_INVALID;
    bool passiveEnabled;
    uint8_t i = 0u;
    uint8_t temp;
    DevBlkCDII2CReg dataRegs[] = {
        {0x08A9, 0},  /* For the replication from Tegra A to Tegra B */
        {0x08AA, 0},  /* For the replication from Tegra A to Tegra C */
    };
    DevBlkCDII2CRegList data = {
        .regs = dataRegs,
        .numRegs = I2C_ARRAY_SIZE(dataRegs),
    };
    if (priv == NULL) {
        return WICRI_STATUS_BAD_PARAMETER;
    }

    port = priv->ctx.txPort;
    mipiOutMode = priv->ctx.mipiOutMode;
    revision = priv->ctx.revision;
    phyMode = priv->ctx.phyMode;
    passiveEnabled = priv->ctx.passiveEnabled;

    /* Replication is not supported on revision 1 in CPHY mode */
    if ((revision == MAX96712_REV_1) &&
        (phyMode == MAX96712_PHY_MODE_CPHY)) {
        LOG_ERROR("MAX96712: Replication in CPHY mode is supported only "
                         "on platforms with MAX96712 revision 2 or higher.");
        return WICRI_STATUS_NOT_SUPPORTED;
    }

    if ((!passiveEnabled) && enable) { /* Update the replication but do NOT enable the replication */
        switch (port) {
            case MAX96712_TXPORT_PHY_C :
                if (mipiOutMode == MAX96712_MIPI_OUT_4x2) {
                    dataRegs[0].data = MAX96712_REPLICATION(MAX96712_TXPORT_PHY_C, MAX96712_TXPORT_PHY_E);
                    dataRegs[1].data = MAX96712_REPLICATION(MAX96712_TXPORT_PHY_C, MAX96712_TXPORT_PHY_F);
                    if (revision < MAX96712_REV_4) {
                        /* 3rd I2C port connected to 3rd Xavier is enabled by default only in MAX96712 Rev D(4)
                         * For other revisions, the replication from PHY C to PHY F is enabled by the master
                         */
                        dataRegs[1].data |= (uint8_t)(1U << 7);
                    }
                } else if (mipiOutMode == MAX96712_MIPI_OUT_2x4) {
                    dataRegs[0].data = MAX96712_REPLICATION(MAX96712_TXPORT_PHY_D, MAX96712_TXPORT_PHY_E);
                } else {
                    return WICRI_STATUS_BAD_PARAMETER;
                }
                break;
            case MAX96712_TXPORT_PHY_D :
                if ((mipiOutMode == MAX96712_MIPI_OUT_2x4) || (mipiOutMode == MAX96712_MIPI_OUT_4x2)) {
                    dataRegs[0].data = MAX96712_REPLICATION(MAX96712_TXPORT_PHY_D, MAX96712_TXPORT_PHY_E);
                } else {
                    return WICRI_STATUS_BAD_PARAMETER;
                }
                break;
            case MAX96712_TXPORT_PHY_E :
                if (mipiOutMode == MAX96712_MIPI_OUT_2x4) {
                    dataRegs[0].data = MAX96712_REPLICATION(MAX96712_TXPORT_PHY_E, MAX96712_TXPORT_PHY_D);
                } else if (mipiOutMode == MAX96712_MIPI_OUT_4x2) {
                    dataRegs[0].data = MAX96712_REPLICATION(MAX96712_TXPORT_PHY_E, MAX96712_TXPORT_PHY_C);
                } else {
                    return WICRI_STATUS_BAD_PARAMETER;
                }
                break;
            default :
                dataRegs[0].data = MAX96712_REPLICATION(MAX96712_TXPORT_PHY_C, MAX96712_TXPORT_PHY_E);
                break;
        }

        status = WriteArray(priv->i2cProgrammer, &data);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    } else if (passiveEnabled) { /* Enable or disable the replication */
        if ((port == MAX96712_TXPORT_PHY_F) && (revision < MAX96712_REV_4)) {
            LOG_INFO("The replication mode is already enabled\n");
            return WICRI_STATUS_OK;
        }

        for (i = 0u; i < 2u; i++) {
            status = ReadUint8(priv->i2cProgrammer,
                                               dataRegs[i].address,
                                               &temp);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
            dataRegs[i].data = temp;
            if (((dataRegs[i].data >> 5) & 3) == port) { /* if the destination is same as port */
                if (enable) {
                    dataRegs[i].data |= (uint8_t)(1U << 7); /* Enable the replication */
                } else {
                    dataRegs[i].data &= ~(uint8_t)(1U << 7); /* Disable the replication */
                }
            }
        }

        status = WriteArray(priv->i2cProgrammer, &data);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    return status;
}

static NvMediaStatus
ConfigureMIPIOutput(
    max96712 *priv,
    uint8_t mipiSpeed,
    PHYModeMAX96712 phyMode)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    MipiOutModeMAX96712 mipiOutMode = priv->ctx.mipiOutMode;
    uint8_t mipiOutModeVal = (mipiOutMode == MAX96712_MIPI_OUT_4x2) ? (1u << 0u) : (1u << 2u);
    DevBlkCDII2CReg mipiOutputReg = {0x08A2, 0x00};
    uint8_t i = 0u;
    uint8_t temp;
    uint8_t prebegin = 0U, post = 0U;

    if ((phyMode != MAX96712_PHY_MODE_DPHY) &&
        (phyMode != MAX96712_PHY_MODE_CPHY)) {
        LOG_ERROR("MAX96712: Invalid MIPI output port");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if ((mipiSpeed < 1u) || (mipiSpeed > 25u)) {
        return WICRI_STATUS_BAD_PARAMETER;
    }

    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_MIPI_OUT_CFG,
                                 mipiOutModeVal,
                                 REG_READ_MOD_WRITE_MODE);

    /* Set prebegin phase, post length and prepare for CPHY mode
     * This is a requirement for CPHY periodic calibration */
    if (phyMode == MAX96712_PHY_MODE_CPHY) {
        if (mipiSpeed == 17) {
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
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_T_T3_PREBEGIN,
                                     prebegin,
                                     REG_READ_MOD_WRITE_MODE);

        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_T_T3_POST_PREP,
                                     post,
                                     REG_READ_MOD_WRITE_MODE);
    }

    /* Put all Phys in standby mode */
    mipiOutputReg.address = 0x08A2;
    // mipiOutputReg.data = 0xF4; /* Bug 200383247 : t_lpx 106.7 ns */
    mipiOutputReg.data = 0xF0;
    status = WriteUint8(priv->i2cProgrammer,
                                        mipiOutputReg.address,
                                        mipiOutputReg.data);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    /* Mapping data lanes Port A */
    mipiOutputReg.address = 0x08A3;
    mipiOutputReg.data = (mipiOutMode == MAX96712_MIPI_OUT_4x2) ? 0x44 : 0xE4;

    status = WriteUint8(priv->i2cProgrammer,
                                        mipiOutputReg.address,
                                        mipiOutputReg.data);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    /* Mapping data lanes Port B */
    mipiOutputReg.address = 0x08A4;
    mipiOutputReg.data = (mipiOutMode == MAX96712_MIPI_OUT_4x2) ? 0x44 : 0xE4;
    status = WriteUint8(priv->i2cProgrammer,
                                        mipiOutputReg.address,
                                        mipiOutputReg.data);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    /* Set CSI2 lane count per Phy */
    for (i = 0u; i < MAX96712_MAX_NUM_PHY; i++) {
        mipiOutputReg.data = (priv->ctx.lanes[i] - 1U) << 6U;
        mipiOutputReg.data |= (phyMode == MAX96712_PHY_MODE_CPHY) ? (1u << 5u) : 0u;
        mipiOutputReg.address = 0x090A + (i * 0x40U);
        status = WriteUint8(priv->i2cProgrammer,
                                            mipiOutputReg.address,
                                            mipiOutputReg.data);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    /* deactive DPLL */
    mipiOutputReg.address = 0x1C00;
    mipiOutputReg.data = 0xF4;

    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        mipiOutputReg.address = (0x1C + i) << 8;

        status = WriteUint8(priv->i2cProgrammer,
                                            mipiOutputReg.address,
                                            mipiOutputReg.data);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    /* Set MIPI speed */
    mipiOutputReg.address = 0x0415;
    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        mipiOutputReg.address =
           (mipiOutputReg.address & 0xFF00U) +
           0x15U +
           (i * 0x3U);
        status = ReadUint8(priv->i2cProgrammer,
                                           mipiOutputReg.address,
                                           &temp);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
        mipiOutputReg.data = temp;

        mipiOutputReg.data &= ~0x3F;
        mipiOutputReg.data |= ((1u << 5u) | mipiSpeed);
        status = WriteUint8(priv->i2cProgrammer,
                                            mipiOutputReg.address,
                                            mipiOutputReg.data);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    /* active DPLL */
    mipiOutputReg.address = 0x1C00;
    mipiOutputReg.data = 0xF5;

    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        mipiOutputReg.address = (0x1C + i) << 8;

        status = WriteUint8(priv->i2cProgrammer,
                                            mipiOutputReg.address,
                                            mipiOutputReg.data);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    /* Deskew is enabled if MIPI speed is faster than or equal to 1.5GHz */
    if ((phyMode == MAX96712_PHY_MODE_DPHY) && (mipiSpeed >= 15)) {
        mipiOutputReg.address = 0x0903;
        mipiOutputReg.data = 0x97; /* enable the initial deskew with 8 * 32K UI */
        for (i = 0; i < MAX96712_MAX_NUM_PHY; i++) {
            mipiOutputReg.address = (mipiOutputReg.address & 0xff00) + ((mipiOutputReg.address + 0x40) & 0xff) ;
            status = WriteUint8(priv->i2cProgrammer,
                                                mipiOutputReg.address,
                                                mipiOutputReg.data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }
    }

    return status;
}

static NvMediaStatus
DisableDE(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t i = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            ClearRegFieldQ(priv);

            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_DE_EN_PHY_A + i,
                                      0u);
            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_DE_PRBS_TYPE_PHY_A + i,
                                      1u);

            ACCESS_REG_FIELD_RET_ERR(REG_WRITE_MODE);
        }
    }

    return status;
}

static NvMediaStatus
SetDBL(
    max96712 *priv,
    LinkMAX96712 link,
    bool enable)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CReg dblReg = {0x0B07, 0x8C};
    uint8_t i = 0u;

    if (enable == false) {
        dblReg.data = 0x0;
    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            dblReg.address += (i << 8);
            status = WriteUint8(priv->i2cProgrammer, dblReg.address, dblReg.data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
            us_sleep(5000);
        }
    }

    return status;
}

static NvMediaStatus
ControlForwardChannels(
    max96712 *priv,
    LinkMAX96712 link,
    bool enable)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    I2CPortMAX96712 i2cPort = priv->ctx.i2cPort;
    uint8_t i = 0u;
    uint8_t data = 0;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            /* WAR Bug 2411206 and 200653773:
               Sometimes when reading the 0x0B04/0x0C04/0x0D04/0x0E04 registers,
               0x00 is returned, regardless of what was written to the register.
               To get around this issue, program the register with i2c write command
               directly, instead of using read-modify-write method with access field
               queue.
             */

            /* Set forward control channel bit if enabled */
            if (enable) {
                data |= 0x1;
            }

            /* Always set reverse control channel bit to 1 */
                data |= 0x2;

            /* Set I2C/UART port bit for Port 1 */
            if (i2cPort == MAX96712_I2CPORT_1) {
                data |= 0x8;
            }

            status = WriteUint8(priv->i2cProgrammer,
                                                regBitFieldProps[REG_FIELD_I2C_FWDCCEN_PHY_A + i].regAddr,
                                                data);
            us_sleep(10000);
        }
    }

    return status;
}

/*
 *  The functions defined below are the entry points when CDI functions are called.
 */

NvMediaStatus
MAX96712CheckLink(
    max96712 *priv,
    LinkMAX96712 link,
    uint32_t linkType,
    bool display)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    GMSLModeMAX96712 gmslMode = MAX96712_GMSL_MODE_INVALID;
    uint8_t i = 0u, linkIndex = 0u, success = 0u;

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null priv passed to MAX96712CheckLink");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null driver priv passed to MAX96712CheckLink");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (priv->ctx.tpgEnabled == true) {
        return WICRI_STATUS_OK;
    }

    for (linkIndex = 0u; linkIndex < MAX96712_MAX_NUM_LINK; linkIndex++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, linkIndex)) {
            gmslMode = priv->ctx.gmslMode[linkIndex];
            /* Check lock for each link */
            switch (linkType) {
                case MAX96712_LINK_LOCK_GMSL1_CONFIG:
                    if (gmslMode != MAX96712_GMSL1_MODE) {
                        LOG_ERROR("MAX96712: Config link lock is only valid in GMSL1 mode on link :%d", (int32_t)linkIndex);
                        return WICRI_STATUS_ERROR;
                    }

                    /* Check for GMSL1 Link Lock.*/
                    ClearRegFieldQ(priv);
                    ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL1_LOCK_A + linkIndex,
                                              0u);
                    ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL1_CONFIG_LOCK_A + linkIndex,
                                              0u);

                    /* From Max96712 programming guide V1.1, typical link rebuilding time is 25 ~ 100ms
                     * check the link lock in 100ms periodically every 10ms */
                    for (i = 0u; i < 50u; i++) {
                        ACCESS_REG_FIELD_RET_ERR(REG_READ_MODE);
                        if ((ReadFromRegFieldQ(priv, 0u) == 1u) ||
                            (ReadFromRegFieldQ(priv, 1u) == 1u))  {
                            LOG_INFO("MAX96712: Link %u: GMSL1 config link lock after %u ms\n", linkIndex, (i * 10u));
                            success = 1;
                            break;
                        }
                        us_sleep(10000);
                    }
                    if (success == 1) {
                        success = 0;
                        break;
                    } else {
                        if (display) {
                            LOG_ERROR("MAX96712: GMSL1 config link lock not detected linkindex:%d i:%d", (int32_t)linkIndex, (int32_t)i);
                        }
                        return WICRI_STATUS_ERROR;
                    }
                case MAX96712_LINK_LOCK_GMSL2:
                    if (!IsGMSL2Mode(gmslMode)) {
                        LOG_ERROR("MAX96712: GMSL2 link lock is only valid in GMSL2 mode linkindex:%d gmslMode:%d", (int32_t)linkIndex, (int32_t)gmslMode);
                        return WICRI_STATUS_ERROR;
                    }

                    /* Only register 0x001A is available on MAX96712 Rev 1 to check
                     * link lock in GMSL2 mode*/
                    if ((priv->ctx.revision == MAX96712_REV_1) &&
                                                    (linkIndex > 0U)) {
                        LOG_DBG(" GMSL2 link lock for link %u is not available on MAX96712 Rev 1\n",
                                 linkIndex);
                        return WICRI_STATUS_OK;
                    }

                    /* From Max96712 programming guide V1.1, typical link rebuilding time is 25 ~ 100ms
                     * check the link lock in 100ms periodically
                     * TODO : Intermittently the link lock takes more than 100ms. Check it with MAXIM */
                    for (i = 0u; i < 50u; i++) {
                        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL2_LOCK_A + linkIndex,
                                                     0u,
                                                     REG_READ_MODE);

                        if (ReadFromRegFieldQ(priv, 0) == 1u)  {
                            LOG_INFO("MAX96712: Link %u: GMSL2 link lock after %u ms", linkIndex, (i * 10u));
                            success = 1;
                            break;
                        }
                        us_sleep(10000);
                    }
                    if (success == 1) {
                        if (i > 10) {
                            LOG_INFO("MAX96712: GMSL2 Link time %d\n", i * 10);
                        }
                        success = 0;
                        break;
                    } else {
                        if (display) {
                            LOG_ERROR("MAX96712: GMSL2 link lock not detected on link :%d", (int32_t)linkIndex);
                        }
                        return WICRI_STATUS_ERROR;
                    }
                case MAX96712_LINK_LOCK_VIDEO:
                    if (gmslMode == MAX96712_GMSL1_MODE) {
                        for (i = 0u; i < 10u; i++) {
                            ClearRegFieldQ(priv);
                            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL1_LOCK_A + linkIndex,
                                                      0u);
                            ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL1_VIDEO_LOCK_A + linkIndex,
                                                      0u);
                            ACCESS_REG_FIELD_RET_ERR(REG_READ_MODE);

                            if ((ReadFromRegFieldQ(priv, 0u) == 1u) &&
                                (ReadFromRegFieldQ(priv, 1u) == 1u))  {
                                LOG_INFO("MAX96712: Link %u: GMSL1 video lock after %u ms", linkIndex, (i * 10u));
                                success = 1;
                                break;
                            }
                            us_sleep(10000);
                        }
                        if (success == 1) {
                            success = 0;
                            break;
                        } else {
                            if (display) {
                                LOG_ERROR("MAX96712: GMSL1 video lock not detected on link :%d", (int32_t)linkIndex);
                            }
                            return WICRI_STATUS_ERROR;
                        }
                    } else if (IsGMSL2Mode(gmslMode)){
                        /* TODO: Check emb pipes if enabled */
                        for (i = 0u; i < 10u; i++) {
                            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_LOCK_PIPE_0 + linkIndex,
                                                         0u,
                                                         REG_READ_MODE);

                            if (ReadFromRegFieldQ(priv, 0u) == 1u)  {
                                LOG_INFO("MAX96712: Link %u: GMSL2 video lock after %u ms", linkIndex, (i * 10u));
                                success = 1;
                                break;
                            }
                            us_sleep(10000);
                        }

                        if (success == 1) {
                            success = 0;
                            break;
                        } else {
                            if (display) {
                                LOG_ERROR("MAX96712: GMSL2 video lock not detected on link :%d", (int32_t)linkIndex);
                            }
                            return WICRI_STATUS_ERROR;
                        }
                    }
                    break;
                default:
                    LOG_ERROR("MAX96712: Bad parameter: Invalid link type");
                    return WICRI_STATUS_BAD_PARAMETER;
            }
        }
    }

    return WICRI_STATUS_OK;
}
EXPORT_SYMBOL(MAX96712CheckLink);

NvMediaStatus
MAX96712CheckPresence(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_ERROR;
    RevisionMAX96712 revision = MAX96712_REV_INVALID;
    uint8_t revisionVal = 0u;
    uint32_t numRev = sizeof(supportedRevisions) / sizeof(supportedRevisions[0]);
    uint8_t devID = 0u;
    uint32_t i = 0u;

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null priv passed to MAX96712CheckPresence");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to MAX96712CheckPresence");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    /* Check device ID */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DEV_ID,
                                 0u,
                                 REG_READ_MODE);
    devID = ReadFromRegFieldQ(priv, 0u);
    if ((devID != MAX96712_DEV_ID) && (devID != MAX96722_DEV_ID)) {
        LOG_ERROR("MAX96712: Device ID mismatch devid:%d", (uint32_t)devID);
        LOG_ERROR("MAX96712: Expected/Readval");
        LOG_ERROR("MAX96722: Expected/Readval");
        status = WICRI_STATUS_ERROR;
        goto done;
    }

    /* Check revision ID */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_DEV_REV,
                                 0u,
                                 REG_READ_MODE);
    revisionVal = ReadFromRegFieldQ(priv, 0u);
    for (i = 0u; i < numRev; i++) {
        if (revisionVal == supportedRevisions[i].revVal) {
            revision = supportedRevisions[i].revId;
            LOG_DBG(" Revision %u detected\n", revision);

            if (revision == MAX96712_REV_1) {
                LOG_INFO("MAX96712: Warning: MAX96712 revision 1 detected. All features may not be supported\n"
                        "Please use a platform with MAX96712 revision 2 or higher for full support\n");
                LOG_INFO("And the below error can be observed"
                        "  - FE_FRAME_ID_FAULT on CSIMUX_FRAME : Frame IDs are mismatched between FS and FE packets\n");
            }
            priv->ctx.revision = revision;
            status = WICRI_STATUS_OK;
            break;
        }
    }

    LOG_INFO("MAX96712:Device ID:%x revision ID:%x\n", devID, revisionVal);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: Unsupported MAX96712 revision detected! revisionVal:0x%02x", (uint32_t)revisionVal);
        LOG_INFO("MAX96712: Unsupported MAX96712 revision %u detected! Supported revisions are:", revisionVal);
        for (i = 0u; i < numRev; i++) {
            LOG_INFO("MAX96712: Revision %u\n", supportedRevisions[i].revVal);
        }
        status = WICRI_STATUS_NOT_SUPPORTED;
    }

done:
    return status;
}
EXPORT_SYMBOL(MAX96712CheckPresence);



NvMediaStatus
MAX96712DumpRegisters(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t data = 0;
    uint32_t i = 0u;

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null priv passed to DriverDestroy");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to DriverDestroy");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0u; i <= MAX96712_REG_MAX_ADDRESS; i++) {
        status = ReadUint8(priv->i2cProgrammer,
                                           ((i / 256u) << 8) | (i % 256u),
                                           &data);
        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX96712: I2C read failed from register/with status i:%d status:%d", (int32_t)i, (int32_t)status);
            return status;
        }

        LOG_INFO("Max96712: 0x%04X%02X - 0x%02X\n", (i / 256u), (i % 256u), data);
    }

    return status;
}

NvMediaStatus
MAX96712GetErrorStatus(
    max96712 *priv,
    uint32_t parameterSize,
    void *parameter)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    ErrorStatusMAX96712 *errorStatus = (ErrorStatusMAX96712 *) parameter;
    uint8_t globalErrorCount = 0U;
    uint8_t linkErrorCount = 0U;
    uint8_t pipelineErrorCount = 0U;
    uint8_t linkNum = 0u;
    uint8_t pipelineNum = 0u;
    bool pipelineErrAppears = false;
    uint8_t regFieldVal = 0u;
    uint8_t i = 0u;
    bool readAgainPwr0;

    if (MAX96712_MAX_LINK_BASED_ERROR_NUM < MAX96712_MAX_LINK_BASED_FAILURE_TYPES) {
        LOG_ERROR("MAX96712: Bad parameter: max link based error found smaller than failure types");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null priv in MAX96712GetErrorStatus");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null driver priv in MAX96712GetErrorStatus");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (parameter == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null parameter storage in MAX96712GetErrorStatus");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (parameterSize != sizeof(ErrorStatusMAX96712)) {
        LOG_ERROR("MAX96712: Bad parameter: Incorrect param size in MAX96712GetErrorStatus");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    memset(errorStatus, 0u, sizeof(ErrorStatusMAX96712));

    /* MAX96712_REG_GMSL1_LINK_A read back as 0 without this delay when any link is powered down */
    us_sleep(5000);

    /* ctrl3 (R0x1A)
     * intr5[3]: LOCKED
     * intr5[2]: ERROR
     * intr5[1]: CMU_LOCKED
     * rest bits are reserved. */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_CTRL3,
                                 0u,
                                 REG_READ_MODE);
    regFieldVal = ReadFromRegFieldQ(priv, 0u);
    if ((regFieldVal & 0x4) != 0u) {
        LOG_ERROR("MAX96712: global ERRB status regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_ERR, false);
    } else {
        /* error disappears, or false alarm, or some other cases not yet privd. Show message and return */
        LOG_INFO("MAX96712: not supported case found: global ERRB not asserted while GetErrorStatus is called\n");
        return WICRI_STATUS_OK;
    }

    if ((regFieldVal & 0x8) == 0u) {
        LOG_ERROR("MAX96712: global locked status regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_UNLOCK_ERR, false);
    }
    if ((regFieldVal & 0x2) == 0u) {
        LOG_ERROR("MAX96712: global CMU locked status regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_CMU_UNLOCK_ERR, false);
    }

    /* intr5 (R0x28)
     * intr5[2]: LFLT_INT
     * (rest bits are for RTTN_CRC_INT, WM, EOM link based errors) */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR5,
                                 0u,
                                 REG_READ_MODE);
    regFieldVal = ReadFromRegFieldQ(priv, 0u);
    if ((regFieldVal & 0x4) != 0u) {
        LOG_ERROR("MAX96712: global line fault error in bit 2 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_LINE_FAULT, false);
    }

    /* intr7 (R0x2A)
     * intr7[3]: LCRC_ERR_FLAG
     * intr7[2]: VPRBS_ERR_FLAG
     * intr7[1]: REM_ERR_FLAG
     * intr7[0]: FSYNC_ERR_FLAG
     * (rerst bits are for G1 link based errors, note we use R0xBCB than R0xB etc in later code) */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR7,
                                 0u,
                                 REG_READ_MODE);
    regFieldVal = ReadFromRegFieldQ(priv, 0u);
    if ((regFieldVal & 0x8) != 0u) {
        LOG_ERROR("MAX96712: global video line crc error in bit 3 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_VID_LINE_CRC, false);
    }
    if ((regFieldVal & 0x4)!= 0u) {
        LOG_ERROR("MAX96712: global video PRBS error in bit 2 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_VID_PRBS, false);
    }
    if ((regFieldVal & 0x2)!= 0u) {
        LOG_ERROR("MAX96712: global remote side error in bit 1 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_REMOTE_SIDE, false);
    }
    if ((regFieldVal & 0x1)!= 0u) {
        LOG_ERROR("MAX96712: global frame sync error in bit 0 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_FRAME_SYNC, false);
    }

    /* vid_pxl_crc_err_int (R0x45)
     * vid_pxl_crc_err_int[7]: mem ecc 2
     * vid_pxl_crc_err_int[6]: mem ecc 1
     * (rest bits are for video pixel crc link based errors) */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VID_PXL_CRC_ERR_INT,
                                 0u,
                                 REG_READ_MODE);
    regFieldVal = ReadFromRegFieldQ(priv, 0u);
    if ((regFieldVal & 0x80) != 0u) {
        LOG_ERROR("MAX96712: global mem ecc 2 error in bit 7 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_MEM_ECC2, false);
    }
    if ((regFieldVal & 0x40) != 0u) {
        LOG_ERROR("MAX96712: global mem ecc error in bit 6 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_MEM_ECC1, false);
    }

    /* fsync_22 (R0x4B6)
     * fsync_22[7]: FSYNC_LOSS_OF_LOCK
     * fsync_22[6]: FSYNC_LOCKED
     * (rest 6 bits are for FRM_DIFF_H, currently not to report) */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_FSYNC_22,
                                 0u,
                                 REG_READ_MODE);
    regFieldVal = ReadFromRegFieldQ(priv, 0u);
    if ((regFieldVal & 0x80) != 0u) {
        LOG_ERROR("MAX96712: global fsync sync loss error in bit 7 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_FSYNC_SYNC_LOSS, false);
    }
    if ((regFieldVal & 0x40) != 0u) {
        LOG_ERROR("MAX96712: global fsync status in bit 6 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_FSYNC_STATUS, false);
    }

    /* VIDEO_MASKED_FLAG (R0x04A)
     * VIDEO_MASKED_FLAG[5]: CMP_VTERM_STATUS
     * VIDEO_MASKED_FLAG[4]: VDD_OV_FLAG */
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_MASKED_FLAG,
                                 0u,
                                 REG_READ_MODE);
    regFieldVal = ReadFromRegFieldQ(priv, 0u);
    if ((regFieldVal & 0x20) == 0u) {
        LOG_ERROR("MAX96712: Vterm is latched low and less than 1v, in video mask reg bit 5 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_CMP_VTERM_STATUS, false);
    }
    if ((regFieldVal & 0x10) != 0u) {
        LOG_ERROR("MAX96712: Vdd_sw overvoltage condition detected, in video masked reg bit 4 regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_VDD_OV_FLAG, false);
    }

    /* PWR0 (R0x012)
     * PWR0[7:5]: VDDBAD_STATUS with bits 5 and bit 6 are effectively used.
     * PWR0[4:0]: CMP_STATUS, with bit 0,1,2 are for Vdd18/Vddio/Vdd_sw undervoltage latch low indicator */
    readAgainPwr0 = false;
    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_PWR0,
                                 0u,
                                 REG_READ_MODE);
    regFieldVal = ReadFromRegFieldQ(priv, 0u);
    if ((regFieldVal & 0x60U) == 0x60U) {
        LOG_ERROR("MAX96712: Vdd_sw less than 0.82v is observed since last read regFieldVal:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_VDDBAD_STATUS, false);
        readAgainPwr0 = true;
    }
    if ((regFieldVal & 0x4U) == 0U) {
        LOG_ERROR("MAX96712: Vdd_sw (1.0v) is latched low (undervoltage), reg value:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_VDDSW_UV, false);
        readAgainPwr0 = true;
    }
    if ((regFieldVal & 0x2U) == 0U) {
        LOG_ERROR("MAX96712: Vddio (1.8v) is latched low (undervoltage), reg value:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_VDDIO_UV, false);
        readAgainPwr0 = true;
    }
    if ((regFieldVal & 0x1U) == 0U) {
        LOG_ERROR("MAX96712: Vdd 1.8v is latched low (undervoltage), reg value:0x%02x", (uint32_t)regFieldVal);
        UPDATE_GLOBAL_ERROR(MAX96712_GLOBAL_VDD18_UV, true);
        readAgainPwr0 = true;
    }
    if (readAgainPwr0) {
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_PWR0,
                                     0u,
                                     REG_READ_MODE);
        regFieldVal = ReadFromRegFieldQ(priv, 0u);
        if (regFieldVal == 0x0U) {
            LOG_INFO("MAX96712: all undervoltage latches in PWR0 are cleared");

            // further read clear PWR_STATUS_FLAG (VDDBAD_INT_FLAG and VDDCMP_INT_FLAG)
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_PWR_STATUS_FLAG,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            LOG_INFO("MAX96712: read clear PWR_STATUS_FLAG (%u)", regFieldVal);
        } else {
            // PWR0 are not cleared, and PWR_STATUS_FLAG will still flag ERRB
            LOG_ERROR("MAX96712: not all undervoltage latches are cleared!");
        }
    }

    for (pipelineNum = 0u; pipelineNum < MAX96712_NUM_VIDEO_PIPELINES; pipelineNum++) {
        pipelineErrorCount = 0U;

        // overflow
        errorStatus->pipelineFailureType[pipelineNum][pipelineErrorCount] = MAX96712_PIPELINE_ERROR_INVALID;
        if (pipelineNum <= 3U) {
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_OVERFLOW_FIRST4,
                                         0u,
                                         REG_READ_MODE);
        } else { /* pipelineNum >= 4U && pipelineNum <= 7U) */
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_OVERFLOW_LAST4,
                                         0U,
                                         REG_READ_MODE);
        }

        regFieldVal = ReadFromRegFieldQ(priv, 0u);
        if (((regFieldVal) & (uint8_t)(1U << pipelineNum)) ||
            ((uint8_t)(regFieldVal >> 4U) & (uint8_t)(1U << pipelineNum))) {
            LOG_ERROR("MAX96712: pipeline overflow :%d", (int32_t)pipelineNum);
            pipelineErrAppears = true;

            /* Check overflow status every 1ms periodically */
            for (i = 0u; i < 100u; i++) {
                if (pipelineNum <= 3U) {
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_OVERFLOW_FIRST4,
                                                 0u,
                                                 REG_READ_MODE);
                } else {
                    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_OVERFLOW_LAST4,
                                                 0u,
                                                 REG_READ_MODE);
                }

                regFieldVal = ReadFromRegFieldQ(priv, 0u);
                if (((regFieldVal &
                    (uint8_t)(1U << pipelineNum)) == 0U) &&
                    (((uint8_t)(regFieldVal >> 4U) &
                    (uint8_t)(1U << pipelineNum)) == 0U)) {
                    LOG_ERROR("MAX96712: overflow disappear after x ms on link y i:%d pipelineNum:%d", (int32_t)i, (int32_t)pipelineNum);
                    pipelineErrAppears = false;
                    break;
                }
                us_sleep(1000);
            }

            if (pipelineErrAppears) {
                // line memory overflow bits are at BACKTOP11 register's bit[3:0]
                if (regFieldVal & (uint8_t)(1U << pipelineNum)) {
                    LOG_ERROR("MAX96712: lmo overflow error for pipeline:%d", pipelineNum);
                    UPDATE_PIPELINE_ERROR(MAX96712_PIPELINE_LMO_OVERFLOW_ERR, false);
                }
                // cmd overflow bits are at BACKTOP11 register's bit[7:4]
                if ((uint8_t)(regFieldVal >> 4) & (uint8_t)(1U << pipelineNum)) {
                    LOG_ERROR("MAX96712: cmd overflow error for pipeline:%d", pipelineNum);
                    UPDATE_PIPELINE_ERROR(MAX96712_PIPELINE_CMD_OVERFLOW_ERR, false);
                }
            }
        }

        // pipe pattern generator video lock status, register 0x1DC etc's bit 0, defined by 8 contiguous enums.
        errorStatus->pipelineFailureType[pipelineNum][pipelineErrorCount] = MAX96712_PIPELINE_ERROR_INVALID;
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_PATGEN_CLK_SRC_PIPE_0 + pipelineNum,
                                     0u,
                                     REG_READ_MODE);
        regFieldVal = ReadFromRegFieldQ(priv, 0u);
        if ((regFieldVal & 0x1) == 0u) {
            LOG_ERROR("MAX96712: video unlock on pipeline :%d", (int32_t)pipelineNum);
            UPDATE_PIPELINE_ERROR(MAX96712_PIPELINE_PGEN_VID_UNLOCK_ERR, false);
        }

        // mem_err
        errorStatus->pipelineFailureType[pipelineNum][pipelineErrorCount] = MAX96712_PIPELINE_ERROR_INVALID;
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_BACKTOP25,
                                     0u,
                                     REG_READ_MODE);
        regFieldVal = ReadFromRegFieldQ(priv, 0u);
        if ((regFieldVal & (0x1 << pipelineNum)) != 0u) {
            LOG_ERROR("MAX96712: line memory error on pipeline :%d", (int32_t)pipelineNum);
            UPDATE_PIPELINE_ERROR(MAX96712_PIPELINE_MEM_ERR, false);
        }

        // video sequence error status, register 0x108 etc's bit 4, defined by 8 contiguous enums.
        errorStatus->pipelineFailureType[pipelineNum][pipelineErrorCount] = MAX96712_PIPELINE_ERROR_INVALID;
        ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_RX8_PIPE_0 + pipelineNum,
                                     0u,
                                     REG_READ_MODE);
        regFieldVal = ReadFromRegFieldQ(priv, 0u);
        if ((regFieldVal & 0x10) != 0u) {
            LOG_ERROR("MAX96712: video sequence error on pipeline :%d", (int32_t)pipelineNum);
            UPDATE_PIPELINE_ERROR(MAX96712_PIPELINE_VID_SEQ_ERR, true);
        }
    }

    for (linkNum = 0u; linkNum < MAX96712_MAX_NUM_LINK; linkNum++) {
        linkErrorCount = 0U;
        errorStatus->linkFailureType[linkNum][linkErrorCount] = MAX96712_GMSL_LINK_ERROR_INVALID;

        // GMSL1/GMSL2 link based errors to be reported
        if (priv->ctx.gmslMode[linkNum] == MAX96712_GMSL1_MODE) {
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL1_LOCK_A + linkNum,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if (regFieldVal != 1u) {
                LOG_ERROR("MAX96712: GMSL1 link unlocked on link :%d", (int32_t)linkNum);
                UPDATE_LINK_ERROR(MAX96712_GMSL1_LINK_UNLOCK_ERR, false);
            }

            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL1_DET_ERR_A + linkNum,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if ((regFieldVal != 0u) &&
                (errorStatus->linkFailureType[linkNum][linkErrorCount] == MAX96712_GMSL_LINK_ERROR_INVALID)) {
                LOG_ERROR("MAX96712: GMSL1 decoding error on pipeline linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum, (uint32_t)regFieldVal);
                LOG_ERROR("MAX96712: Link &  GMSL1 decoding error linkNum:%d regFieldVal:0x%02x ", linkNum, regFieldVal);
                UPDATE_LINK_ERROR(MAX96712_GMSL1_LINK_DET_ERR, false);
            }

            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_CC_CRC_ERRCNT_A + linkNum,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if ((regFieldVal != 0u) &&
                (errorStatus->linkFailureType[linkNum][linkErrorCount] == MAX96712_GMSL_LINK_ERROR_INVALID)) {
                LOG_ERROR("MAX96712: GMSL1 PKTCC CRC failure on link linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum, (uint32_t)regFieldVal);
                UPDATE_LINK_ERROR(MAX96712_GMSL1_LINK_PKTCC_CRC_ERR, true);
            }
        } else if (IsGMSL2Mode(priv->ctx.gmslMode[linkNum])) {
            // link lock err
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL2_LOCK_A + linkNum,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if (regFieldVal == 0u) {
                LOG_ERROR("MAX96712: GMSL2 link unlocked linkNum:%d", (uint32_t)linkNum);
                UPDATE_LINK_ERROR(MAX96712_GMSL2_LINK_UNLOCK_ERR, false);
            }

            // dec err
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL2_DEC_ERR_A + linkNum,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if ((regFieldVal != 0u) &&
                (errorStatus->linkFailureType[linkNum][linkErrorCount] == MAX96712_GMSL_LINK_ERROR_INVALID)) {
                LOG_ERROR("MAX96712: GMSL2 decoding error linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum, (uint32_t)regFieldVal);
                UPDATE_LINK_ERROR(MAX96712_GMSL2_LINK_DEC_ERR, false);
            }

            // idle err
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_GMSL2_IDLE_ERR_A + linkNum,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if ((regFieldVal != 0u) &&
                (errorStatus->linkFailureType[linkNum][linkErrorCount] == MAX96712_GMSL_LINK_ERROR_INVALID)) {
                LOG_ERROR("MAX96712: GMSL2 idle error linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum,  (uint32_t)regFieldVal);
                UPDATE_LINK_ERROR(MAX96712_GMSL2_LINK_IDLE_ERR, false);
            }

            // EOM error (intr5, bit[7:4])
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR5,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if (((regFieldVal & (0x1 << (linkNum + 4u))) != 0u) &&
                (errorStatus->linkFailureType[linkNum][linkErrorCount] == MAX96712_GMSL_LINK_ERROR_INVALID)) {
                LOG_ERROR("MAX96712: Link eye open monitor error linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum, (uint32_t)regFieldVal);
                UPDATE_LINK_ERROR(MAX96712_GMSL2_LINK_EOM_ERR, false);
            }

            // ARQ errors (intr11)
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_INTR11,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if ((regFieldVal != 0u) &&
                (errorStatus->linkFailureType[linkNum][linkErrorCount] == MAX96712_GMSL_LINK_ERROR_INVALID)) {
                if ((regFieldVal & (1 << (linkNum + 4u))) != 0u) {
                    LOG_ERROR("MAX96712: Combined ARQ transmission error linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum, (uint32_t)regFieldVal);
                    UPDATE_LINK_ERROR(MAX96712_GMSL2_LINK_ARQ_RETRANS_ERR, false);
                }
                if ((regFieldVal & (1U << linkNum)) != 0u) {
                    LOG_ERROR("MAX96712: Combined ARQ max transmission error linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum, (uint32_t)regFieldVal);
                    UPDATE_LINK_ERROR(MAX96712_GMSL2_LINK_MAX_RETRANS_ERR, false);
                }
            }

            // vid_pxl_crc_err_int (R0x45[[3:0])
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_VID_PXL_CRC_ERR_INT,
                                         0u,
                                         REG_READ_MODE);
            regFieldVal = ReadFromRegFieldQ(priv, 0u);
            if ((regFieldVal & 0x0f) != 0u) {
                if ((regFieldVal & (uint8_t)(1U << linkNum)) != 0u) {
                    LOG_ERROR("MAX96712: Video pixel crc count linkNum:%d regFieldVal:0x%02x", (uint32_t)linkNum, (uint32_t)regFieldVal);
                    UPDATE_LINK_ERROR(MAX96712_GMSL2_LINK_VIDEO_PXL_CRC_ERR, true);
                }
            }
        }
    }

    return WICRI_STATUS_OK;
}

NvMediaStatus
MAX96712GetSerializerErrorStatus(max96712 *priv,
                                 bool * isSerError)
{
    NvMediaStatus status = WICRI_STATUS_OK;

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Bad parameter: Null priv passed to MAX96712GetSerializerErrorStatus");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_REM_ERR_FLAG,
                                 0u,
                                 REG_READ_MODE);
    if (ReadFromRegFieldQ(priv, 0u) == 1u) {
        *isSerError = true;
    }

    return status;
}

NvMediaStatus
MAX96712ReadParameters(
    max96712 *priv,
    uint32_t parameterType,
    uint32_t parameterSize,
    void *parameter)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    bool isValidSize = false;
    ReadParametersParamMAX96712 *param = (ReadParametersParamMAX96712 *) parameter;
    static const char *cmdString[] = {
        [READ_PARAM_CMD_MAX96712_REV_ID] =
            "READ_PARAM_CMD_MAX96712_REV_ID",
        [READ_PARAM_CMD_MAX96712_CONTROL_CHANNEL_CRC_ERROR] =
            "READ_PARAM_CMD_MAX96712_CONTROL_CHANNEL_CRC_ERROR",
        [READ_PARAM_CMD_MAX96712_ENABLED_LINKS] =
            "READ_PARAM_CMD_MAX96712_ENABLED_LINKS",
        [READ_PARAM_CMD_MAX96712_ERRB] =
            "READ_PARAM_CMD_MAX96712_ERRB",
    };

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null priv passed to MAX96712ReadParameters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv pased to MAX96712ReadParameters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (parameter == NULL) {
        LOG_ERROR("MAX96712: Bad driver parameter: Null ptr");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if ((parameterType == READ_PARAM_CMD_MAX96712_INVALID) ||
        (parameterType >= READ_PARAM_CMD_MAX96712_NUM)) {
        LOG_ERROR("MAX96712: Bad parameter: Invalid command");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    LOG_INFO("MAX96712: %s", cmdString[parameterType]);
    switch (parameterType) {
        case READ_PARAM_CMD_MAX96712_REV_ID:
            if (parameterSize == sizeof(param->revision)) {
                isValidSize = true;
                param->revision = priv->ctx.revision;
                status = WICRI_STATUS_OK;
            }
            break;
        case READ_PARAM_CMD_MAX96712_CONTROL_CHANNEL_CRC_ERROR:
            if (parameterSize == sizeof(param->ErrorStatus)) {
                isValidSize = true;
                status = ReadCtrlChnlCRCErr(priv,
                                            param->ErrorStatus.link,
                                            &param->ErrorStatus.errVal);
            }
            break;
        case READ_PARAM_CMD_MAX96712_ENABLED_LINKS:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = GetEnabledLinks(priv,
                                         &param->link);
            }
            break;
        case READ_PARAM_CMD_MAX96712_ERRB:
            if (parameterSize == sizeof(param->ErrorStatus)) {
                isValidSize = true;
                status = ClearErrb(priv,
                                   &param->ErrorStatus.link,
                                   &param->ErrorStatus.errVal);
            }
            break;
        default:
            LOG_ERROR("MAX96712: Bad parameter: Unprivd command");
            isValidSize = true;
            status = WICRI_STATUS_BAD_PARAMETER;
            break;
    }

    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: Command failed parameterType:%s", cmdString[parameterType]);
    }

    if (!isValidSize) {
        LOG_ERROR("MAX96712: Bad parameter: Invalid param size parameterType:%s", cmdString[parameterType]);
        status = WICRI_STATUS_BAD_PARAMETER;
    }

    return status;
}
EXPORT_SYMBOL(MAX96712ReadParameters);

NvMediaStatus
MAX96712ReadRegister(
    max96712 *priv,
    uint32_t deviceIndex,
    uint32_t registerNum,
    uint32_t dataLength,
    uint8_t *dataBuff)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint32_t i = 0u;

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null priv passed to MAX96712ReadRegister");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to MAX96712ReadRegister");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (dataBuff == NULL) {
        LOG_ERROR("MAX96712: Null data buffer passed to MAX96712ReadRegister");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0; i < dataLength; i ++) {
        status = ReadUint8(priv->i2cProgrammer,
                                           registerNum,
                                           &dataBuff[i]);
        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX96712: Register read failed with status registerNum:0x%04x status:%d", registerNum, (uint32_t)status);
        }
    }

    return status;
}

static NvMediaStatus
GMSL2LinkAdaptation(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    GMSLModeMAX96712 gmslMode = MAX96712_GMSL_MODE_INVALID;
    
    RevisionMAX96712 rev = priv->ctx.revision;
    uint8_t regVal = 0u, i = 0u, loop = 0u;

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(link, i)) {
            gmslMode = priv->ctx.gmslMode[i];

            if (!IsGMSL2Mode(gmslMode)) {
                LOG_INFO("MAX96712: Link %d: adaptation is required only in GMSL2 mode\n", i);
                continue;
            }

            /* Disable OSN */
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_ENABLE_OSN_0 + i,
                                         0u,
                                         REG_READ_MOD_WRITE_MODE);

            /* Reseed and set to default value 31 */
            ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_OSN_COEFF_MANUAL_SEED_0 + i,
                                         1u,
                                         REG_READ_MOD_WRITE_MODE);

            us_sleep(10000);

            for (loop = 0; loop < 100; loop++) {
                /* Read back OSN value */
                ACCESS_ONE_REG_FIELD_RET_ERR(REG_FIELD_OSN_COEFFICIENT_0 + i, 0u, REG_READ_MODE);
                regVal = ReadFromRegFieldQ(priv, 0u);
                if (regVal == 31) {
                    break;
                }
                us_sleep(1000);
            }
            (void)rev;
            LOG_INFO("MAX96712 Rev %d manual adaptation on the link %d (%d)\n", rev,
                                                                               i,
                                                                               regVal);
        }
    }

    return WICRI_STATUS_OK;
}

static NvMediaStatus
EnableMemoryECC(
    max96712 *priv,
    bool enable2bitReport,
    bool enable1bitReport)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CReg memECCReg = {0x0044, 0x0F};

    if (priv->ctx.revision < MAX96712_REV_3) {
        return WICRI_STATUS_NOT_SUPPORTED;
    }

    if (enable2bitReport) {
        memECCReg.data |= (uint8_t)(1U << 7);
    }
    if (enable1bitReport) {
        memECCReg.data |= (uint8_t)(1U << 6);
    }

    status = WriteUint8(priv->i2cProgrammer,
                                        memECCReg.address,
                                        memECCReg.data);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    return status;
}

static NvMediaStatus
SetCRUSSCModes(
    max96712 *priv)
{
    NvMediaStatus status;
    
    DevBlkCDII2CReg CRUSSCMode0 = {0x1445U, 0x0U};
    DevBlkCDII2CReg CRUSSCMode1 = {0x1545U, 0x0U};
    DevBlkCDII2CReg CRUSSCMode2 = {0x1645U, 0x0U};
    DevBlkCDII2CReg CRUSSCMode3 = {0x1745U, 0x0U};

    status = WriteUint8(priv->i2cProgrammer,
                                        CRUSSCMode0.address,
                                        (uint8_t)CRUSSCMode0.data);

    if (status != WICRI_STATUS_OK)
        return status;

    status = WriteUint8(priv->i2cProgrammer,
                                        CRUSSCMode1.address,
                                        (uint8_t)CRUSSCMode1.data);

    if (status != WICRI_STATUS_OK)
        return status;

    status = WriteUint8(priv->i2cProgrammer,
                                        CRUSSCMode2.address,
                                        (uint8_t)CRUSSCMode2.data);

    if (status != WICRI_STATUS_OK)
        return status;

    status = WriteUint8(priv->i2cProgrammer,
                                        CRUSSCMode3.address,
                                        (uint8_t)CRUSSCMode3.data);

    if (status != WICRI_STATUS_OK)
        return status;

    return status;
}

static NvMediaStatus
CheckCSIPLLLock(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    bool passiveEnabled = priv->ctx.passiveEnabled;

    uint8_t i = 0u;
    MipiOutModeMAX96712 mipiOutMode;
    DevBlkCDII2CReg CSIPllLockReg = {0x0400, 0x00};
    uint8_t data = 0;

    mipiOutMode = priv->ctx.mipiOutMode;

    if (!passiveEnabled) {
        for (i = 0u; i < 20u; i++) {
            status = ReadUint8(priv->i2cProgrammer,
                                               CSIPllLockReg.address,
                                               &data);
            if (status != WICRI_STATUS_OK) {
                return status;
            }

            if (((mipiOutMode == MAX96712_MIPI_OUT_2x4) && ((data & 0xF0) == 0x60)) ||
                ((mipiOutMode == MAX96712_MIPI_OUT_4x2) && ((data & 0xF0) == 0xF0))) {
                break;
            }
            us_sleep(10000);
        }

        if (((mipiOutMode == MAX96712_MIPI_OUT_2x4) && ((data & 0xF0) != 0x60)) ||
            ((mipiOutMode == MAX96712_MIPI_OUT_4x2) && ((data & 0xF0) != 0xF0))) {
            LOG_ERROR("MAX96712: CSI PLL unlock regval MSB 4bit: 0x%02x", (uint32_t)(data & 0xF0));
            return WICRI_STATUS_ERROR;
        }
    }

    return status;
}

static NvMediaStatus
GMSL2PHYOptimizationRevE(
    max96712 *priv,
    LinkMAX96712 link)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint8_t i = 0u;
    DevBlkCDII2CReg increaseCMUOutVoltageReg = {0x06C2, 0x10}; /* Increase CMU regulator output voltage (bit 4) */
    DevBlkCDII2CReg vgaHiGain_InitReg = {0x14D1, 0x03}; /* Set VgaHiGain_Init_6G (bit 1) and VgaHiGain_Init_3G (bit 0) */

    status = WriteUint8(priv->i2cProgrammer,
                                        increaseCMUOutVoltageReg.address,
                                        increaseCMUOutVoltageReg.data);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: Failed to increase CMU output voltage status:%d", (int32_t)status);
        return status;
    }

    for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
        if ((MAX96712_IS_GMSL_LINK_SET(link, i)) &&
            (IsGMSL2Mode(priv->ctx.gmslMode[i]))) {
            vgaHiGain_InitReg.address = 0x14D1 + (i * 0x100);
            status = WriteUint8(priv->i2cProgrammer,
                                                vgaHiGain_InitReg.address,
                                                vgaHiGain_InitReg.data);
            if (status != WICRI_STATUS_OK) {
                LOG_ERROR("MAX96712: Failed to set VgaHighGain_Init on link i:%d status:%d", (int32_t)i, (int32_t)status);
                return status;
            }

            LOG_INFO("MAX96712 Link %d: PHY optimization was enabled\n", i);
        }
    }

    return WICRI_STATUS_OK;
}

static NvMediaStatus
GMSL2PHYOptimization(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t i = 0u;

    if ((priv == NULL)) {
        LOG_ERROR("MAX96712: Null priv passed to GMSL2PHYOptimization");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to GMSL2PHYOptimization");
        return WICRI_STATUS_BAD_PARAMETER;
    }
    // If any link is configured in GMSL2 mode, execute the link adaptation
    if ((priv->ctx.revision == MAX96712_REV_2) ||
        (priv->ctx.revision == MAX96712_REV_3)){
        for (i = 0u; i < MAX96712_MAX_NUM_LINK; i++) {
            if (priv->ctx.gmslMode[i] != MAX96712_GMSL_MODE_UNUSED) {
                if (MAX96712_IS_GMSL_LINK_SET(priv->ctx.linkMask, i)) {
                    status = ConfigTxAmpTiming(priv, (LinkMAX96712)(1 << i));
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }

                    status = GMSL2LinkAdaptation(priv, (LinkMAX96712)(1 << i));
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                }
            }
        }
    } else if (priv->ctx.revision == MAX96712_REV_5) {
        for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
            if (priv->ctx.gmslMode[i] != MAX96712_GMSL_MODE_UNUSED) {
                if ((MAX96712_IS_GMSL_LINK_SET(priv->ctx.linkMask, i)) &&
                    (IsGMSL2Mode(priv->ctx.gmslMode[i]))) {
                    status = GMSL2PHYOptimizationRevE(priv, (LinkMAX96712)(1 << i));
                    if (status != WICRI_STATUS_OK) {
                        return status;
                    }
                }
            }
        }
    }

    return status;
}

static NvMediaStatus
EnableGPIORx(
    max96712 *priv,
    uint8_t gpioIndex)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    DevBlkCDII2CReg setGPIOMode = {0x0300, 0x1C}; /* pull-up 1M ohm, GPIO source en for GMSL2 */
    uint8_t data = 0;

    setGPIOMode.address += (gpioIndex * 3u);
    /* 0x30F, 0x31F, 0x32F are not used */
    setGPIOMode.address += ((setGPIOMode.address & 0xFF) > 0x2E) ? 3 :
                      (((setGPIOMode.address & 0xFF) > 0x1E) ? 2 :
                      (((setGPIOMode.address & 0xFF) > 0xE) ? 1 : 0));
    status = ReadUint8(priv->i2cProgrammer,
                                       setGPIOMode.address,
                                       &data);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    data |= 0x4; /* Set GPIO_RX_EN */
    data &= ~0x3; /* Unset GPIO_TX_EN, GPIO_OUT_DIS */

    return WriteUint8(priv->i2cProgrammer,
                                      setGPIOMode.address,
                                      data);
}

NvMediaStatus
MAX96712SetDefaults(
    max96712 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint8_t i = 0u;

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null priv passed to MAX96712SetDefaults");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to MAX96712SetDefaults");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (priv->ctx.revision == MAX96712_REV_3) {
        /* Bug 2446492: Disable 2-bit ECC error reporting as spurious ECC errors are
         * intermittently observed on Rev C of MAX96712
         * Disable reporting 2-bit ECC errors to ERRB
         */
        status = EnableMemoryECC(priv, false, false);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    } else if (priv->ctx.revision >= MAX96712_REV_4) {
        status = EnableMemoryECC(priv, true, true);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    if (priv->ctx.revision == MAX96712_REV_5) {
        status = SetCRUSSCModes(priv);

        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX96712: MAX96712SetDefaults: SetCRUSSCModes failed status:%d", (int32_t)status);
            return status;
        }
    }

    status = SetLinkMode(priv, (LinkMAX96712)(priv->ctx.linkMask));
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: MAX96712SetDefaults: SetLinkMode failed status:%d", (int32_t)status);
        return status;
    }

    status = GMSL2PHYOptimization(priv);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    /* Default mode is GMSL2, 6Gbps
     * one shot reset is required for GMSL1 mode & GMSL2
     */
    status = MAX96712OneShotReset(priv, (LinkMAX96712)(priv->ctx.linkMask));
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        if (MAX96712_IS_GMSL_LINK_SET(priv->ctx.linkMask, i)) {
            if (priv->ctx.gmslMode[i] == MAX96712_GMSL1_MODE) {
                /* HIM mode is not enabled yet so the link lock will not be set
                 * Instead use sleep function */
                us_sleep(100000);
            } else if ((priv->ctx.gmslMode[i] == MAX96712_GMSL2_MODE_3GBPS) ||
                       (priv->ctx.gmslMode[i] == MAX96712_GMSL2_MODE_6GBPS)) {
                status = MAX96712CheckLink(priv, priv->ctx.linkMask,MAX96712_LINK_LOCK_GMSL2, true);
                if (status != WICRI_STATUS_OK) {
                    return status;
                }
            }
        }
    }

    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
        if ((IsGMSL2Mode(priv->ctx.gmslMode[i])) &&
            (priv->ctx.longCables[i] == true)) {
            status = UpdateVGAHighGain(priv, (LinkMAX96712)(1 << i), priv->ctx.longCables[i]);
            if (status != WICRI_STATUS_OK) {
                return status;
            }
        }
    }

    status = SetI2CPort(priv);
    if (status != WICRI_STATUS_OK) {
        return status;
    }

    /* Disable all pipelines*/
    ClearRegFieldQ(priv);
    for (i = 0u; i < MAX96712_NUM_VIDEO_PIPELINES; i++) {
        ADD_ONE_REG_FIELD_RET_ERR(REG_FIELD_VIDEO_PIPE_EN_0 + i,
                                  0u);
    }
    ACCESS_REG_FIELD_RET_ERR(REG_WRITE_MODE);

    // Enable extra SMs
    if (priv->ctx.revision >= MAX96712_REV_4) {
        status = EnableExtraSMs(priv);
        if (status != WICRI_STATUS_OK) {
            return status;
        }
    }

    return WICRI_STATUS_OK;
}
EXPORT_SYMBOL(MAX96712SetDefaults);

NvMediaStatus
MAX96712SetDeviceConfig(
    max96712 *priv,
    uint32_t enumeratedDeviceConfig)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    static const char *cmdString[] = {
        [CONFIG_MAX96712_MAP_UNUSED_PIPE] =
            "CONFIG_MAX96712_MAP_UNUSED_PIPE",
        [CONFIG_MAX96712_ENABLE_PG] =
            "CONFIG_MAX96712_ENABLE_PG",
        [CONFIG_MAX96712_DISABLE_PG] =
            "CONFIG_MAX96712_DISABLE_PG",
        [CONFIG_MAX96712_ENABLE_BACKTOP] =
            "CONFIG_MAX96712_ENABLE_BACKTOP",
        [CONFIG_MAX96712_DISABLE_BACKTOP] =
            "CONFIG_MAX96712_DISABLE_BACKTOP",
        [CONFIG_MAX96712_TRIGGER_DESKEW] =
            "CONFIG_MAX96712_TRIGGER_DESKEW",
        [CONFIG_MAX96712_CHECK_CSIPLL_LOCK] =
            "CONFIG_MAX96712_CHECK_CSIPLL_LOCK",
        [CONFIG_MAX96712_ENABLE_REPLICATION] =
            "CONFIG_MAX96712_ENABLE_REPLICATION",
        [CONFIG_MAX96712_DISABLE_REPLICATION] =
            "CONFIG_MAX96712_DISABLE_REPLICATION",
    };

    if ((priv == NULL)) {
        LOG_ERROR("MAX96712: Null priv passed to MAX96712SetDefaults");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to MAX96712SetDefaults");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if ((enumeratedDeviceConfig == CONFIG_MAX96712_INVALID) ||
        (enumeratedDeviceConfig >= CONFIG_MAX96712_NUM)) {
        LOG_ERROR("MAX96712: Bad parameter: Invalid command");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    LOG_INFO("MAX96712: %s", cmdString[enumeratedDeviceConfig]);
    switch (enumeratedDeviceConfig) {
        case CONFIG_MAX96712_ENABLE_PG:
            status = EnablePG(priv,true);
            break;
        case CONFIG_MAX96712_DISABLE_PG:
            status = EnablePG(priv,false);
            break;
        case CONFIG_MAX96712_MAP_UNUSED_PIPE:
            status = MapUnusedPipe(priv);
            break;
        case CONFIG_MAX96712_ENABLE_BACKTOP:
            status = EnableBackTop(priv,
                                   true);
            break;
        case CONFIG_MAX96712_DISABLE_BACKTOP:
            status = EnableBackTop(priv,
                                   false);
            break;
        case CONFIG_MAX96712_TRIGGER_DESKEW:
            status = TriggerDeskew(priv);
            break;
        case CONFIG_MAX96712_CHECK_CSIPLL_LOCK:
            status = CheckCSIPLLLock(priv);
            break;
        case CONFIG_MAX96712_ENABLE_REPLICATION:
            status = EnableReplication(priv, true);
            break;
        case CONFIG_MAX96712_DISABLE_REPLICATION:
            status = EnableReplication(priv, false);
            break;
        case CONFIG_MAX96712_ENABLE_ERRB:
            status = EnableERRB(priv, true);
            break;
        case CONFIG_MAX96712_DISABLE_ERRB:
            status = EnableERRB(priv, false);
            break;
        default:
            LOG_ERROR("MAX96712: Bad parameter: Unrecognized command");
            status = WICRI_STATUS_BAD_PARAMETER;
            break;
    }

    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: command failed parameterType:%s", cmdString[enumeratedDeviceConfig]);
    }

    return status;
}
EXPORT_SYMBOL(MAX96712SetDeviceConfig);

NvMediaStatus
MAX96712WriteRegister(
    max96712 *priv,
    uint32_t deviceIndex,
    uint32_t registerNum,
    uint32_t dataLength,
    uint8_t *dataBuff)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint32_t i = 0u;

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null priv passed to MAX96712WriteRegister");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to MAX96712WriteRegister");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (dataBuff == NULL) {
        LOG_ERROR("MAX96712: Null data buffer passed to MAX96712WriteRegister");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (dataLength > REG_WRITE_BUFFER_BYTES) {
        LOG_ERROR("MAX96712: Insufficient buffer size in MAX96712WriteRegister");
        return WICRI_STATUS_INSUFFICIENT_BUFFERING;
    }

    for (i = 0; i < dataLength; i++) {
        status = WriteUint8(priv->i2cProgrammer,
                                            registerNum,
                                            dataBuff[i]);
    }
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: Register I2C write failed with status registerNum:0x%04x status:%d", registerNum, (uint32_t)status);
    }

    return status;
}

NvMediaStatus
MAX96712WriteParameters(
    max96712 *priv,
    uint32_t parameterType,
    size_t parameterSize,
    void *parameter)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    WriteParametersParamMAX96712 *param = (WriteParametersParamMAX96712 *) parameter;
    bool isValidSize = false;
    static const char *cmdString[] = {
        [WRITE_PARAM_CMD_MAX96712_ENABLE_SPECIFIC_LINKS] =
            "WRITE_PARAM_CMD_MAX96712_ENABLE_SPECIFIC_LINKS",
        [WRITE_PARAM_CMD_MAX96712_DISABLE_FORWARD_CHANNELS] =
            "WRITE_PARAM_CMD_MAX96712_DISABLE_FORWARD_CHANNELS",
        [WRITE_PARAM_CMD_MAX96712_ENABLE_FORWARD_CHANNELS] =
            "WRITE_PARAM_CMD_MAX96712_ENABLE_FORWARD_CHANNELS",
        [WRITE_PARAM_CMD_MAX96712_ENABLE_PACKET_BASED_CONTROL_CHANNEL] =
            "WRITE_PARAM_CMD_MAX96712_ENABLE_PACKET_BASED_CONTROL_CHANNEL",
        [WRITE_PARAM_CMD_MAX96712_DISABLE_DE] =
            "WRITE_PARAM_CMD_MAX96712_DISABLE_DE",
        [WRITE_PARAM_CMD_MAX96712_SET_DEFAULT_GMSL1_HIM_ENABLED] =
            "WRITE_PARAM_CMD_MAX96712_SET_DEFAULT_GMSL1_HIM_ENABLED",
        [WRITE_PARAM_CMD_MAX96712_SET_DBL] =
            "WRITE_PARAM_CMD_MAX96712_SET_DBL",
        [WRITE_PARAM_CMD_MAX96712_SET_FSYNC] =
            "WRITE_PARAM_CMD_MAX96712_SET_FSYNC",
        [WRITE_PARAM_CMD_MAX96712_ENABLE_DOUBLE_PIXEL_MODE] =
            "WRITE_PARAM_CMD_MAX96712_ENABLE_DOUBLE_PIXEL_MODE",
        [WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING] =
            "WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING",
        [WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_YUV] =
            "WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_YUV",
        [WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_TPG] =
            "WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_TPG",
        [WRITE_PARAM_CMD_MAX96712_OVERRIDE_DATATYPE] =
            "WRITE_PARAM_CMD_MAX96712_OVERRIDE_DATATYPE",
        [WRITE_PARAM_CMD_MAX96712_SET_MIPI] =
            "WRITE_PARAM_CMD_MAX96712_SET_MIPI",
        [WRITE_PARAM_CMD_MAX96712_SET_TX_SRC_ID] =
            "WRITE_PARAM_CMD_MAX96712_SET_TX_SRC_ID",
        [WRITE_PARAM_CMD_MAX96712_UNSET_DBL] =
            "WRITE_PARAM_CMD_MAX96712_UNSET_DBL",
        [WRITE_PARAM_CMD_MAX96712_DISABLE_PACKET_BASED_CONTROL_CHANNEL] =
            "WRITE_PARAM_CMD_MAX96712_DISABLE_PACKET_BASED_CONTROL_CHANNEL",
        [WRITE_PARAM_CMD_MAX96712_ENABLE_PERIODIC_AEQ] =
            "WRITE_PARAM_CMD_MAX96712_ENABLE_PERIODIC_AEQ",
        [WRITE_PARAM_CMD_MAX96712_DISABLE_AUTO_ACK] =
            "WRITE_PARAM_CMD_MAX96712_DISABLE_AUTO_ACK",
        [WRITE_PARAM_CMD_MAX96712_ENABLE_GPIO_RX] =
            "WRITE_PARAM_CMD_MAX96712_ENABLE_GPIO_RX",
        [WRITE_PARAM_CMD_MAX96712_SET_PG] =
            "WRITE_PARAM_CMD_MAX96712_SET_PG",
    };

    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null priv passed to MAX96712WriteParameters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if (priv == NULL) {
        LOG_ERROR("MAX96712: Null driver priv passed to MAX96712WriteParameters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (parameter == NULL) {
        LOG_ERROR("MAX96712: Null parameter storage passed to MAX96712WriteParameters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if ((parameterType == WRITE_PARAM_CMD_MAX96712_INVALID) ||
        (parameterType >= WRITE_PARAM_CMD_MAX96712_NUM)) {
        LOG_ERROR("MAX96712: Bad parameter: Invalid command");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    LOG_INFO("MAX96712: %s", cmdString[parameterType]);
    switch (parameterType) {
        case WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING:
            if (parameterSize == sizeof(param->PipelineMapping)) {
                isValidSize = true;
                status = SetPipelineMap(priv,
                                        param->PipelineMapping.link,
                                        param->PipelineMapping.linkPipelineMap);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_YUV:
            if (parameterSize == sizeof(param->PipelineMapping)) {
                isValidSize = true;
                status = SetYuvPipelineMap(priv,
                                        param->PipelineMapping.link,
                                        param->PipelineMapping.linkPipelineMap);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_TPG:
            if (parameterSize == sizeof(param->PipelineMapping)) {
                isValidSize = true;
                status = SetPipelineMapTPG(priv,
                                           param->PipelineMappingTPG.linkIndex,
                                           param->PipelineMappingTPG.linkPipelineMap);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_OVERRIDE_DATATYPE:
            if (parameterSize == sizeof(param->PipelineMapping)) {
                isValidSize = true;
                status = OverrideDataType(priv,
                                          param->PipelineMapping.link,
                                          param->PipelineMapping.linkPipelineMap);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_SET_FSYNC:
            if (parameterSize == sizeof(param->FSyncSettings)) {
                isValidSize = true;
                status = SetFSYNCMode(priv,
                                      param->FSyncSettings.FSyncMode,
                                      param->FSyncSettings.pclk,
                                      param->FSyncSettings.fps,
                                      param->FSyncSettings.link);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_ENABLE_SPECIFIC_LINKS:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = EnableSpecificLinks(priv,
                                             param->link);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_DISABLE_FORWARD_CHANNELS:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = ControlForwardChannels(priv,
                                                   param->link,
                                                   false);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_ENABLE_FORWARD_CHANNELS:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = ControlForwardChannels(priv,
                                                   param->link,
                                                   true);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_ENABLE_PACKET_BASED_CONTROL_CHANNEL:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = EnablePacketBasedControlChannel(priv,
                                                         param->link,
                                                         true);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_DISABLE_DE:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = DisableDE(priv,
                                   param->link);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_SET_DEFAULT_GMSL1_HIM_ENABLED:
            if (parameterSize == sizeof(param->GMSL1HIMEnabled)) {
                isValidSize = true;
                status = SetDefaultGMSL1HIMEnabled(priv,
                                                   param->GMSL1HIMEnabled.link,
                                                   param->GMSL1HIMEnabled.step);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_SET_DBL:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = SetDBL(priv,
                                param->link,
                                true);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_ENABLE_DOUBLE_PIXEL_MODE:
            if (parameterSize == sizeof(param->DoublePixelMode)) {
                isValidSize = true;
                status = EnableDoublePixelMode(priv,
                                               param->DoublePixelMode.link,
                                               param->DoublePixelMode.dataType,
                                               param->DoublePixelMode.embDataType,
                                               param->DoublePixelMode.isSharedPipeline);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_SET_MIPI:
            if (parameterSize == sizeof(param->MipiSettings)) {
                isValidSize = true;
                status = ConfigureMIPIOutput(priv,
                                             param->MipiSettings.mipiSpeed,
                                             param->MipiSettings.phyMode);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_SET_TX_SRC_ID:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = SetTxSRCId(priv,
                                    param->link);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_UNSET_DBL:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = SetDBL(priv,
                                param->link,
                                false);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_DISABLE_PACKET_BASED_CONTROL_CHANNEL:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = EnablePacketBasedControlChannel(priv,
                                                         param->link,
                                                         false);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_ENABLE_PERIODIC_AEQ:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = EnablePeriodicAEQ(priv,
                                           param->link);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_DISABLE_AUTO_ACK:
            if (parameterSize == sizeof(param->link)) {
                isValidSize = true;
                status = DisableAutoAck(priv,
                                        param->link);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_ENABLE_GPIO_RX:
            if (parameterSize == sizeof(param->gpioIndex)) {
                isValidSize = true;
                status = EnableGPIORx(priv,
                                      param->gpioIndex);
            }
            break;
        case WRITE_PARAM_CMD_MAX96712_SET_PG:
            if (parameterSize == sizeof(param->SetPGSetting)) {
                isValidSize = true;
                status = ConfigPGSettings(priv,
                                          param->SetPGSetting.width,
                                          param->SetPGSetting.height,
                                          param->SetPGSetting.frameRate,
                                          param->SetPGSetting.linkIndex);
            }
            break;
        default:
            LOG_ERROR("MAX96712: Bad parameter: Unrecognized command");
            isValidSize = true;
            status = WICRI_STATUS_BAD_PARAMETER;
            break;
    }

    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: Command failed parameterType:%s", cmdString[parameterType]);
    }

    if (!isValidSize) {
        LOG_ERROR("MAX96712: Bad parameter: Invalid param size parameterType:%s", cmdString[parameterType]);
        status = WICRI_STATUS_BAD_PARAMETER;
    }

    return status;
}
EXPORT_SYMBOL(MAX96712WriteParameters);

LinkMAX96712
GetMAX96712Link(
    uint8_t linkNum)
{
    switch (linkNum) {
        case 0u:
            return MAX96712_LINK_0;
        case 1u:
            return MAX96712_LINK_1;
        case 2u:
            return MAX96712_LINK_2;
        case 3u:
            return MAX96712_LINK_3;
        default:
            return MAX96712_LINK_NONE;
    }
}
EXPORT_SYMBOL(GetMAX96712Link);
const struct of_device_id max96712_of_match[] = {
	{ .compatible = "maxim,max96712", },
	{ },
};
MODULE_DEVICE_TABLE(of, max96712_of_match);

static NvMediaStatus
Doinit(
    max96712 *priv,
    WriteParametersParamMAX96712 writeParamsMAX96712)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    ReadParametersParamMAX96712 readParamsMAX96712 = {};
    /* I2C wake time typically takes 2.25ms
     * TODO: add the link lock time up to 60ms
     */
	msleep(3);

    /*! Check deserializer is present */
    LOG_INFO("Check deserializer is present\n");
    status = MAX96712CheckPresence(priv);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: MAX96712CheckPresence failed with NvMedia error %d", (int32_t)status);
        return status;
    }

    LOG_INFO("Set deserializer defaults\n");
    status = MAX96712SetDefaults(priv);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: MAX96712SetDefaults failed with NvMedia error %d", (int32_t)status);
        return status;
    }
	msleep(50);

    /* Get deserializer revision */
    LOG_INFO("Get deserializer revision\n");
    status = MAX96712ReadParameters(priv,
                                       READ_PARAM_CMD_MAX96712_REV_ID,
                                       sizeof(readParamsMAX96712.revision),
                                       &readParamsMAX96712);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: MAX96712ReadParameters(READ_PARAM_CMD_MAX96712_REV_ID) failed with NvMedia error %d", status);
        return status;
    }

    /* Set MIPI output mode */
    LOG_INFO("Set MIPI output mode\n");

    if (writeParamsMAX96712.MipiSettings.mipiSpeed == 0)
    {
        LOG_ERROR("send error writeParamsMAX96712 parameter!");
        return WICRI_STATUS_BAD_PARAMETER;
    }
    status = MAX96712WriteParameters(priv,
                                        WRITE_PARAM_CMD_MAX96712_SET_MIPI,
                                        sizeof(writeParamsMAX96712.MipiSettings),
                                        &writeParamsMAX96712);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: MAX96712WriteParameters(WRITE_PARAM_CMD_MAX96712_SET_MIPI) failed with NvMedia error %d", (int32_t)status);
        return status;
    }
	msleep(20);


    return WICRI_STATUS_OK;
}
NvMediaStatus
EnableLinks(
    max96712 *priv,
    LinkMAX96712 linkMask)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    ReadParametersParamMAX96712 readParamsMAX96712 = {};
    WriteParametersParamMAX96712 writeParamsMAX96712 = {};
	uint8_t link = 0u;
    /* Get the links currently enabled */
    printk("alex:MAX96712 read link status\n");
    status = MAX96712ReadParameters(priv,
                                       READ_PARAM_CMD_MAX96712_ENABLED_LINKS,
                                       sizeof(readParamsMAX96712.link),
                                       &readParamsMAX96712);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX96712: MAX96712ReadParameters(READ_PARAM_CMD_MAX96712_ENABLED_LINKS) failed with NvMedia error %d", (int32_t)status);
        return status;
    }
    link = readParamsMAX96712.link;
    if (link != linkMask) {
        LOG_INFO("Enabling links\n");
        writeParamsMAX96712.link = (LinkMAX96712) linkMask;
        printk("alex:MAX96712 read link status\n");
        status = MAX96712WriteParameters(priv,
                                            WRITE_PARAM_CMD_MAX96712_ENABLE_SPECIFIC_LINKS,
                                            sizeof(writeParamsMAX96712.link),
                                            &writeParamsMAX96712);
        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX96712: MAX96712WriteParameters(WRITE_PARAM_CMD_MAX96712_ENABLE_SPECIFIC_LINK) failed with NvMedia error %d", (int32_t)status);
            return status;
        }
    }
    return WICRI_STATUS_OK;
}
EXPORT_SYMBOL(EnableLinks);

NvMediaStatus max96712_power_on(struct device *dev)
{
	max96712 *priv = dev_get_drvdata(dev);
	NvMediaStatus err = WICRI_STATUS_OK;

	mutex_lock(&priv->lock);
	if (priv->pw_ref == 0) {
		usleep_range(1, 2);
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

	priv->pw_ref++;
	mutex_unlock(&priv->lock);

	return err;
}
EXPORT_SYMBOL(max96712_power_on);

void max96712_power_off(struct device *dev)
{
	max96712 *priv = dev_get_drvdata(dev);

	mutex_lock(&priv->lock);
	priv->pw_ref--;

	if (priv->pw_ref == 0) {
		/* enter reset mode: XCLR */
		usleep_range(1, 2);
		if (priv->reset_gpio)
			gpio_set_value(priv->reset_gpio, 0);
	}

	mutex_unlock(&priv->lock);
}
EXPORT_SYMBOL(max96712_power_off);

int read_property_u32(
	struct device_node *node, const char *name, u32 *value)
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

static int max96712_parse_dt(max96712 *priv,
				struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	int err = 0;
	const char *str_value;
    u32 value;
    int i;
    const char *gmsl_mode[4];
	const struct of_device_id *match;

	if (!node)
		return -EINVAL;

	match = of_match_device(max96712_of_match, &client->dev);
	if (!match) {
		LOG_ERROR("Failed to find matching dt id\n");
		return -EFAULT;
	}
    priv->ctx.linkMask = 0xF;
    for (i = 0; i < MAX96712_MAX_NUM_LINK; i++) {
		of_property_read_string_index(node, "gmsl-mode", i,
						&gmsl_mode[i]);
		if (!gmsl_mode[i]) {
			LOG_ERROR("invalid GMSL mode info\n");
		}
		if (!strcmp(gmsl_mode[i], "gmsl2-6gbps")) {
			priv->ctx.gmslMode[i] = MAX96712_GMSL2_MODE_6GBPS;
		} else if (!strcmp(gmsl_mode[i], "gmsl2-3gbps")) {
			priv->ctx.gmslMode[i] = MAX96712_GMSL2_MODE_3GBPS;
		} else if (!strcmp(gmsl_mode[i], "gmsl1")) {
			priv->ctx.gmslMode[i] = MAX96712_GMSL1_MODE;
		} else {
            priv->ctx.linkMask ^= (1 << i); 
			LOG_ERROR( "No support's or None GMSL mode\n");
			priv->ctx.gmslMode[i] = MAX96712_GMSL_MODE_UNUSED;
		}
	}
    LOG_INFO( "MAX96712: register:0x%02x", priv->ctx.linkMask);
	err = read_property_u32(node, "i2cport", &value);
	if (err < 0) {
		dev_err(&client->dev, "i2cport property not found\n");
		return err;
	}
    switch (value)
    {
    case 0:
        priv->ctx.i2cPort = MAX96712_I2CPORT_0;
        break;
    case 1:
        priv->ctx.i2cPort = MAX96712_I2CPORT_1;
        break;
    case 2:
        priv->ctx.i2cPort = MAX96712_I2CPORT_2;
        break;
    default:
        LOG_ERROR("not match i2cPort");
        break;
    }
    

    err = read_property_u32(node, "txport", &value);
	if (err < 0) {
		dev_err(&client->dev, "txport property not found\n");
		return err;
	}
    switch (value)
    {
    case 0:
        priv->ctx.txPort  = MAX96712_TXPORT_PHY_C;
        break;
    case 1:
        priv->ctx.txPort  = MAX96712_TXPORT_PHY_D;
        break;
    case 2:
        priv->ctx.txPort  = MAX96712_TXPORT_PHY_E;
        break;
    case 3:
        priv->ctx.txPort  = MAX96712_TXPORT_PHY_F;
        break;
    default:
        LOG_ERROR("not match txPort");
        break;
    }
    priv->ctx.passiveEnabled = false;
	err = of_property_read_string(node, "phy-mode", &str_value);
	if (err) {
		dev_dbg(&client->dev, "%s: use default phy mode DPHY\n", __func__);
		priv->ctx.phyMode = MAX96712_PHY_MODE_DPHY;
	} else {
		if (strcmp(str_value, "CPHY") == 0)
			priv->ctx.phyMode = MAX96712_PHY_MODE_CPHY;
		else if (strcmp(str_value, "DPHY") == 0)
			priv->ctx.phyMode = MAX96712_PHY_MODE_DPHY;
		else {
			dev_err(&client->dev, "%s: Invalid Phy mode\n", __func__);
			return -EINVAL;
		}
	}


    /**
     * @brief Get csi mode 
     * 
     */
	err = of_property_read_string(node, "csi-mode", &str_value);
	if (err < 0) {
		dev_err(&client->dev, "csi-mode property not found\n");
		return err;
	}

	if (!strcmp(str_value, "2x4")) {
        LOG_INFO("MAX96712_MIPI_OUT_2x4 select!");
		priv->ctx.mipiOutMode = MAX96712_MIPI_OUT_2x4;
        for (i = 0U; i < MAX96712_MAX_NUM_PHY; i++) {
            priv->ctx.lanes[i] = 4U;
        }
	} else if (!strcmp(str_value, "4x2")) {
		priv->ctx.mipiOutMode = MAX96712_MIPI_OUT_4x2;
        for (i = 0U; i < MAX96712_MAX_NUM_PHY; i++) {
            priv->ctx.lanes[i] = 2U;
        }

	} else {
		dev_err(&client->dev, "invalid csi mode\n");
		return -EINVAL;
	}


	priv->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (priv->reset_gpio < 0) {
		dev_err(&client->dev, "reset-gpios not found %d\n", err);
		return err;
	}

	return 0;
}


static struct regmap_config max96712_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max96712_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
    max96712 *priv;
	int err = 0;
    NvMediaStatus nvmStatus = WICRI_STATUS_OK;
	WriteParametersParamMAX96712 writeParamsMAX96712={};
	dev_info(&client->dev, "[MAX96712]: probing GMSL Deserializer\n");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->i2cProgrammer = devm_regmap_init_i2c(priv->i2c_client,
				&max96712_regmap_config);
	if (IS_ERR(priv->i2cProgrammer)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->i2cProgrammer));
		return -ENODEV;
	}

	err = max96712_parse_dt(priv, client);
	if (err) {
		dev_err(&client->dev, "unable to parse dt\n");
		return -EFAULT;
	}
    writeParamsMAX96712.MipiSettings.mipiSpeed = 8;
	writeParamsMAX96712.MipiSettings.phyMode = priv->ctx.phyMode;
	nvmStatus=Doinit(priv,writeParamsMAX96712);
    printk(KERN_ERR "mipiSpeed=%d\n",writeParamsMAX96712.MipiSettings.mipiSpeed);
	if (nvmStatus!=WICRI_STATUS_OK)
	{
		LOG_ERROR("MAX96712:Doinit Failed!");
        return nvmStatus;
	}
	mutex_init(&(priv->lock));

	dev_set_drvdata(&client->dev, priv);

	/* dev communication gets validated when GMSL link setup is done */
	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}


static int max96712_remove(struct i2c_client *client)
{
	max96712 *priv;

	if (client != NULL) {
		priv = dev_get_drvdata(&client->dev);
		mutex_destroy(&(priv->lock));
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max96712_id[] = {
	{ "max96712", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, max96712_id);

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
MODULE_AUTHOR("Huangzeng <huangzeng@wicri.org");
MODULE_LICENSE("GPL v2");
