#define DEBUG
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <media/camera_common.h>
#include <linux/module.h>
#include <media/gmsl-common.h>
#include <media/nv_max9295.h>

#define GET_BLOCK_LENGTH(x)            x[0]
#define GET_SIZE(x)                    sizeof(x)
#define GET_BLOCK_DATA(x)              &x[1]
#define SET_NEXT_BLOCK(x)              x += (x[0] + 1)

#define MAX9295_NUM_ADDR_BYTES         2u
#define MAX9295_NUM_DATA_BYTES         1u
#define REG_WRITE_BUFFER_BYTES         MAX9295_NUM_DATA_BYTES
#define MAX9295_DEVICE_INDEX       0u

#define REG_DEV_ID_ADDR                0x0D
#define MAX9295A_DEV_ID                0x91
#define MAX9295B_DEV_ID                0x93
#define REG_DEV_REV_ADDR               0x0E
#define REG_LFLT_INT                   0x1B
#define MAX9295_REG_MAX_ADDRESS        0x1576

typedef struct {
    RevisionMAX9295 revId;
    uint32_t revVal;
} Revision;

/* These values must include all of values in the RevisionMAX9295 enum */
static Revision supportedRevisions[] = {
    {MAX9295_REV_5, 5u},
    {MAX9295_REV_7, 7u},
    {MAX9295_REV_8, 8u},
};

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
    LOG_INFO("MAX9295:0x%04x,0x%02x ",addr,val);
    if (err!=0)
    {
        LOG_ERROR("i2c Write error! addr:%04x val:%02x",addr,val);
        return WICRI_STATUS_BAD_PARAMETER;
    }
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
            usleep_range(regList->regs[i].delayUsec,regList->regs[i].delayUsec);
        }
        usleep_range(10,10);
    }
    
    return status;
}

static NvMediaStatus
SetDeviceAddress(
    max9295 *priv,
    uint8_t address)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg setAddrRegs[] = {{0x0000, 0x0}};
    DevBlkCDII2CRegList setAddr = {
        .regs = setAddrRegs,
        .numRegs = I2C_ARRAY_SIZE(setAddrRegs),
    };

    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: null driver priv passed to SetDeviceAddress");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    /* Check for 7 bit I2C address */
    if (address >= 0x80) {
        LOG_ERROR("MAX9295: Bad parameter: Address is greater than 0x80 %02x", (uint32_t)address);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    setAddrRegs[0].data = address << 1;

    status = WriteArray(priv->i2cProgrammer, &setAddr);
    if (WICRI_STATUS_OK != status) {
        LOG_ERROR("MAX9295: Failed to set device address %d", (int32_t)status);
        LOG_ERROR("MAX9295:  Attempting to set address %02x", (uint32_t)address);
    }

done:
    return status;
}

static NvMediaStatus
SetTranslator(
    max9295 *priv,
    uint32_t parameterType,
    uint8_t source,
    uint8_t destination)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg setTranslatorRegs[] = {
        {0x0042, 0x00},
        {0x0043, 0x00},
    };
    DevBlkCDII2CRegList setTranslator = {
        .regs = setTranslatorRegs,
        .numRegs = I2C_ARRAY_SIZE(setTranslatorRegs),
    };



    /* Check for 7 bit I2C address */
    if ((source >= 0x80) || (destination >= 0x80)) {
        LOG_ERROR("MAX9295: Source or destination address out of 7-bit range");
        LOG_ERROR("MAX9295:  - source address %02x", (uint32_t) source);
        LOG_ERROR("MAX9295:  - destination address %02x", (uint32_t) destination);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (parameterType == WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_B) {
        setTranslatorRegs[0].address += 2;
        setTranslatorRegs[1].address += 2;
    }

    setTranslatorRegs[0].data = source << 1;
    setTranslatorRegs[1].data = destination << 1;

    status = WriteArray(priv->i2cProgrammer, &setTranslator);
    if (WICRI_STATUS_OK != status) {
        LOG_ERROR("MAX9295: Set translation I2C write failed %d", (int32_t)status);
    }

done:
    return status;
}

static NvMediaStatus
ConfigPipelines(
    max9295 *priv,
    DataTypeMAX9295 dataType,
    bool embDataType)
{
    /* Configure pipeline X for pixel data and
     * pipeline Y for emb data if enabled */
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg startPipeFromPortRegs[] = {{0x0311, 0x00}};
    DevBlkCDII2CRegList startPipeFromPort = {
        .regs = startPipeFromPortRegs,
        .numRegs = I2C_ARRAY_SIZE(startPipeFromPortRegs),
    };

    DevBlkCDII2CReg pipePortMapRegs[] = {{0x0308, 0x40}};
    DevBlkCDII2CRegList pipePortMap = {
        .regs = pipePortMapRegs,
        .numRegs = I2C_ARRAY_SIZE(pipePortMapRegs),
    };

    DevBlkCDII2CReg videoTxEnRegs[] = {{0x0002, 0x13}};
    DevBlkCDII2CRegList videoTxEn = {
        .regs = videoTxEnRegs,
        .numRegs = I2C_ARRAY_SIZE(videoTxEnRegs),
    };

    DevBlkCDII2CReg mappingPixelRegs[] = {
                        {0x0314, 0x6C}, /* Route 12bit RAW to VIDEO_X (MSB enable) */
                        {0x0053, 0x10}, /* Stream ID for packets for VIDEO_X */
                    };
    DevBlkCDII2CRegList mappingPixel = {
        .regs = mappingPixelRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingPixelRegs),
    };

    DevBlkCDII2CReg mappingEmbRegs[] = {
                        {0x0316, 0x52}, /* Route EMBEDDED8 to VIDEO_Y (MSB enable) */
                        {0x0057, 0x11}, /* Stream ID for packets for VIDEO_Y */
                    };
    DevBlkCDII2CRegList mappingEmb = {
        .regs = mappingEmbRegs,
        .numRegs = I2C_ARRAY_SIZE(mappingEmbRegs),
    };

    DevBlkCDII2CReg doubleBpp12Bpp8Regs[] = {
                        {0x0312, 0x02},
                        {0x0313, 0x10},
                        {0x031C, 0x38},
                        {0x031D, 0x30},
                    };
    DevBlkCDII2CRegList doubleBpp12Bpp8 = {
        .regs = doubleBpp12Bpp8Regs,
        .numRegs = I2C_ARRAY_SIZE(doubleBpp12Bpp8Regs),
    };

    DevBlkCDII2CReg disableHeartbeatRegs[] = {
                        {0x0102, 0x0E},
                        {0x010A, 0x0E},
                    };
    DevBlkCDII2CRegList disableHeartbeat = {
        .regs = disableHeartbeatRegs,
        .numRegs = I2C_ARRAY_SIZE(disableHeartbeatRegs),
    };



    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver driver priv passed to ConfigPipelines");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }
    startPipeFromPortRegs[0].data = 0x10;
    pipePortMapRegs[0].data |= 0x21;
    if (embDataType) {
        startPipeFromPortRegs[0].data |= 0x20;
        pipePortMapRegs[0].data |= 0x02;
        videoTxEnRegs[0].data |= 0x20;
    }
    /* Update mapping table with data type */
    switch (dataType) {
        case MAX9295_DATA_TYPE_RAW10:
            mappingPixelRegs[0].data = 0x6B;
            break;
        case MAX9295_DATA_TYPE_RAW12:
            mappingPixelRegs[0].data = 0x6C;
            break;
        case MAX9295_DATA_TYPE_RAW16:
            mappingPixelRegs[0].data = 0x6E;
            break;
        case MAX9295_DATA_TYPE_YUV_8:
            mappingPixelRegs[0].data = 0xf0;
            startPipeFromPortRegs[0].data |= 0x20;
            pipePortMapRegs[0].data |= 0x22;
            videoTxEnRegs[0].data |= 0x20;
            break;
        default:
            LOG_ERROR("MAX9295: Invalid data type passed to ConfigPipelines %d", (uint32_t)dataType);
            status = WICRI_STATUS_BAD_PARAMETER;
            goto done;
    }


    status = WriteArray(priv->i2cProgrammer, &startPipeFromPort);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Start pipe from port write failed %d", (int32_t)status);
        goto done;
    }

    status = WriteArray(priv->i2cProgrammer, &pipePortMap);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Pipe port map write failed %d", (int32_t)status);
        goto done;
    }

    status = WriteArray(priv->i2cProgrammer, &mappingPixel);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Mapping pixel write failed %d", (int32_t)status);
        goto done;
    }

    if (embDataType) {
        status = WriteArray(priv->i2cProgrammer, &mappingEmb);
        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX9295: Mapping emb write failed %d", (int32_t)status);
            goto done;
        }
    }

    status = WriteArray(priv->i2cProgrammer, &videoTxEn);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Video tx enable write failed %d", (int32_t)status);
        goto done;
    }

    /* Turn on double mode only if emb data type is enabled and pixel data type is RAW12/RAW10 */
    if (dataType == MAX9295_DATA_TYPE_RAW10) {
        doubleBpp12Bpp8Regs[1].data = 0x1;
        doubleBpp12Bpp8Regs[2].data = 0x34;
    }

    status = WriteArray(priv->i2cProgrammer, &doubleBpp12Bpp8);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Double BPP12BPP8 write failed %d", (int32_t)status);
        goto done;
    }

    status = WriteArray(priv->i2cProgrammer, &disableHeartbeat);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Disable heart beat write failed %d", (int32_t)status);
        goto done;
    }

done:
    return status;
}

static NvMediaStatus
ConfigPhy(
    max9295 *priv,
    phyMapMAX9295 *mapping,
    phyPolarityMAX9295 *polarity,
    uint8_t numDataLanes)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg phyMapRegs[] = {
        {0x0330, 0x00},
        {0x0331, 0x00},
        {0x0332, 0xE0},
        {0x0333, 0x04}
    };
    DevBlkCDII2CRegList phyMap = {
        .regs = phyMapRegs,
        .numRegs = I2C_ARRAY_SIZE(phyMapRegs),
    };

    DevBlkCDII2CReg phyPolarityRegs[] = {
        {0x0334, 0x00},
        {0x0335, 0x00}
    };
    DevBlkCDII2CRegList phyPolarity = {
        .regs = phyPolarityRegs,
        .numRegs = I2C_ARRAY_SIZE(phyPolarityRegs),
    };


    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to ConfigPhy");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (mapping == NULL) {
        LOG_ERROR("MAX9295: Null mapping passed to ConfigPhy");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (polarity == NULL) {
        LOG_ERROR("MAX9295: Null polarity passed to ConfigPhy");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if ((numDataLanes != 2) && (numDataLanes != 4)) {
        LOG_ERROR("MAX9295: Number of data lanes was not 2 or 4  %d", (int32_t)numDataLanes);
        status = WICRI_STATUS_NOT_SUPPORTED;
        goto done;
    }

    if ((numDataLanes == 2) && (mapping->enableMapping)) {
        LOG_ERROR("MAX9295: Lane swapping is supported only in 4 lane mode");
        status = WICRI_STATUS_NOT_SUPPORTED;
        goto done;
    }

    status = ReadUint8(priv->i2cProgrammer, phyMapRegs[1].address, (uint8_t *) &(phyMapRegs[1].data));
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set phy map read failed %d", (int32_t)status);
        goto done;
    }

    /* data lanes indexing starts at 0 (0 = 1 lane, 1 = 2 lanes, etc) */
    numDataLanes -= 1;
    /* clear the data lanes settings for Port B */
    phyMapRegs[1].data &= ~0x30;
    /* Set num data lanes for Port B */
    phyMapRegs[1].data |= (numDataLanes << 4);

    if (mapping->enableMapping) {
        phyMapRegs[2].data = (mapping->phy1_d1 << 6) |
                             (mapping->phy1_d0 << 4) |
                             (mapping->phy0_d1 << 2) |
                             (mapping->phy0_d0);
        phyMapRegs[3].data = (mapping->phy3_d1 << 6) |
                             (mapping->phy3_d0 << 4) |
                             (mapping->phy2_d1 << 2) |
                             (mapping->phy2_d0);
    }

    status = WriteArray(priv->i2cProgrammer, &phyMap);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set phy map write failed %d", (int)status);
        goto done;
    }

    if (polarity->setPolarity) {
        phyPolarityRegs[0].data = (polarity->phy1_clk << 6) |
                                  (polarity->phy1_d1  << 5) |
                                  (polarity->phy1_d0  << 4);
        phyPolarityRegs[1].data = (polarity->phy2_clk << 2) |
                                  (polarity->phy2_d1  << 1) |
                                  (polarity->phy2_d0);

        status = WriteArray(priv->i2cProgrammer, &phyPolarity);
        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX9295: Set phy polarity write failed %d", (int32_t)status);
            goto done;
        }
    }

done:
    return status;
}

static NvMediaStatus
SetGPIOOutput(
    max9295 *priv,
    GPIOTypeMAX9295 gpio,
    bool level)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg setGPIOAModeRegs[] = {{0x02BE, 0x80}}; /* pull-up 1M ohm, output driver enabled */
    DevBlkCDII2CRegList setGPIOAMode = {
        .regs = setGPIOAModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setGPIOAModeRegs),
    };

    DevBlkCDII2CReg setGPIOBModeRegs[] = {{0x02BF, 0x60}}; /* pull-up, output push-pull */
    DevBlkCDII2CRegList setGPIOBMode = {
        .regs = setGPIOBModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setGPIOBModeRegs),
    };


    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to SetGPIOOutput");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (gpio == MAX9295_GPIO_TYPE_INVALID) {
        LOG_ERROR("MAX9295: Invalid GPIO pin passed to SetGPIOOutput %d", (int32_t)gpio);
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (gpio >= MAX9295_GPIO_TYPE_NUM) {
        LOG_ERROR("MAX9295: GPIO pin passed to SetGPIOOutput exceeds maximum %d", (int32_t)gpio);
        return WICRI_STATUS_BAD_PARAMETER;
    }

    setGPIOAModeRegs[0].address += ((uint8_t) gpio - 1u) * 3u;
    setGPIOBModeRegs[0].address += ((uint8_t) gpio - 1u) * 3u;

    if (level) {
        setGPIOAModeRegs[0].data |= (1 << 4);
    }

    status = WriteArray(priv->i2cProgrammer, &setGPIOAMode);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set GPIO A mode write failed %d", (int32_t)status);
        goto done;
    }

    status = WriteArray(priv->i2cProgrammer, &setGPIOBMode);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set GPIO B mode write failed %d", (int32_t)status);
        goto done;
    }

done:
    return status;
}

static NvMediaStatus
SetFsyncGPIO(
    max9295 *priv,
    GPIOTypeMAX9295 gpio,
    uint8_t rxID)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg setGPIOModeRegs[] = {{0x02BE, 0x84}}; /* pull-up 1M ohm, GPIO source en for GMSL2 */
    DevBlkCDII2CRegList setGPIOMode = {
        .regs = setGPIOModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setGPIOModeRegs),
    };

    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to SetFsyncGPIO");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (gpio == MAX9295_GPIO_TYPE_INVALID) {
        LOG_ERROR("MAX9295: Invalid GPIO pin passed to SetFsyncGPIO %d", (int32_t)gpio);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (gpio >= MAX9295_GPIO_TYPE_NUM) {
        LOG_ERROR("MAX9295: GPIO pin passed to SetFsyncGPIO exceeds maximum %d", (int32_t)gpio);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    setGPIOModeRegs[0].address += ((uint16_t) gpio - 1u) * 3u;

    status = WriteArray(priv->i2cProgrammer, &setGPIOMode);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set FSync write failed %d", (int32_t)status);
        goto done;
    }

    /* Update the offset from GPIO A to GPIO C*/
    setGPIOModeRegs[0].address += 2u;
    status = ReadUint8(priv->i2cProgrammer, setGPIOModeRegs[0].address, (uint8_t *) &(setGPIOModeRegs[0].data));
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set FSync read failed %d", (int32_t)status);
        goto done;
    }

    setGPIOModeRegs[0].data &= 0xE0;
    setGPIOModeRegs[0].data |= (rxID & 0x1F); /* GPIO receive ID */
    status = WriteArray(priv->i2cProgrammer, &setGPIOMode);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set FSync GPIO write failed %d", (int32_t)status);
        goto done;
    }

done:
    return status;
}

static NvMediaStatus
Setnvp2650(
    max9295 *priv
    )
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg setNvp2650ModeRegs[] = {
        // {0x0010,0x10},
        // {0x03f1,0x89},
        // {0x0330,0x00},
        // {0x0331,0x30},
        // {0x0332,0xe0},
        // {0x0333,0x04},
        // {0x0316,0x1e},
        // {0x0317,0x1e},
        // {0x0311,0x20},
        // {0x0308,0x62},
        // {0x0002,0x23},
        {0x02be,0x10},
        {0x02bf,0x60},
        {0x03f1,0x89},
        {0x0330,0x00},
        {0x0331,0x30},
        {0x0332,0xe0},
        {0x0333,0x04},
        {0x0316,0x1e},
        {0x0311,0x20},
        {0x0308,0x62},
        {0x0002,0x23},
    }; 
    DevBlkCDII2CRegList setNvp2650ModeReg = {
        .regs = setNvp2650ModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setNvp2650ModeRegs),
    };

    printk("alex:reset lin...\r\n");
    status = WriteUint8(priv->i2cProgrammer, 0x10, 0x31);
    if (status != WICRI_STATUS_OK)
        LOG_ERROR("write reglist Failed!");
    usleep_range(1000000,1000000);

    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to SetFsyncGPIO");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }
    status = WriteArray(priv->i2cProgrammer, &setNvp2650ModeReg);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set FSync write failed %d", (int32_t)status);
        goto done;
    }

done:
    return status;
}

static NvMediaStatus
EnableRefClock(
    max9295 *priv,
    GPIOTypeMAX9295 gpio,
    bool enableRClk)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg enablePCLKOutRegs[] = {{0x3F1, 0x00}};
    DevBlkCDII2CRegList enablePCLKOut = {
        .regs = enablePCLKOutRegs,
        .numRegs = I2C_ARRAY_SIZE(enablePCLKOutRegs),
    };



    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to EnableRefClock");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (gpio == MAX9295_GPIO_TYPE_INVALID) {
        LOG_ERROR("MAX9295: Invalid GPIO pin passed to EnableRefClock %d", (int32_t)gpio);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (gpio >= MAX9295_GPIO_TYPE_NUM) {
        LOG_ERROR("MAX9295: GPIO pin passed to EnableRefClock exceeds GPIO maximum %d", (int32_t)gpio);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (enableRClk) {
        enablePCLKOutRegs[0].data |= 0x80;
    }

    enablePCLKOutRegs[0].data |= ((((uint8_t) gpio - 1u) << 1) | 0x1);

    status = WriteArray(priv->i2cProgrammer, &enablePCLKOut);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Enable ref clock write failed %d", (int32_t)status);
        goto done;
    }

done:
    return status;
}

static NvMediaStatus
ForwardGPIO(
    max9295 *priv,
    uint8_t srcGpio,
    uint8_t dstGpio)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    

    DevBlkCDII2CReg setGPIOModeRegs[] = {{0x02BE, 0x1C}};
    DevBlkCDII2CRegList setGPIOMode = {
        .regs = setGPIOModeRegs,
        .numRegs = I2C_ARRAY_SIZE(setGPIOModeRegs),
    };



    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to ForwardGPIO");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (srcGpio > 10U) {
        LOG_ERROR("MAX9295: Source GPIO to forward exceeds 10 %d", (int32_t)srcGpio);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    if (dstGpio > 31U) {
        LOG_ERROR("MAX9295: Destination GPIO exceeds 31 %d", (int32_t)dstGpio);
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    setGPIOModeRegs[0].address += (srcGpio * 3u);

    status = ReadUint8(priv->i2cProgrammer, setGPIOModeRegs[0].address, (uint8_t *) &(setGPIOModeRegs[0].data));
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set forward GPIO read failed %d", (int32_t)status);
        goto done;
    }

    setGPIOModeRegs[0].data |= 0x3; /* Set GPIO_TX_EN, GPIO_OUT_DIS */
    setGPIOModeRegs[0].data &= ~(1 << 2); /* Unset GPIO_RX_EN */

    status = WriteArray(priv->i2cProgrammer, &setGPIOMode);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set Forward GPIO write failed %d", (int32_t)status);
        goto done;
    }

    /* Update the offset from GPIO A to GPIO B */
    setGPIOModeRegs[0].address += 1u;
    status = ReadUint8(priv->i2cProgrammer, setGPIOModeRegs[0].address, (uint8_t *) &(setGPIOModeRegs[0].data));
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set Forward GPIIO read failed %d", (int32_t)status);
        goto done;
    }

    setGPIOModeRegs[0].data &= 0xE0;
    setGPIOModeRegs[0].data |= (dstGpio & 0x1F);
    status = WriteArray(priv->i2cProgrammer, &setGPIOMode);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Set Forward GPIO mode write failed %d", (int32_t)status);
        goto done;
    }

done:
    return status;
}

static NvMediaStatus
GetRevId(
    max9295 *priv,
    RevisionMAX9295 *rev)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    uint32_t numRev = sizeof(supportedRevisions) / sizeof(supportedRevisions[0]);
    uint8_t revision = 0u;
    uint32_t i = 0u;


    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to GetRevId");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    status = ReadUint8(priv->i2cProgrammer, REG_DEV_REV_ADDR, &revision);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: GetRevId read failed %d", (int32_t)status);
        goto done;
    }

    revision &= 0x0F;
    for (i = 0u; i < numRev; i++) {
        if (revision == supportedRevisions[i].revVal) {
            *rev = supportedRevisions[i].revId;
            LOG_DBG("MAX9295: Revision %u detected!\n", (int)revision);
            goto done;
        }
    }

    LOG_DBG("MAX9295: Unsupported MAX9295 revision %u detected!\nSupported revisions are:", (int)revision);
    for (i = 0u; i < numRev; i++) {
        LOG_DBG("MAX9295: Revision %u\n", supportedRevisions[i].revVal);
    }
    status = WICRI_STATUS_NOT_SUPPORTED;

done:
    return status;
}

NvMediaStatus
MAX9295SetDefaults(
    max9295 *priv)
{
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to MAX9295SetDefaults");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    return WICRI_STATUS_OK;
}

NvMediaStatus
MAX9295WriteParameters(
    max9295 *priv,
    uint32_t parameterType,
    uint32_t parameterSize,
    void *parameter)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    bool isValidSize = false;
    ReadWriteParamsMAX9295 *param = (ReadWriteParamsMAX9295 *) parameter;
    
    static const char *cmdString[] = {
        [WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_A] =
            "WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_A",
        [WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_B] =
            "WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_B",
        [WRITE_PARAM_CMD_MAX9295_SET_DEVICE_ADDRESS] =
            "WRITE_PARAM_CMD_MAX9295_SET_DEVICE_ADDRESS",
        [WRITE_PARAM_CMD_MAX9295_SET_FSYNC_GPIO] =
            "WRITE_PARAM_CMD_MAX9295_SET_FSYNC_GPIO",
        [WRITE_PARAM_CMD_MAX9295_SET_GPIO_OUTPUT] =
            "WRITE_PARAM_CMD_MAX9295_SET_GPIO_OUTPUT",
        [WRITE_PARAM_CMD_MAX9295_ENABLE_NVP2650] = 
            "WRITE_PARAM_CMD_MAX9295_ENABLE_NVP2650",
        [WRITE_PARAM_CMD_MAX9295_ENABLE_REF_CLOCK] =
            "WRITE_PARAM_CMD_MAX9295_ENABLE_REF_CLOCK",
        [WRITE_PARAM_CMD_MAX9295_CONFIG_VIDEO_PIPELINES] =
            "WRITE_PARAM_CMD_MAX9295_CONFIG_VIDEO_PIPELINES",
        [WRITE_PARAM_CMD_MAX9295_CONFIG_PHY] =
            "WRITE_PARAM_CMD_MAX9295_CONFIG_PHY",
        [WRITE_PARAM_CMD_MAX9295_GPIO_FORWARD] =
            "WRITE_PARAM_CMD_MAX9295_GPIO_FORWARD",
    };

    if (parameter == NULL) {
        LOG_ERROR("MAX9295: Null parameter passed to MAX9295WriteParameters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    
    if  (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to MAX9295WriteParameters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if (parameterType == WRITE_PARAM_CMD_MAX9295_INVALID) {
        LOG_ERROR("MAX9295: Invalid parameter type passed to MAX9295WriteParameters parameterType:%d", (uint32_t)parameterType);
        return WICRI_STATUS_BAD_PARAMETER;
    }

    if  (parameterType >= WRITE_PARAM_CMD_MAX9295_NUM) {
        LOG_ERROR("MAX9295: Out of range parameter type passed to MAX9295WriteParameters parameterType:%d", (uint32_t)parameterType);
        return WICRI_STATUS_BAD_PARAMETER;
    }

    LOG_DBG("MAX9295: Executing command %s", cmdString[parameterType]);
    switch (parameterType) {
        case WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_A:
        case WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_B:
            if (parameterSize == sizeof(param->Translator)) {
                isValidSize = true;
                status = SetTranslator(priv,
                                       parameterType,
                                       param->Translator.source,
                                       param->Translator.destination);
            }
            break;
        case WRITE_PARAM_CMD_MAX9295_SET_DEVICE_ADDRESS:
            if (parameterSize == sizeof(param->DeviceAddress)) {
                isValidSize = true;
                status = SetDeviceAddress(priv,
                                          param->DeviceAddress.address);
            };
            break;
        case WRITE_PARAM_CMD_MAX9295_SET_GPIO_OUTPUT:
            if (parameterSize == sizeof(param->GPIOOutp)) {
                isValidSize = true;
                status = SetGPIOOutput(priv,
                                       param->GPIOOutp.gpioInd,
                                       param->GPIOOutp.level);
            };
            break;
        case WRITE_PARAM_CMD_MAX9295_SET_FSYNC_GPIO:
            if (parameterSize == sizeof(param->FSyncGPIO)) {
                isValidSize = true;
                status = SetFsyncGPIO(priv,
                                       param->FSyncGPIO.gpioInd,
                                       param->FSyncGPIO.rxID);
            };
            break;
        case WRITE_PARAM_CMD_MAX9295_ENABLE_NVP2650:
            isValidSize = true;
            status = Setnvp2650(priv);
            break;
        case WRITE_PARAM_CMD_MAX9295_ENABLE_REF_CLOCK:
            if (parameterSize == sizeof(param->RefClkGPIO)) {
                isValidSize = true;
                status = EnableRefClock(priv,
                                        param->RefClkGPIO.gpioInd,
                                        param->RefClkGPIO.enableRClk);
            };
            break;
        case WRITE_PARAM_CMD_MAX9295_GPIO_FORWARD:
            if (parameterSize == sizeof(param->GPIOForward)) {
                isValidSize = true;
                status = ForwardGPIO(priv,
                                     param->GPIOForward.srcGpio,
                                     param->GPIOForward.dstGpio);
            };
            break;
        case WRITE_PARAM_CMD_MAX9295_CONFIG_VIDEO_PIPELINES:
            if (parameterSize == sizeof(param->ConfigVideoPipeline)) {
                isValidSize = true;
                status = ConfigPipelines(priv,
                                         param->ConfigVideoPipeline.dataType,
                                         param->ConfigVideoPipeline.embDataType);
            };
            break;
        case WRITE_PARAM_CMD_MAX9295_CONFIG_PHY:
            if (parameterSize == sizeof(param->ConfigPhy)) {
                isValidSize = true;
                status = ConfigPhy(priv,
                                   &param->ConfigPhy.mapping,
                                   &param->ConfigPhy.polarity,
                                   param->ConfigPhy.numDataLanes);
            };
            break;
        default:
            LOG_ERROR("MAX9295: Invalid command %d", (int32_t)parameterType);
            isValidSize = true;
            status = WICRI_STATUS_BAD_PARAMETER;
            break;
    }

    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Command failed %s", cmdString[parameterType]);
    }

    if (!isValidSize) {
        LOG_ERROR("MAX9295: Invalid parameter size");
        status = WICRI_STATUS_BAD_PARAMETER;
    }

    return status;
}
EXPORT_SYMBOL(MAX9295WriteParameters);


NvMediaStatus
MAX9295DumpRegisters(
    max9295 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    uint32_t i = 0u;
    
    DevBlkCDII2CReg dumpRegs[] = {{0x0000, 0x00}};

    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to MAX9295DumpRegisters");
        return WICRI_STATUS_BAD_PARAMETER;
    }

    for (i = 0u; i <= MAX9295_REG_MAX_ADDRESS; i++) {
        dumpRegs[0].address = (uint16_t) i;
        status = ReadUint8(priv->i2cProgrammer, dumpRegs[0].address, (uint8_t *) &(dumpRegs[0].data));
        if (status != WICRI_STATUS_OK) {
            LOG_ERROR("MAX9295: Failed to dump register i:%d status:%d", (int32_t)i, (int32_t)status);
            return status;
        }

        LOG_ERROR("MAX9295: Regsiter has value dumpRegs[0].address:%d dumpRegs[0].data:%d", (int32_t)dumpRegs[0].address, (int32_t)dumpRegs[0].data);
    }

    return status;
}
NvMediaStatus
MAX9295CheckPresence(
    max9295 *priv)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    RevisionMAX9295 rev = MAX9295_INVALID_REV;
    uint8_t devID = 0u;
    

    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to MAX9295CheckPresence");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    status = ReadUint8(priv->i2cProgrammer, REG_DEV_ID_ADDR, &devID);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Read device ID failed %d", (int32_t)status);
        goto done;
    };

    if ((devID != MAX9295A_DEV_ID) && devID != MAX9295B_DEV_ID) {
        LOG_ERROR("MAX9295: Device ID mismatch (expected, expected) MAX9295A_DEV_ID:%02x MAX9295B_DEV_ID:%02x", MAX9295A_DEV_ID, MAX9295B_DEV_ID);
        LOG_ERROR("MAX9295: Device ID mismatch (returned) %d", (int32_t)devID);
        status = WICRI_STATUS_NOT_SUPPORTED;
        goto done;
    }

    status = GetRevId(priv,
                      &rev);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: GetRevId failed %d", (int32_t)status);
        goto done;
    }

done:
    return status;
}
EXPORT_SYMBOL(MAX9295CheckPresence);

NvMediaStatus
MAX9295ReadErrorStatus(
    max9295 *priv,
    uint32_t dataLength,
    uint8_t *dataBuff)
{
    NvMediaStatus status = WICRI_STATUS_OK;
    
    
    if (priv == NULL) {
        LOG_ERROR("MAX9295: Null driver priv passed to MAX9295ReadErrorStatus");
        status = WICRI_STATUS_BAD_PARAMETER;
        goto done;
    }

    status = ReadUint8(priv->i2cProgrammer, REG_LFLT_INT, dataBuff);
    if (status != WICRI_STATUS_OK) {
        LOG_ERROR("MAX9295: Read error status failed %d", (int32_t)status);
        goto done;
    };

done:
    return status;
}

static  struct regmap_config max9295_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int max9295_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	max9295 *priv;
	int err = 0;

	dev_info(&client->dev, "[MAX9295]: probing GMSL Serializer\n");

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;
	priv->i2cProgrammer = devm_regmap_init_i2c(priv->i2c_client,
				&max9295_regmap_config);
	if (IS_ERR(priv->i2cProgrammer)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(priv->i2cProgrammer));
		return -ENODEV;
	}

	mutex_init(&priv->lock);

	dev_set_drvdata(&client->dev, priv);

	/* dev communication gets validated when GMSL link setup is done */
	dev_info(&client->dev, "%s:  success\n", __func__);

	return err;
}

static int max9295_remove(struct i2c_client *client)
{
	max9295 *priv;
	if (client != NULL) {
		priv = dev_get_drvdata(&client->dev);
		mutex_destroy(&priv->lock);
		i2c_unregister_device(client);
		client = NULL;
	}

	return 0;
}

static const struct i2c_device_id max9295_id[] = {
	{ "max9295", 0 },
	{ },
};

const struct of_device_id max9295_of_match[] = {
	{ .compatible = "nvidia,max9295", },
	{ },
};
MODULE_DEVICE_TABLE(of, max9295_of_match);
MODULE_DEVICE_TABLE(i2c, max9295_id);

static struct i2c_driver max9295_i2c_driver = {
	.driver = {
		.name = "max9295",
		.owner = THIS_MODULE,
	},
	.probe = max9295_probe,
	.remove = max9295_remove,
	.id_table = max9295_id,
};

static int __init max9295_init(void)
{
	return i2c_add_driver(&max9295_i2c_driver);
}

static void __exit max9295_exit(void)
{
	i2c_del_driver(&max9295_i2c_driver);
}

module_init(max9295_init);
module_exit(max9295_exit);

MODULE_DESCRIPTION("GMSL Serializer driver max9295");
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com>");
MODULE_LICENSE("GPL v2");