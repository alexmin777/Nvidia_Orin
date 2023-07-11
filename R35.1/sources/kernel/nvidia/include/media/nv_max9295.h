#ifndef _NV_MAX9295_H_
#define _NV_MAX9295_H_


typedef struct {
    struct i2c_client *i2c_client;
    struct regmap *i2cProgrammer;
    struct mutex lock;
} max9295;



typedef enum {
    /* This type must be contiguous and start from 0 */
    WRITE_PARAM_CMD_MAX9295_INVALID = 0u,
    WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_A,
    WRITE_PARAM_CMD_MAX9295_SET_TRANSLATOR_B,
    WRITE_PARAM_CMD_MAX9295_SET_DEVICE_ADDRESS,
    WRITE_PARAM_CMD_MAX9295_SET_GPIO_OUTPUT,
    WRITE_PARAM_CMD_MAX9295_SET_FSYNC_GPIO,
    WRITE_PARAM_CMD_MAX9295_ENABLE_NVP2650,
    WRITE_PARAM_CMD_MAX9295_ENABLE_REF_CLOCK,
    WRITE_PARAM_CMD_MAX9295_CONFIG_VIDEO_PIPELINES,
    WRITE_PARAM_CMD_MAX9295_CONFIG_PHY,
    WRITE_PARAM_CMD_MAX9295_GPIO_FORWARD,
    WRITE_PARAM_CMD_MAX9295_NUM,
} WriteParametersCmdMAX9295;

typedef enum {
    MAX9295_DATA_TYPE_INVALID = 0u,
    MAX9295_DATA_TYPE_RAW10,
    MAX9295_DATA_TYPE_RAW12,
    MAX9295_DATA_TYPE_RAW16,
    MAX9295_DATA_TYPE_YUV_8,
} DataTypeMAX9295;

typedef enum {
    MAX9295_GPIO_TYPE_INVALID = 0u,
    MAX9295_GPIO_TYPE_MFP0,
    MAX9295_GPIO_TYPE_MFP1,
    MAX9295_GPIO_TYPE_MFP2,
    MAX9295_GPIO_TYPE_MFP3,
    MAX9295_GPIO_TYPE_MFP4,
    MAX9295_GPIO_TYPE_MFP5,
    MAX9295_GPIO_TYPE_MFP6,
    MAX9295_GPIO_TYPE_MFP7,
    MAX9295_GPIO_TYPE_MFP8,
    MAX9295_GPIO_TYPE_NUM,
} GPIOTypeMAX9295;

typedef enum {
    MAX9295_INVALID_REV = 0u,
    MAX9295_REV_5,
    MAX9295_REV_7,
    MAX9295_REV_8,
} RevisionMAX9295;

typedef struct {
    uint8_t phy0_d0;
    uint8_t phy0_d1;
    uint8_t phy1_d0;              /* data lane0 connected between sensor and serializer */
    uint8_t phy1_d1;              /* data lane1 connected between sensor and serializer */
    uint8_t phy2_d0;              /* data lane2 connected between sensor and serializer */
    uint8_t phy2_d1;              /* data lane3 connected between sensor and serializer */
    uint8_t phy3_d0;
    uint8_t phy3_d1;
    bool enableMapping;
} phyMapMAX9295;

typedef struct {
    uint8_t phy1_d0;               /* lane0pol */
    uint8_t phy1_d1;               /* lane1pol */
    uint8_t phy1_clk;              /* clk1pol */
    uint8_t phy2_d0;               /* lane2pol */
    uint8_t phy2_d1;               /* lane3pol */
    uint8_t phy2_clk;              /* clk2pol */
    bool setPolarity;
} phyPolarityMAX9295;


typedef union {
    struct {
        uint8_t source;             /* 7 bit I2C address */
        uint8_t destination;        /* 7 bit I2C address */
    } Translator;

    struct {
        uint8_t address;            /* 7 bit I2C address */
    } DeviceAddress;

    struct {
        GPIOTypeMAX9295 gpioInd;    /* Must be 0-8 for MFP0-MFP8 pins */
        bool level;                 /* level = true to set logic high */
    } GPIOOutp;

    struct {
        GPIOTypeMAX9295 gpioInd;    /* Must be 0-8 for MFP0-MFP8 pins */
        uint8_t rxID;               /* GPIO Rx ID. Must match with deserialiser val */
    } FSyncGPIO;

    struct {
        GPIOTypeMAX9295 gpioInd;    /* Must be 0-8 for MFP0-MFP8 pins */
        bool enableRClk;            /* Enable RCLK output on PCLKOUT pin */
    } RefClkGPIO;

    struct {
        uint8_t srcGpio;            /* Serializer GPIO number as the input */
        uint8_t dstGpio;            /* Destination GPIO number as the output */
    } GPIOForward;

    struct {
        DataTypeMAX9295 dataType;   /* Sensor data type for pixel data */
        bool embDataType;           /* Set to true if emb data has emb data type */
    } ConfigVideoPipeline;

    struct {
        phyMapMAX9295 mapping;
        phyPolarityMAX9295 polarity;
        uint8_t numDataLanes;
    } ConfigPhy;
} ReadWriteParamsMAX9295;

NvMediaStatus
MAX9295CheckPresence(
    max9295 *priv);

NvMediaStatus
MAX9295SetDefaults(
    max9295 *priv);

NvMediaStatus
MAX9295WriteParameters(
    max9295 *priv,
    uint32_t parameterType,
    uint32_t parameterSize,
    void *parameter);

NvMediaStatus
MAX9295ReadErrorStatus(
    max9295 *priv,
    uint32_t dataLength,
    uint8_t *dataBuff);

NvMediaStatus
MAX9295DumpRegisters(
    max9295 *priv);
#endif /* _NV_MAX9295_H_ */
