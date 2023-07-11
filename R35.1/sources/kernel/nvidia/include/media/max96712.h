#ifndef MAX96712_H
#define MAX96712_H

#define MAX96712_MAX_NUM_LINK                 4U
#define MAX96712_MAX_NUM_PHY                  4U
#define MAX96712_MAX_NUM_PG                   2U
#define MAX96712_NUM_VIDEO_PIPELINES          8U
#define MAX96712_OSC_MHZ                      25U

#define MAX96712_MAX_GLOBAL_ERROR_NUM         20U
#define MAX96712_MAX_PIPELINE_ERROR_NUM       16U
#define MAX96712_MAX_LINK_BASED_ERROR_NUM     16U

// Currently driver implements about 20+16*8+16*4 = 212 error status
#define MAX96712_MAX_ERROR_STATUS_COUNT       255U

#define MAX96712_IS_GMSL_LINK_SET(linkVar, linkNum) (((1U << (linkNum)) & (uint8_t) (linkVar)) != 0U)
#define MAX96712_ADD_LINK(linkVar, linkVal)   ((linkVar) = (LinkMAX96712)((uint8_t) (linkVar) | \
                                                                        (uint8_t) (linkVal)))


typedef enum {
    CONFIG_MAX96712_INVALID,
    CONFIG_MAX96712_START_MIPIOUT,
    CONFIG_MAX96712_STOP_MIPIOUT,
    CONFIG_MAX96712_ENABLE_PG,
    CONFIG_MAX96712_DISABLE_PG,
    CONFIG_MAX96712_MAP_UNUSED_PIPE,
    CONFIG_MAX96712_ENABLE_BACKTOP,
    CONFIG_MAX96712_DISABLE_BACKTOP,
    CONFIG_MAX96712_TRIGGER_DESKEW,
    CONFIG_MAX96712_CHECK_CSIPLL_LOCK,
    CONFIG_MAX96712_ENABLE_REPLICATION,
    CONFIG_MAX96712_DISABLE_REPLICATION,
    CONFIG_MAX96712_ENABLE_ERRB,
    CONFIG_MAX96712_DISABLE_ERRB,
    CONFIG_MAX96712_NUM,
} ConfigSetsMAX96712;

typedef enum {
    READ_PARAM_CMD_MAX96712_INVALID,
    READ_PARAM_CMD_MAX96712_REV_ID,
    READ_PARAM_CMD_MAX96712_ERRB,
    READ_PARAM_CMD_MAX96712_CONTROL_CHANNEL_CRC_ERROR,
    READ_PARAM_CMD_MAX96712_ENABLED_LINKS,
    READ_PARAM_CMD_MAX96712_GET_PWR_METHOD,
    READ_PARAM_CMD_MAX96712_NUM,
} ReadParametersCmdMAX96712;

typedef enum {
    WRITE_PARAM_CMD_MAX96712_INVALID,

    /* GMSL1 related APIs */
    WRITE_PARAM_CMD_MAX96712_SET_DEFAULT_GMSL1_HIM_ENABLED,
    WRITE_PARAM_CMD_MAX96712_ENABLE_FORWARD_CHANNELS,
    WRITE_PARAM_CMD_MAX96712_DISABLE_FORWARD_CHANNELS,
    WRITE_PARAM_CMD_MAX96712_SET_DBL,
    WRITE_PARAM_CMD_MAX96712_DISABLE_DE,
    WRITE_PARAM_CMD_MAX96712_ENABLE_PACKET_BASED_CONTROL_CHANNEL,
    WRITE_PARAM_CMD_MAX96712_DISABLE_PACKET_BASED_CONTROL_CHANNEL,
    WRITE_PARAM_CMD_MAX96712_ENABLE_SPECIFIC_LINKS,
    WRITE_PARAM_CMD_MAX96712_SET_FSYNC,
    WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING,
    WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_YUV,
    WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_TPG,
    WRITE_PARAM_CMD_MAX96712_OVERRIDE_DATATYPE,
    WRITE_PARAM_CMD_MAX96712_SET_MIPI,
    WRITE_PARAM_CMD_MAX96712_ENABLE_DOUBLE_PIXEL_MODE,
    WRITE_PARAM_CMD_MAX96712_SET_TX_SRC_ID,
    WRITE_PARAM_CMD_MAX96712_UNSET_DBL,
    WRITE_PARAM_CMD_MAX96712_ENABLE_PERIODIC_AEQ,
    WRITE_PARAM_CMD_MAX96712_DISABLE_AUTO_ACK,
    WRITE_PARAM_CMD_MAX96712_ENABLE_GPIO_RX,
    WRITE_PARAM_CMD_MAX96712_SET_PG,
    WRITE_PARAM_CMD_MAX96712_NUM,
} WriteParametersCmdMAX96712;

typedef enum {
     MAX96712_GMSL_MODE_INVALID,
     MAX96712_GMSL_MODE_UNUSED,
     MAX96712_GMSL1_MODE,
     MAX96712_GMSL2_MODE_6GBPS,
     MAX96712_GMSL2_MODE_3GBPS,
} GMSLModeMAX96712;

typedef enum {
    MAX96712_I2CPORT_INVALID,
    MAX96712_I2CPORT_0,
    MAX96712_I2CPORT_1,
    MAX96712_I2CPORT_2,
} I2CPortMAX96712;

typedef enum {
    MAX96712_MIPI_OUT_INVALID,
    MAX96712_MIPI_OUT_4x2,
    MAX96712_MIPI_OUT_2x4,
} MipiOutModeMAX96712;

typedef enum {
    MAX96712_PHY_MODE_INVALID,
    MAX96712_PHY_MODE_DPHY,
    MAX96712_PHY_MODE_CPHY,
} PHYModeMAX96712;

typedef enum {
    MAX96712_TXPORT_PHY_C,
    MAX96712_TXPORT_PHY_D,
    MAX96712_TXPORT_PHY_E,
    MAX96712_TXPORT_PHY_F,
} TxPortMAX96712;

typedef enum {
    MAX96712_REV_INVALID,
    MAX96712_REV_1,
    MAX96712_REV_2,
    MAX96712_REV_3,
    MAX96712_REV_4,
    MAX96712_REV_5,
} RevisionMAX96712;

typedef enum {
    MAX96712_LINK_NONE = 0U,
    MAX96712_LINK_0 = (1U << 0U),
    MAX96712_LINK_1 = (1U << 1U),
    MAX96712_LINK_2 = (1U << 2U),
    MAX96712_LINK_3 = (1U << 3U),
    MAX96712_LINK_ALL = 0xFU,
} LinkMAX96712;

typedef enum {
    MAX96712_FSYNC_INVALID,
    MAX96712_FSYNC_MANUAL,
    MAX96712_FSYNC_AUTO,
    MAX96712_FSYNC_OSC_MANUAL,
    MAX96712_FSYNC_EXTERNAL,
} FSyncModeMAX96712;

typedef enum {
    MAX96712_DATA_TYPE_INVALID,
    MAX96712_DATA_TYPE_RAW10,
    MAX96712_DATA_TYPE_RAW12,
    MAX96712_DATA_TYPE_RAW16,
    MAX96712_DATA_TYPE_RGB,
    MAX96712_DATA_TYPE_YUV_8,
} DataTypeMAX96712;

/* Used as param for CheckLink() */
typedef enum {
    MAX96712_LINK_LOCK_INVALID,
    MAX96712_LINK_LOCK_GMSL1_CONFIG,
    MAX96712_LINK_LOCK_GMSL2,
    MAX96712_LINK_LOCK_VIDEO,
} LinkLockTypeMAX96712;

typedef enum {
    MAX96712_PIPELINE_ERROR_INVALID,

    MAX96712_PIPELINE_LMO_OVERFLOW_ERR,
    MAX96712_PIPELINE_CMD_OVERFLOW_ERR,

    MAX96712_PIPELINE_PGEN_VID_UNLOCK_ERR,

    MAX96712_PIPELINE_MEM_ERR,

    MAX96712_PIPELINE_VID_SEQ_ERR,

    MAX96712_MAX_PIPELINE_FAILURE_TYPES
} PipelineFailureTypeMAX96712;

typedef enum {
    MAX96712_GLOBAL_ERROR_INVALID,

    /* GMSL non-link based global errors (cnt:19) */
    MAX96712_GLOBAL_UNLOCK_ERR,
    MAX96712_GLOBAL_ERR,
    MAX96712_GLOBAL_CMU_UNLOCK_ERR,

    MAX96712_GLOBAL_WM,
    MAX96712_GLOBAL_WM2,
    MAX96712_GLOBAL_LINE_FAULT,
    MAX96712_GLOBAL_MEM_STORE_CRC,

    MAX96712_GLOBAL_FRAME_SYNC,
    MAX96712_GLOBAL_REMOTE_SIDE,
    MAX96712_GLOBAL_VID_PRBS,
    MAX96712_GLOBAL_VID_LINE_CRC,

    MAX96712_GLOBAL_MEM_ECC1,
    MAX96712_GLOBAL_MEM_ECC2,

    MAX96712_GLOBAL_FSYNC_SYNC_LOSS,
    MAX96712_GLOBAL_FSYNC_STATUS,

    MAX96712_GLOBAL_CMP_VTERM_STATUS,
    MAX96712_GLOBAL_VDD_OV_FLAG,

    MAX96712_GLOBAL_VDDBAD_STATUS,
    MAX96712_GLOBAL_CMP_STATUS,

    MAX96712_GLOBAL_VDDSW_UV,
    MAX96712_GLOBAL_VDDIO_UV,
    MAX96712_GLOBAL_VDD18_UV,

    MAX96712_MAX_GLOBAL_FAILURE_TYPES
} GlobalFailureTypeMAX96712;

typedef enum {
    MAX96712_GMSL_LINK_ERROR_INVALID,

    /* GMSL2 link based errors (cnt:7) */
    MAX96712_GMSL2_LINK_UNLOCK_ERR,
    MAX96712_GMSL2_LINK_DEC_ERR,
    MAX96712_GMSL2_LINK_IDLE_ERR,
    MAX96712_GMSL2_LINK_EOM_ERR,
    MAX96712_GMSL2_LINK_ARQ_RETRANS_ERR,
    MAX96712_GMSL2_LINK_MAX_RETRANS_ERR,
    MAX96712_GMSL2_LINK_VIDEO_PXL_CRC_ERR,

    /* GMSL1 link based errors (cnt:3) */
    MAX96712_GMSL1_LINK_UNLOCK_ERR,
    MAX96712_GMSL1_LINK_DET_ERR,
    MAX96712_GMSL1_LINK_PKTCC_CRC_ERR,

    MAX96712_MAX_LINK_BASED_FAILURE_TYPES
} LinkFailureTypeMAX96712;

typedef enum {
    MAX96712_GPIO_0  = 0U,  /* MFP0 */
    MAX96712_GPIO_1  = 1U,  /* MFP1 */
    MAX96712_GPIO_2  = 2U,  /* MFP2 */
    MAX96712_GPIO_3  = 3U,  /* MFP3 */
    MAX96712_GPIO_4  = 4U,  /* MFP4 */
    MAX96712_GPIO_5  = 5U,  /* MFP5 */
    MAX96712_GPIO_6  = 6U,  /* MFP6 */
    MAX96712_GPIO_7  = 7U,  /* MFP7 */
    MAX96712_GPIO_8  = 8U,  /* MFP8 */
    MAX96712_GPIO_20 = 20U  /* This GPIO is not existed physically. Used only for the GPIO ID */
} GPIOIndexMAX96712;

typedef enum {
    MAX96712_PG_1920_1236_30FPS,
    MAX96712_PG_1920_1236_60FPS,
    MAX96712_PG_3848_2168_30FPS,
    MAX96712_PG_3848_2174_30FPS,
    MAX96712_PG_2880_1860_30FPS,
    MAX96712_PG_NUM,
} PGModeMAX96712;

typedef enum {
    MAX96712_PG_GEN_0,
    MAX96712_PG_GEN_1,
    MAX96712_PG_GEN_NUM,
} PGGENMAX96712;

typedef enum {
    MAX96712_PG_PCLK_150MHX,
    MAX96712_PG_PCLK_375MHX,
    MAX96712_PG_PCLK_NUM,
} PGPCLKMAX96712;

typedef struct {
    uint8_t vcID;
    /* flag to indicate if emb data lines have emb data type */
    bool isEmbDataType;
    DataTypeMAX96712 dataType;
    /* flag to indicate if data type override is enabled */
    bool isDTOverride;
    /* flag to indicate if pipeline output is mapped to unused CSI controller.
     * Used only for Max96712 TPG modes */
    bool isMapToUnusedCtrl;
    /* flag to indicate if all DTs(Data Type source) come via a single pipeline */
    bool isSinglePipeline;
} LinkPipelineMapMAX96712;

typedef union {
    /* Used with READ_PARAM_CMD_MAX96712_REV_ID */
    RevisionMAX96712 revision;

    /* Used with READ_PARAM_CMD_MAX96712_CONTROL_CHANNEL_CRC_ERROR */
    struct {
        LinkMAX96712 link;
        uint8_t errVal;
    } ErrorStatus;

    /* Used with READ_PARAM_CMD_MAX96712_ENABLED_LINKS */
    LinkMAX96712 link;

    /* Used with READ_PARAM_CMD_MAX96712_GET_PWR_METHOD */
    uint8_t pwrMethod;
} ReadParametersParamMAX96712;

typedef union {
    /* Used with WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING */
    struct {
        LinkPipelineMapMAX96712 linkPipelineMap[MAX96712_MAX_NUM_LINK];
        LinkMAX96712 link;
    } PipelineMapping;

    /* Used with WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING_TPG */
    struct {
        LinkPipelineMapMAX96712 linkPipelineMap[MAX96712_MAX_NUM_LINK];
        uint8_t linkIndex;
    } PipelineMappingTPG;

    /* Used with WRITE_PARAM_CMD_MAX96712_SET_DEFAULT_GMSL1_HIM_ENABLED */
    struct {
        LinkMAX96712 link;
        uint8_t step;
    } GMSL1HIMEnabled;

    /* Used with WRITE_PARAM_CMD_MAX96712_SET_FSYNC */
    struct {
        LinkMAX96712 link;
        FSyncModeMAX96712 FSyncMode;
        uint32_t pclk;
        uint32_t fps;
    } FSyncSettings;

    /* Used with WRITE_PARAM_CMD_MAX96712_SET_MIPI */
    struct {
        uint8_t mipiSpeed;                       /* MIPI speed in multiples of 100MHz */
        PHYModeMAX96712 phyMode;                 /* CPHY or DPHY */
    } MipiSettings;

    /* Used with WRITE_PARAM_CMD_MAX96712_PIPELINE_MAPPING */
    struct {
        DataTypeMAX96712 dataType;
        LinkMAX96712 link;
        bool embDataType;           /* Set to true if emb data has emb data type */
        bool isSharedPipeline;      /* flag to indicate if all DTs come via a single pipeline */
    } DoublePixelMode;

    /* Used with WRITE_PARAM_CMD_MAX96712_SET_PG */
    struct {
        uint32_t width;
        uint32_t height;
        uint32_t frameRate;
        uint8_t linkIndex;
    } SetPGSetting;

    /* Used with
     * WRITE_PARAM_CMD_MAX96712_SET_I2C_PORT
     * WRITE_PARAM_CMD_MAX96712_ENABLE_SPECIFIC_LINK
     * WRITE_PARAM_CMD_MAX96712_DISABLE_FORWARD_CHANNELS
     * WRITE_PARAM_CMD_MAX96712_ENABLE_FORWARD_CHANNELS
     * WRITE_PARAM_CMD_MAX96712_ENABLE_PACKET_BASED_CONTROL_CHANNEL
     * WRITE_PARAM_CMD_MAX96712_DISABLE_PACKET_BASED_CONTROL_CHANNE
     * WRITE_PARAM_CMD_MAX96712_DISABLE_DE
     * WRITE_PARAM_CMD_MAX96712_SET_DBL
     * WRITE_PARAM_CMD_MAX96712_SET_TX_SRC_ID
     */
    LinkMAX96712 link;

    uint8_t gpioIndex;
} WriteParametersParamMAX96712;

/* Parameter type used for GetErrorStatus() */
typedef struct {
    GlobalFailureTypeMAX96712 globalFailureType[MAX96712_MAX_GLOBAL_ERROR_NUM]; /* Outp param */
    uint8_t globalRegVal[MAX96712_MAX_GLOBAL_ERROR_NUM];  /* Outp param */

    uint8_t pipeline;                       /* Inp param. A pipeline whose status needs to be checked */
    PipelineFailureTypeMAX96712 pipelineFailureType[MAX96712_NUM_VIDEO_PIPELINES][MAX96712_MAX_PIPELINE_ERROR_NUM]; /* Outp param */
    uint8_t pipelineRegVal[MAX96712_NUM_VIDEO_PIPELINES][MAX96712_MAX_PIPELINE_ERROR_NUM];  /* Outp param */

    uint8_t link;                           /* Inp param. A single link whose status needs to be checked */
    LinkFailureTypeMAX96712 linkFailureType[MAX96712_MAX_NUM_LINK][MAX96712_MAX_LINK_BASED_ERROR_NUM]; /* Outp param */
    uint8_t linkRegVal[MAX96712_MAX_NUM_LINK][MAX96712_MAX_LINK_BASED_ERROR_NUM];  /* Outp param */

    uint8_t count; /* Outp param, total max96712 error count in current state */
} ErrorStatusMAX96712;

typedef struct {
    /* These must be set in supplied client ctx during driver creation */
    GMSLModeMAX96712 gmslMode[MAX96712_MAX_NUM_LINK]; /* GMSL1 or GMSL2. Unused links must be set to
                                                         MAX96712_GMSL_MODE_UNUSED */
    I2CPortMAX96712 i2cPort;                          /* I2C port 1 or 2 */
    TxPortMAX96712 txPort;                            /* MIPI output port */
    MipiOutModeMAX96712 mipiOutMode;                  /* MIPI configuration */
    uint8_t lanes[MAX96712_MAX_NUM_PHY];              /* The number of lanes */
    PHYModeMAX96712 phyMode;                          /* CPHY or DPHY */
    bool passiveEnabled;                              /* Doesn't need to control sensor/serializer
                                                       * through aggregator */

    /* These will be overwritten during creation */
    RevisionMAX96712 revision;                        /* Chip revision information */
    uint16_t manualFSyncFPS;                          /* Used to store manual fsync frequency */
    uint8_t linkMask;                                 /* Indicate what links to be enabled */

    /* Long cable support */
    bool longCables[MAX96712_MAX_NUM_LINK];

    /* reset all sequence needed at init */
    bool defaultResetAll;

    /* PG setting */
    bool tpgEnabled;                                  /* TPG mode */
    PGModeMAX96712 pgMode[MAX96712_MAX_NUM_PG];       /* PG0/1 modes */
    uint8_t pipelineEnabled;                          /* Pipeline status. 0 - disabled, 1 - enabled */
} ContextMAX96712;

#endif 

