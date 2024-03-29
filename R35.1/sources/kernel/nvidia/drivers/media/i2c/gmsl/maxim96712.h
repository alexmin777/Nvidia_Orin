#ifndef __MAXIM96712_H__
#define __MAXIM96712_H__

#include <media/gmsl-common.h>
#include "gmsl_link.h"
#include "deserializer.h"

#define MAX96712_MAX_NUM_LINK                   4
#define MAX96712_NUM_VIDEO_PIPELINES            8
#define MAX96712_MAX_NUM_PHY                    4

/* data defines */
#define MAX96712_CSI_MODE_4X2            0x1
#define MAX96712_CSI_MODE_2X4            0x4
#define MAX96712_CSI_MODE_1X4A_22        0x8
#define MAX96712_CSI_MODE_1X4B_22        0x10

#define MAX96712_DEV_ID                 0xA0u

#define MAX96712_REG_MAX_FIELDS_PER_REG               8u

/***********************define for register***************************************************/
typedef struct {
    uint16_t regAddr;
    uint8_t msbPos;
    uint8_t lsbPos;
    uint32_t delayNS;
} RegBitFieldProp;

typedef enum {
    /* Used for array indexes. Must start from 0
     * Do not change the order.
     * New fields must be added to the bottom of the list */
    REG_FIELD_GMSL1_LOCK_A = 0u,
    REG_FIELD_GMSL1_LOCK_B,
    REG_FIELD_GMSL1_LOCK_C,
    REG_FIELD_GMSL1_LOCK_D,

    REG_FIELD_GMSL1_DET_ERR_A,
    REG_FIELD_GMSL1_DET_ERR_B,
    REG_FIELD_GMSL1_DET_ERR_C,
    REG_FIELD_GMSL1_DET_ERR_D,

    REG_FIELD_GMSL1_VIDEO_LOCK_A,
    REG_FIELD_GMSL1_VIDEO_LOCK_B,
    REG_FIELD_GMSL1_VIDEO_LOCK_C,
    REG_FIELD_GMSL1_VIDEO_LOCK_D,

    REG_FIELD_GMSL1_CONFIG_LOCK_A,
    REG_FIELD_GMSL1_CONFIG_LOCK_B,
    REG_FIELD_GMSL1_CONFIG_LOCK_C,
    REG_FIELD_GMSL1_CONFIG_LOCK_D,

    REG_FIELD_GMSL2_LOCK_A,
    REG_FIELD_GMSL2_LOCK_B,
    REG_FIELD_GMSL2_LOCK_C,
    REG_FIELD_GMSL2_LOCK_D,

    REG_FIELD_GMSL2_DEC_ERR_A,
    REG_FIELD_GMSL2_DEC_ERR_B,
    REG_FIELD_GMSL2_DEC_ERR_C,
    REG_FIELD_GMSL2_DEC_ERR_D,

    REG_FIELD_GMSL2_IDLE_ERR_A,
    REG_FIELD_GMSL2_IDLE_ERR_B,
    REG_FIELD_GMSL2_IDLE_ERR_C,
    REG_FIELD_GMSL2_IDLE_ERR_D,

    REG_FIELD_VIDEO_LOCK_PIPE_0,
    REG_FIELD_VIDEO_LOCK_PIPE_1,
    REG_FIELD_VIDEO_LOCK_PIPE_2,
    REG_FIELD_VIDEO_LOCK_PIPE_3,

    REG_FIELD_DIS_REM_CC_A,
    REG_FIELD_DIS_REM_CC_B,
    REG_FIELD_DIS_REM_CC_C,
    REG_FIELD_DIS_REM_CC_D,

    REG_FIELD_SEC_XOVER_SEL_PHY_A,
    REG_FIELD_SEC_XOVER_SEL_PHY_B,
    REG_FIELD_SEC_XOVER_SEL_PHY_C,
    REG_FIELD_SEC_XOVER_SEL_PHY_D,

    REG_FIELD_LINK_EN_A,
    REG_FIELD_LINK_EN_B,
    REG_FIELD_LINK_EN_C,
    REG_FIELD_LINK_EN_D,

    REG_FIELD_LINK_GMSL2_A,
    REG_FIELD_LINK_GMSL2_B,
    REG_FIELD_LINK_GMSL2_C,
    REG_FIELD_LINK_GMSL2_D,

    REG_FIELD_RX_RATE_PHY_A,
    REG_FIELD_RX_RATE_PHY_B,
    REG_FIELD_RX_RATE_PHY_C,
    REG_FIELD_RX_RATE_PHY_D,

    REG_FIELD_SOFT_BPP_0,
    REG_FIELD_SOFT_BPP_1,
    REG_FIELD_SOFT_BPP_2_L,
    REG_FIELD_SOFT_BPP_3,
    REG_FIELD_SOFT_BPP_2_H,
    REG_FIELD_SOFT_BPP_4,
    REG_FIELD_SOFT_BPP_5,
    REG_FIELD_SOFT_BPP_6_H,
    REG_FIELD_SOFT_BPP_6_L,
    REG_FIELD_SOFT_BPP_7,

    REG_FIELD_SOFT_DT_0,
    REG_FIELD_SOFT_DT_1_L,
    REG_FIELD_SOFT_DT_2_L,
    REG_FIELD_SOFT_DT_3,
    REG_FIELD_SOFT_DT_1_H,
    REG_FIELD_SOFT_DT_2_H,
    REG_FIELD_SOFT_DT_4,
    REG_FIELD_SOFT_DT_5_H,
    REG_FIELD_SOFT_DT_5_L,
    REG_FIELD_SOFT_DT_6_H,
    REG_FIELD_SOFT_DT_6_L,
    REG_FIELD_SOFT_DT_7,

    REG_FIELD_SOFT_OVR_0_EN,
    REG_FIELD_SOFT_OVR_1_EN,
    REG_FIELD_SOFT_OVR_2_EN,
    REG_FIELD_SOFT_OVR_3_EN,
    REG_FIELD_SOFT_OVR_4_EN,
    REG_FIELD_SOFT_OVR_5_EN,
    REG_FIELD_SOFT_OVR_6_EN,
    REG_FIELD_SOFT_OVR_7_EN,

    REG_FIELD_CC_CRC_ERRCNT_A,
    REG_FIELD_CC_CRC_ERRCNT_B,
    REG_FIELD_CC_CRC_ERRCNT_C,
    REG_FIELD_CC_CRC_ERRCNT_D,

    REG_FIELD_I2C_FWDCCEN_PHY_A,
    REG_FIELD_I2C_FWDCCEN_PHY_B,
    REG_FIELD_I2C_FWDCCEN_PHY_C,
    REG_FIELD_I2C_FWDCCEN_PHY_D,

    REG_FIELD_I2C_REVCCEN_PHY_A,
    REG_FIELD_I2C_REVCCEN_PHY_B,
    REG_FIELD_I2C_REVCCEN_PHY_C,
    REG_FIELD_I2C_REVCCEN_PHY_D,

    REG_FIELD_I2C_PORT_GMSL1_PHY_A,
    REG_FIELD_I2C_PORT_GMSL1_PHY_B,
    REG_FIELD_I2C_PORT_GMSL1_PHY_C,
    REG_FIELD_I2C_PORT_GMSL1_PHY_D,

    REG_FIELD_DE_EN_PHY_A,
    REG_FIELD_DE_EN_PHY_B,
    REG_FIELD_DE_EN_PHY_C,
    REG_FIELD_DE_EN_PHY_D,

    REG_FIELD_DE_PRBS_TYPE_PHY_A,
    REG_FIELD_DE_PRBS_TYPE_PHY_B,
    REG_FIELD_DE_PRBS_TYPE_PHY_C,
    REG_FIELD_DE_PRBS_TYPE_PHY_D,

    REG_FIELD_PKTCCEN_LINK_A,
    REG_FIELD_PKTCCEN_LINK_B,
    REG_FIELD_PKTCCEN_LINK_C,
    REG_FIELD_PKTCCEN_LINK_D,

    REG_FIELD_ALT_MEM_MAP12_PHY0,
    REG_FIELD_ALT_MEM_MAP12_PHY1,
    REG_FIELD_ALT_MEM_MAP12_PHY2,
    REG_FIELD_ALT_MEM_MAP12_PHY3,
    REG_FIELD_ALT_MEM_MAP8_PHY0,
    REG_FIELD_ALT_MEM_MAP8_PHY1,
    REG_FIELD_ALT_MEM_MAP8_PHY2,
    REG_FIELD_ALT_MEM_MAP8_PHY3,
    REG_FIELD_ALT_MEM_MAP10_PHY0,
    REG_FIELD_ALT_MEM_MAP10_PHY1,
    REG_FIELD_ALT_MEM_MAP10_PHY2,
    REG_FIELD_ALT_MEM_MAP10_PHY3,
    REG_FIELD_ALT2_MEM_MAP8_PHY0,
    REG_FIELD_ALT2_MEM_MAP8_PHY1,
    REG_FIELD_ALT2_MEM_MAP8_PHY2,
    REG_FIELD_ALT2_MEM_MAP8_PHY3,

    REG_FIELD_BPP8DBL_4,
    REG_FIELD_BPP8DBL_5,
    REG_FIELD_BPP8DBL_6,
    REG_FIELD_BPP8DBL_7,

    REG_FIELD_BPP8DBL_MODE_4,
    REG_FIELD_BPP8DBL_MODE_5,
    REG_FIELD_BPP8DBL_MODE_6,
    REG_FIELD_BPP8DBL_MODE_7,

    REG_FIELD_AEQ_PHY_A,
    REG_FIELD_AEQ_PHY_B,
    REG_FIELD_AEQ_PHY_C,
    REG_FIELD_AEQ_PHY_D,

    REG_FIELD_PERIODIC_AEQ_PHY_A,
    REG_FIELD_PERIODIC_AEQ_PHY_B,
    REG_FIELD_PERIODIC_AEQ_PHY_C,
    REG_FIELD_PERIODIC_AEQ_PHY_D,

    REG_FIELD_EOM_PER_THR_PHY_A,
    REG_FIELD_EOM_PER_THR_PHY_B,
    REG_FIELD_EOM_PER_THR_PHY_C,
    REG_FIELD_EOM_PER_THR_PHY_D,

    REG_FIELD_VIDEO_PIPE_SEL_0,
    REG_FIELD_VIDEO_PIPE_SEL_1,
    REG_FIELD_VIDEO_PIPE_SEL_2,
    REG_FIELD_VIDEO_PIPE_SEL_3,
    REG_FIELD_VIDEO_PIPE_SEL_4,
    REG_FIELD_VIDEO_PIPE_SEL_5,
    REG_FIELD_VIDEO_PIPE_SEL_6,
    REG_FIELD_VIDEO_PIPE_SEL_7,

    REG_FIELD_VIDEO_PIPE_EN_0,
    REG_FIELD_VIDEO_PIPE_EN_1,
    REG_FIELD_VIDEO_PIPE_EN_2,
    REG_FIELD_VIDEO_PIPE_EN_3,
    REG_FIELD_VIDEO_PIPE_EN_4,
    REG_FIELD_VIDEO_PIPE_EN_5,
    REG_FIELD_VIDEO_PIPE_EN_6,
    REG_FIELD_VIDEO_PIPE_EN_7,

    REG_FIELD_PATGEN_CLK_SRC_PIPE_0,
    REG_FIELD_PATGEN_CLK_SRC_PIPE_1,
    REG_FIELD_PATGEN_CLK_SRC_PIPE_2,
    REG_FIELD_PATGEN_CLK_SRC_PIPE_3,
    REG_FIELD_PATGEN_CLK_SRC_PIPE_4,
    REG_FIELD_PATGEN_CLK_SRC_PIPE_5,
    REG_FIELD_PATGEN_CLK_SRC_PIPE_6,
    REG_FIELD_PATGEN_CLK_SRC_PIPE_7,

    REG_FIELD_MIPI_OUT_CFG,
    REG_FIELD_MIPI_PHY_EN,
    REG_FIELD_MIPI_PHY_MAP_A,
    REG_FIELD_MIPI_PHY_MAP_B,
    REG_FIELD_T_T3_PREBEGIN,
    REG_FIELD_T_T3_POST_PREP,
    REG_FIELD_DEV_REV,
    REG_FIELD_DEV_ID,
    REG_FIELD_RESET_ONESHOT,
    REG_FIELD_BACKTOP_EN,
    REG_FIELD_FORCE_CSI_OUT_EN,
    REG_FIELD_ERRB,
    REG_FIELD_OVERFLOW_FIRST4,
    REG_FIELD_OVERFLOW_LAST4,

    REG_FIELD_OSN_COEFFICIENT_0,
    REG_FIELD_OSN_COEFFICIENT_1,
    REG_FIELD_OSN_COEFFICIENT_2,
    REG_FIELD_OSN_COEFFICIENT_3,

    REG_FIELD_ENABLE_OSN_0,
    REG_FIELD_ENABLE_OSN_1,
    REG_FIELD_ENABLE_OSN_2,
    REG_FIELD_ENABLE_OSN_3,

    REG_FIELD_OSN_COEFF_MANUAL_SEED_0,
    REG_FIELD_OSN_COEFF_MANUAL_SEED_1,
    REG_FIELD_OSN_COEFF_MANUAL_SEED_2,
    REG_FIELD_OSN_COEFF_MANUAL_SEED_3,

    REG_FIELD_SET_TX_AMP_0,
    REG_FIELD_SET_TX_AMP_1,
    REG_FIELD_SET_TX_AMP_2,
    REG_FIELD_SET_TX_AMP_3,

    REG_FIELD_SLV_TO_P0_A,
    REG_FIELD_SLV_TO_P0_B,
    REG_FIELD_SLV_TO_P0_C,
    REG_FIELD_SLV_TO_P0_D,
    REG_FIELD_SLV_TO_P1_A,
    REG_FIELD_SLV_TO_P1_B,
    REG_FIELD_SLV_TO_P1_C,
    REG_FIELD_SLV_TO_P1_D,
    REG_FIELD_SLV_TO_P2_A,
    REG_FIELD_SLV_TO_P2_B,
    REG_FIELD_SLV_TO_P2_C,
    REG_FIELD_SLV_TO_P2_D,

    REG_FIELD_REM_ERR_FLAG,

    REG_FIELD_CTRL3,

    REG_FIELD_INTR4,

    REG_FIELD_INTR5,

    REG_FIELD_INTR6,

    REG_FIELD_INTR7,

    REG_FIELD_INTR8,

    REG_FIELD_INTR9,

    REG_FIELD_INTR10,

    REG_FIELD_INTR11,

    REG_FIELD_INTR12_MEM_ERR_OEN,

    REG_FIELD_VID_PXL_CRC_ERR_INT,

    REG_FIELD_FSYNC_22,

    REG_FIELD_BACKTOP25,

    REG_FIELD_VIDEO_RX8_PIPE_0,
    REG_FIELD_VIDEO_RX8_PIPE_1,
    REG_FIELD_VIDEO_RX8_PIPE_2,
    REG_FIELD_VIDEO_RX8_PIPE_3,
    REG_FIELD_VIDEO_RX8_PIPE_4,
    REG_FIELD_VIDEO_RX8_PIPE_5,
    REG_FIELD_VIDEO_RX8_PIPE_6,
    REG_FIELD_VIDEO_RX8_PIPE_7,

    REG_FIELD_VIDEO_MASKED_OEN,

    REG_FIELD_VIDEO_MASKED_FLAG,

    REG_FIELD_PWR0,

    REG_FIELD_PWR_STATUS_FLAG,

    REG_FIELD_ENABLE_LOCK,
    REG_FIELD_ENABLE_ERRB,

    REG_FIELD_CRUSSC_MODE_0,
    REG_FIELD_CRUSSC_MODE_1,
    REG_FIELD_CRUSSC_MODE_2,
    REG_FIELD_CRUSSC_MODE_3,

    REG_FIELD_CSI_MIPI_OUT_EN,

    REG_FIELD_PHY0_SPEED,
    REG_FIELD_PHY1_SPEED,
    REG_FIELD_PHY2_SPEED,
    REG_FIELD_PHY3_SPEED,
   
    REG_FIELD_MAX,
} RegBitField;

static const RegBitFieldProp regBitFieldProps[REG_FIELD_MAX] = {
    [REG_FIELD_GMSL1_LOCK_A]        = {.regAddr = 0x0BCB, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_GMSL1_LOCK_B]        = {.regAddr = 0x0CCB, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_GMSL1_LOCK_C]        = {.regAddr = 0x0DCB, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_GMSL1_LOCK_D]        = {.regAddr = 0x0ECB, .msbPos = 0, .lsbPos = 0},

    [REG_FIELD_GMSL1_DET_ERR_A]     = {.regAddr = 0x0B15, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL1_DET_ERR_B]     = {.regAddr = 0x0C15, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL1_DET_ERR_C]     = {.regAddr = 0x0D15, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL1_DET_ERR_D]     = {.regAddr = 0x0E15, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_GMSL1_VIDEO_LOCK_A]  = {.regAddr = 0x0BCB, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_GMSL1_VIDEO_LOCK_B]  = {.regAddr = 0x0CCB, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_GMSL1_VIDEO_LOCK_C]  = {.regAddr = 0x0DCB, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_GMSL1_VIDEO_LOCK_D]  = {.regAddr = 0x0ECB, .msbPos = 6, .lsbPos = 6},

    [REG_FIELD_GMSL1_CONFIG_LOCK_A] = {.regAddr = 0x0BCB, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_GMSL1_CONFIG_LOCK_B] = {.regAddr = 0x0CCB, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_GMSL1_CONFIG_LOCK_C] = {.regAddr = 0x0DCB, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_GMSL1_CONFIG_LOCK_D] = {.regAddr = 0x0ECB, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_GMSL2_LOCK_A]        = {.regAddr = 0x001A, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_GMSL2_LOCK_B]        = {.regAddr = 0x000A, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_GMSL2_LOCK_C]        = {.regAddr = 0x000B, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_GMSL2_LOCK_D]        = {.regAddr = 0x000C, .msbPos = 3, .lsbPos = 3},

    [REG_FIELD_GMSL2_DEC_ERR_A]     = {.regAddr = 0x0035, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL2_DEC_ERR_B]     = {.regAddr = 0x0036, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL2_DEC_ERR_C]     = {.regAddr = 0x0037, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL2_DEC_ERR_D]     = {.regAddr = 0x0038, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_GMSL2_IDLE_ERR_A]         = {.regAddr = 0x0039, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL2_IDLE_ERR_B]         = {.regAddr = 0x003A, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL2_IDLE_ERR_C]         = {.regAddr = 0x003B, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_GMSL2_IDLE_ERR_D]         = {.regAddr = 0x003C, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_VIDEO_LOCK_PIPE_0]        = {.regAddr = 0x01DC, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_VIDEO_LOCK_PIPE_1]        = {.regAddr = 0x01FC, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_VIDEO_LOCK_PIPE_2]        = {.regAddr = 0x021C, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_VIDEO_LOCK_PIPE_3]        = {.regAddr = 0x023C, .msbPos = 0, .lsbPos = 0},

    [REG_FIELD_DIS_REM_CC_A]             = {.regAddr = 0x0003, .msbPos = 1, .lsbPos = 0},
    [REG_FIELD_DIS_REM_CC_B]             = {.regAddr = 0x0003, .msbPos = 3, .lsbPos = 2},
    [REG_FIELD_DIS_REM_CC_C]             = {.regAddr = 0x0003, .msbPos = 5, .lsbPos = 4},
    [REG_FIELD_DIS_REM_CC_D]             = {.regAddr = 0x0003, .msbPos = 7, .lsbPos = 6},

    [REG_FIELD_SEC_XOVER_SEL_PHY_A]      = {.regAddr = 0x0007, .msbPos = 4, .lsbPos = 4},
    [REG_FIELD_SEC_XOVER_SEL_PHY_B]      = {.regAddr = 0x0007, .msbPos = 5, .lsbPos = 5},
    [REG_FIELD_SEC_XOVER_SEL_PHY_C]      = {.regAddr = 0x0007, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_SEC_XOVER_SEL_PHY_D]      = {.regAddr = 0x0007, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_LINK_EN_A]                = {.regAddr = 0x0006, .msbPos = 0, .lsbPos = 0, .delayNS = 5000},
    [REG_FIELD_LINK_EN_B]                = {.regAddr = 0x0006, .msbPos = 1, .lsbPos = 1, .delayNS = 5000},
    [REG_FIELD_LINK_EN_C]                = {.regAddr = 0x0006, .msbPos = 2, .lsbPos = 2, .delayNS = 5000},
    [REG_FIELD_LINK_EN_D]                = {.regAddr = 0x0006, .msbPos = 3, .lsbPos = 3, .delayNS = 5000},

    [REG_FIELD_LINK_GMSL2_A]             = {.regAddr = 0x0006, .msbPos = 4, .lsbPos = 4, .delayNS = 5000},
    [REG_FIELD_LINK_GMSL2_B]             = {.regAddr = 0x0006, .msbPos = 5, .lsbPos = 5, .delayNS = 5000},
    [REG_FIELD_LINK_GMSL2_C]             = {.regAddr = 0x0006, .msbPos = 6, .lsbPos = 6, .delayNS = 5000},
    [REG_FIELD_LINK_GMSL2_D]             = {.regAddr = 0x0006, .msbPos = 7, .lsbPos = 7, .delayNS = 5000},

    [REG_FIELD_RX_RATE_PHY_A]            = {.regAddr = 0x0010, .msbPos = 1, .lsbPos = 0},
    [REG_FIELD_RX_RATE_PHY_B]            = {.regAddr = 0x0010, .msbPos = 5, .lsbPos = 4},
    [REG_FIELD_RX_RATE_PHY_C]            = {.regAddr = 0x0011, .msbPos = 1, .lsbPos = 0},
    [REG_FIELD_RX_RATE_PHY_D]            = {.regAddr = 0x0011, .msbPos = 5, .lsbPos = 4},

    [REG_FIELD_SOFT_BPP_0]               = {.regAddr = 0x040B, .msbPos = 7, .lsbPos = 3},
    [REG_FIELD_SOFT_BPP_1]               = {.regAddr = 0x0411, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_SOFT_BPP_2_L]             = {.regAddr = 0x0412, .msbPos = 1, .lsbPos = 0},
    [REG_FIELD_SOFT_BPP_3]               = {.regAddr = 0x0412, .msbPos = 6, .lsbPos = 2},
    [REG_FIELD_SOFT_BPP_2_H]             = {.regAddr = 0x0411, .msbPos = 7, .lsbPos = 5},
    [REG_FIELD_SOFT_BPP_4]               = {.regAddr = 0x042B, .msbPos = 7, .lsbPos = 3},
    [REG_FIELD_SOFT_BPP_5]               = {.regAddr = 0x0431, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_SOFT_BPP_6_H]             = {.regAddr = 0x0431, .msbPos = 7, .lsbPos = 5},
    [REG_FIELD_SOFT_BPP_6_L]             = {.regAddr = 0x0432, .msbPos = 1, .lsbPos = 0},
    [REG_FIELD_SOFT_BPP_7]               = {.regAddr = 0x0432, .msbPos = 6, .lsbPos = 2},

    [REG_FIELD_SOFT_DT_0]                = {.regAddr = 0x040E, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_SOFT_DT_1_L]              = {.regAddr = 0x040F, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_SOFT_DT_2_L]              = {.regAddr = 0x0410, .msbPos = 1, .lsbPos = 0},
    [REG_FIELD_SOFT_DT_3]                = {.regAddr = 0x0410, .msbPos = 7, .lsbPos = 2},
    [REG_FIELD_SOFT_DT_1_H]              = {.regAddr = 0x040E, .msbPos = 7, .lsbPos = 6},
    [REG_FIELD_SOFT_DT_2_H]              = {.regAddr = 0x040F, .msbPos = 7, .lsbPos = 4},
    [REG_FIELD_SOFT_DT_4]                = {.regAddr = 0x042E, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_SOFT_DT_5_H]              = {.regAddr = 0x042E, .msbPos = 7, .lsbPos = 6},
    [REG_FIELD_SOFT_DT_5_L]              = {.regAddr = 0x042F, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_SOFT_DT_6_H]              = {.regAddr = 0x042F, .msbPos = 7, .lsbPos = 4},
    [REG_FIELD_SOFT_DT_6_L]              = {.regAddr = 0x0430, .msbPos = 1, .lsbPos = 0},
    [REG_FIELD_SOFT_DT_7]                = {.regAddr = 0x0430, .msbPos = 7, .lsbPos = 2},


    [REG_FIELD_SOFT_OVR_0_EN]            = {.regAddr = 0x0415, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_SOFT_OVR_1_EN]            = {.regAddr = 0x0415, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_SOFT_OVR_2_EN]            = {.regAddr = 0x0418, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_SOFT_OVR_3_EN]            = {.regAddr = 0x0418, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_SOFT_OVR_4_EN]            = {.regAddr = 0x041B, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_SOFT_OVR_5_EN]            = {.regAddr = 0x041B, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_SOFT_OVR_6_EN]            = {.regAddr = 0x041D, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_SOFT_OVR_7_EN]            = {.regAddr = 0x041D, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_CC_CRC_ERRCNT_A]          = {.regAddr = 0x0B19, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_CC_CRC_ERRCNT_B]          = {.regAddr = 0x0C19, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_CC_CRC_ERRCNT_C]          = {.regAddr = 0x0D19, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_CC_CRC_ERRCNT_D]          = {.regAddr = 0x0E19, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_I2C_FWDCCEN_PHY_A]        = {.regAddr = 0x0B04, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_I2C_FWDCCEN_PHY_B]        = {.regAddr = 0x0C04, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_I2C_FWDCCEN_PHY_C]        = {.regAddr = 0x0D04, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_I2C_FWDCCEN_PHY_D]        = {.regAddr = 0x0E04, .msbPos = 0, .lsbPos = 0},

    [REG_FIELD_I2C_REVCCEN_PHY_A]        = {.regAddr = 0x0B04, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_I2C_REVCCEN_PHY_B]        = {.regAddr = 0x0C04, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_I2C_REVCCEN_PHY_C]        = {.regAddr = 0x0D04, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_I2C_REVCCEN_PHY_D]        = {.regAddr = 0x0E04, .msbPos = 1, .lsbPos = 1},

    [REG_FIELD_I2C_PORT_GMSL1_PHY_A]     = {.regAddr = 0x0B04, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_I2C_PORT_GMSL1_PHY_B]     = {.regAddr = 0x0C04, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_I2C_PORT_GMSL1_PHY_C]     = {.regAddr = 0x0D04, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_I2C_PORT_GMSL1_PHY_D]     = {.regAddr = 0x0E04, .msbPos = 3, .lsbPos = 3},

    [REG_FIELD_DE_EN_PHY_A]              = {.regAddr = 0x0B0F, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_DE_EN_PHY_B]              = {.regAddr = 0x0C0F, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_DE_EN_PHY_C]              = {.regAddr = 0x0D0F, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_DE_EN_PHY_D]              = {.regAddr = 0x0E0F, .msbPos = 3, .lsbPos = 3},

    [REG_FIELD_DE_PRBS_TYPE_PHY_A]       = {.regAddr = 0x0B0F, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_DE_PRBS_TYPE_PHY_B]       = {.regAddr = 0x0C0F, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_DE_PRBS_TYPE_PHY_C]       = {.regAddr = 0x0D0F, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_DE_PRBS_TYPE_PHY_D]       = {.regAddr = 0x0E0F, .msbPos = 0, .lsbPos = 0},

    [REG_FIELD_PKTCCEN_LINK_A]           = {.regAddr = 0x0B08, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_PKTCCEN_LINK_B]           = {.regAddr = 0x0C08, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_PKTCCEN_LINK_C]           = {.regAddr = 0x0D08, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_PKTCCEN_LINK_D]           = {.regAddr = 0x0E08, .msbPos = 2, .lsbPos = 2},

    [REG_FIELD_ALT_MEM_MAP12_PHY0]       = {.regAddr = 0x0933, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_ALT_MEM_MAP12_PHY1]       = {.regAddr = 0x0973, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_ALT_MEM_MAP12_PHY2]       = {.regAddr = 0x09B3, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_ALT_MEM_MAP12_PHY3]       = {.regAddr = 0x09F3, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_ALT_MEM_MAP8_PHY0]        = {.regAddr = 0x0933, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_ALT_MEM_MAP8_PHY1]        = {.regAddr = 0x0973, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_ALT_MEM_MAP8_PHY2]        = {.regAddr = 0x09B3, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_ALT_MEM_MAP8_PHY3]        = {.regAddr = 0x09F3, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_ALT_MEM_MAP10_PHY0]       = {.regAddr = 0x0933, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_ALT_MEM_MAP10_PHY1]       = {.regAddr = 0x0973, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_ALT_MEM_MAP10_PHY2]       = {.regAddr = 0x09B3, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_ALT_MEM_MAP10_PHY3]       = {.regAddr = 0x09F3, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_ALT2_MEM_MAP8_PHY0]       = {.regAddr = 0x0933, .msbPos = 4, .lsbPos = 4},
    [REG_FIELD_ALT2_MEM_MAP8_PHY1]       = {.regAddr = 0x0973, .msbPos = 4, .lsbPos = 4},
    [REG_FIELD_ALT2_MEM_MAP8_PHY2]       = {.regAddr = 0x09B3, .msbPos = 4, .lsbPos = 4},
    [REG_FIELD_ALT2_MEM_MAP8_PHY3]       = {.regAddr = 0x09F3, .msbPos = 4, .lsbPos = 4},

    [REG_FIELD_BPP8DBL_4]                = {.regAddr = 0x0434, .msbPos = 4, .lsbPos = 4},
    [REG_FIELD_BPP8DBL_5]                = {.regAddr = 0x0434, .msbPos = 5, .lsbPos = 5},
    [REG_FIELD_BPP8DBL_6]                = {.regAddr = 0x0434, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_BPP8DBL_7]                = {.regAddr = 0x0434, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_BPP8DBL_MODE_4]           = {.regAddr = 0x0437, .msbPos = 4, .lsbPos = 4},
    [REG_FIELD_BPP8DBL_MODE_5]           = {.regAddr = 0x0437, .msbPos = 5, .lsbPos = 5},
    [REG_FIELD_BPP8DBL_MODE_6]           = {.regAddr = 0x0437, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_BPP8DBL_MODE_7]           = {.regAddr = 0x0437, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_AEQ_PHY_A]                = {.regAddr = 0x0B14, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_AEQ_PHY_B]                = {.regAddr = 0x0C14, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_AEQ_PHY_C]                = {.regAddr = 0x0D14, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_AEQ_PHY_D]                = {.regAddr = 0x0E14, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_PERIODIC_AEQ_PHY_A]       = {.regAddr = 0x0B14, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_PERIODIC_AEQ_PHY_B]       = {.regAddr = 0x0C14, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_PERIODIC_AEQ_PHY_C]       = {.regAddr = 0x0D14, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_PERIODIC_AEQ_PHY_D]       = {.regAddr = 0x0E14, .msbPos = 6, .lsbPos = 6},

    [REG_FIELD_EOM_PER_THR_PHY_A]        = {.regAddr = 0x0B14, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_EOM_PER_THR_PHY_B]        = {.regAddr = 0x0C14, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_EOM_PER_THR_PHY_C]        = {.regAddr = 0x0D14, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_EOM_PER_THR_PHY_D]        = {.regAddr = 0x0E14, .msbPos = 4, .lsbPos = 0},

    [REG_FIELD_VIDEO_PIPE_SEL_0]         = {.regAddr = 0x00F0, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_VIDEO_PIPE_SEL_1]         = {.regAddr = 0x00F0, .msbPos = 7, .lsbPos = 4},
    [REG_FIELD_VIDEO_PIPE_SEL_2]         = {.regAddr = 0x00F1, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_VIDEO_PIPE_SEL_3]         = {.regAddr = 0x00F1, .msbPos = 7, .lsbPos = 4},
    [REG_FIELD_VIDEO_PIPE_SEL_4]         = {.regAddr = 0x00F2, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_VIDEO_PIPE_SEL_5]         = {.regAddr = 0x00F2, .msbPos = 7, .lsbPos = 4},
    [REG_FIELD_VIDEO_PIPE_SEL_6]         = {.regAddr = 0x00F3, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_VIDEO_PIPE_SEL_7]         = {.regAddr = 0x00F3, .msbPos = 7, .lsbPos = 4},

    [REG_FIELD_VIDEO_PIPE_EN_0]          = {.regAddr = 0x00F4, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_VIDEO_PIPE_EN_1]          = {.regAddr = 0x00F4, .msbPos = 1, .lsbPos = 1},
    [REG_FIELD_VIDEO_PIPE_EN_2]          = {.regAddr = 0x00F4, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_VIDEO_PIPE_EN_3]          = {.regAddr = 0x00F4, .msbPos = 3, .lsbPos = 3},
    [REG_FIELD_VIDEO_PIPE_EN_4]          = {.regAddr = 0x00F4, .msbPos = 4, .lsbPos = 4},
    [REG_FIELD_VIDEO_PIPE_EN_5]          = {.regAddr = 0x00F4, .msbPos = 5, .lsbPos = 5},
    [REG_FIELD_VIDEO_PIPE_EN_6]          = {.regAddr = 0x00F4, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_VIDEO_PIPE_EN_7]          = {.regAddr = 0x00F4, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_PATGEN_CLK_SRC_PIPE_0]    = {.regAddr = 0x01DC, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_PATGEN_CLK_SRC_PIPE_1]    = {.regAddr = 0x01FC, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_PATGEN_CLK_SRC_PIPE_2]    = {.regAddr = 0x021C, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_PATGEN_CLK_SRC_PIPE_3]    = {.regAddr = 0x023C, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_PATGEN_CLK_SRC_PIPE_4]    = {.regAddr = 0x025C, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_PATGEN_CLK_SRC_PIPE_5]    = {.regAddr = 0x027C, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_PATGEN_CLK_SRC_PIPE_6]    = {.regAddr = 0x029C, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_PATGEN_CLK_SRC_PIPE_7]    = {.regAddr = 0x02BC, .msbPos = 7, .lsbPos = 7},

    [REG_FIELD_MIPI_OUT_CFG]             = {.regAddr = 0x08A0, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_MIPI_PHY_EN]              = {.regAddr = 0x08A2, .msbPos = 7, .lsbPos = 4},
    [REG_FIELD_MIPI_PHY_MAP_A]           = {.regAddr = 0x08A3, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_MIPI_PHY_MAP_B]           = {.regAddr = 0x08A4, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_T_T3_PREBEGIN]            = {.regAddr = 0x08AD, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_T_T3_POST_PREP]           = {.regAddr = 0x08AE, .msbPos = 6, .lsbPos = 0},
    [REG_FIELD_DEV_REV]                  = {.regAddr = 0x004C, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_DEV_ID]                   = {.regAddr = 0x000D, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_RESET_ONESHOT]            = {.regAddr = 0x0018, .msbPos = 3, .lsbPos = 0},
    [REG_FIELD_BACKTOP_EN]               = {.regAddr = 0x0400, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_FORCE_CSI_OUT_EN]         = {.regAddr = 0x08A0, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_ERRB]                     = {.regAddr = 0x001A, .msbPos = 2, .lsbPos = 2},
    [REG_FIELD_OVERFLOW_FIRST4]          = {.regAddr = 0x040A, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_OVERFLOW_LAST4]           = {.regAddr = 0x042A, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_OSN_COEFFICIENT_0]        = {.regAddr = 0x142E, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_OSN_COEFFICIENT_1]        = {.regAddr = 0x152E, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_OSN_COEFFICIENT_2]        = {.regAddr = 0x162E, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_OSN_COEFFICIENT_3]        = {.regAddr = 0x172E, .msbPos = 5, .lsbPos = 0},

    [REG_FIELD_ENABLE_OSN_0]             = {.regAddr = 0x1432, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_ENABLE_OSN_1]             = {.regAddr = 0x1532, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_ENABLE_OSN_2]             = {.regAddr = 0x1632, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_ENABLE_OSN_3]             = {.regAddr = 0x1732, .msbPos = 6, .lsbPos = 6},

    [REG_FIELD_OSN_COEFF_MANUAL_SEED_0]  = {.regAddr = 0x1457, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_OSN_COEFF_MANUAL_SEED_1]  = {.regAddr = 0x1557, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_OSN_COEFF_MANUAL_SEED_2]  = {.regAddr = 0x1657, .msbPos = 0, .lsbPos = 0},
    [REG_FIELD_OSN_COEFF_MANUAL_SEED_3]  = {.regAddr = 0x1757, .msbPos = 0, .lsbPos = 0},

    [REG_FIELD_SET_TX_AMP_0]             = {.regAddr = 0x1495, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_SET_TX_AMP_1]             = {.regAddr = 0x1595, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_SET_TX_AMP_2]             = {.regAddr = 0x1695, .msbPos = 5, .lsbPos = 0},
    [REG_FIELD_SET_TX_AMP_3]             = {.regAddr = 0x1795, .msbPos = 5, .lsbPos = 0},

    [REG_FIELD_SLV_TO_P0_A]              = {.regAddr = 0x0640, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P0_B]              = {.regAddr = 0x0650, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P0_C]              = {.regAddr = 0x0660, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P0_D]              = {.regAddr = 0x0670, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P1_A]              = {.regAddr = 0x0680, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P1_B]              = {.regAddr = 0x0690, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P1_C]              = {.regAddr = 0x06A0, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P1_D]              = {.regAddr = 0x06B0, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P2_A]              = {.regAddr = 0x0688, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P2_B]              = {.regAddr = 0x0698, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P2_C]              = {.regAddr = 0x06A8, .msbPos = 2, .lsbPos = 0},
    [REG_FIELD_SLV_TO_P2_D]              = {.regAddr = 0x06B8, .msbPos = 2, .lsbPos = 0},

    [REG_FIELD_REM_ERR_FLAG]             = {.regAddr = 0x002A, .msbPos = 1, .lsbPos = 1},

    [REG_FIELD_CTRL3]                    = {.regAddr = 0x001A, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_INTR4]                    = {.regAddr = 0x0027, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_INTR5]                    = {.regAddr = 0x0028, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_INTR6]                    = {.regAddr = 0x0029, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_INTR7]                    = {.regAddr = 0x002A, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_INTR8]                    = {.regAddr = 0x002B, .msbPos = 3, .lsbPos = 0},

    [REG_FIELD_INTR9]                    = {.regAddr = 0x002C, .msbPos = 3, .lsbPos = 0},

    [REG_FIELD_INTR10]                   = {.regAddr = 0x002D, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_INTR11]                   = {.regAddr = 0x002E, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_INTR12_MEM_ERR_OEN]       = {.regAddr = 0x002F, .msbPos = 6, .lsbPos = 6},

    [REG_FIELD_VID_PXL_CRC_ERR_INT]      = {.regAddr = 0x0045, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_FSYNC_22]                 = {.regAddr = 0x04B6, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_BACKTOP25]                = {.regAddr = 0x0438, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_VIDEO_RX8_PIPE_0]         = {.regAddr = 0x0108, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_VIDEO_RX8_PIPE_1]         = {.regAddr = 0x011A, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_VIDEO_RX8_PIPE_2]         = {.regAddr = 0x012C, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_VIDEO_RX8_PIPE_3]         = {.regAddr = 0x013E, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_VIDEO_RX8_PIPE_4]         = {.regAddr = 0x0150, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_VIDEO_RX8_PIPE_5]         = {.regAddr = 0x0168, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_VIDEO_RX8_PIPE_6]         = {.regAddr = 0x017A, .msbPos = 7, .lsbPos = 0},
    [REG_FIELD_VIDEO_RX8_PIPE_7]         = {.regAddr = 0x018C, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_VIDEO_MASKED_OEN]         = {.regAddr = 0x0049, .msbPos = 5, .lsbPos = 0},

    [REG_FIELD_VIDEO_MASKED_FLAG]        = {.regAddr = 0x004A, .msbPos = 5, .lsbPos = 0},

    [REG_FIELD_PWR0]                     = {.regAddr = 0x0012, .msbPos = 7, .lsbPos = 0},

    [REG_FIELD_PWR_STATUS_FLAG]          = {.regAddr = 0x0047U, .msbPos = 7U, .lsbPos = 0U},

    [REG_FIELD_ENABLE_LOCK]              = {.regAddr = 0x0005, .msbPos = 7, .lsbPos = 7},
    [REG_FIELD_ENABLE_ERRB]              = {.regAddr = 0x0005, .msbPos = 6, .lsbPos = 6},
    [REG_FIELD_CSI_MIPI_OUT_EN]          = {.regAddr = 0x040B, .msbPos = 1, .lsbPos = 1},

    [REG_FIELD_PHY0_SPEED]               = {.regAddr = 0x0415, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_PHY1_SPEED]               = {.regAddr = 0x0418, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_PHY2_SPEED]               = {.regAddr = 0x041B, .msbPos = 4, .lsbPos = 0},
    [REG_FIELD_PHY3_SPEED]               = {.regAddr = 0x041E, .msbPos = 4, .lsbPos = 0},
};
/****************************************************************************************************/

typedef struct {
    RegBitField name[MAX96712_REG_MAX_FIELDS_PER_REG];
    uint8_t val[MAX96712_REG_MAX_FIELDS_PER_REG];
    uint8_t numRegBitFieldArgs;
} RegBitFieldQ;


typedef enum {
    REG_READ_MODE,
    REG_WRITE_MODE,
    REG_READ_MOD_WRITE_MODE,
} RegBitFieldAccessMode;

typedef enum {
    MAX96712_LINK_NONE = 0U,
    MAX96712_LINK_0 = (1U << 0U),
    MAX96712_LINK_1 = (1U << 1U),
    MAX96712_LINK_2 = (1U << 2U),
    MAX96712_LINK_3 = (1U << 3U),
    MAX96712_LINK_ALL = 0xFU,
} LinkStatus;

typedef enum {
    MAX96712_LINK_LOCK_GMSL1,
    MAX96712_LINK_LOCK_GMSL2,
    MAX96712_LINK_LOCK_MAX,
} LinkLockTypeMAX96712;

typedef enum {
    MAX96712_I2CPORT_INVALID,
    MAX96712_I2CPORT_0,
    MAX96712_I2CPORT_1,
    MAX96712_I2CPORT_2,
} I2CPortMAX96712;

typedef enum {
    MIPI_CSI_Controller_0 = 0,
    MIPI_CSI_Controller_1,
    MIPI_CSI_Controller_2,
    MIPI_CSI_Controller_3,
    MIPI_CSI_Controller_MAX,
} MIPICSIController;
/*
 * Utility macro used to call sAccessRegFieldArray() and return if status is not OK.
 * The macro expects the following variables to be available: priv, status
 */
#define ACCESS_REG_FIELD_RET_ERR(mode)      \
{                                           \
    status = sAccessRegFieldQ(priv,         \
                                mode);      \
    if (status != WICRI_STATUS_OK) {        \
        return status;                      \
    }                                       \
}

/*
 * Utility macro used when add one reg field to queue and return if status is not OK.
 * The macro expects the following variables to be available: priv, status
 */
#define ADD_ONE_REG_FIELD_RET_ERR(name, val)    \
{                                               \
    status = AddToRegFieldQ(priv,               \
                            name,               \
                            val);               \
    if (status != WICRI_STATUS_OK) {            \
        return status;                          \
    }                                           \
}

/*
 * Utility macro used when access to only one reg field is needed.
 * This will clear the RegFieldQ, add name, val to the queue, access the register in specified mode
 * and return if status is not OK.
 * The macro expects the following variables to be available: priv, status
 */
#define ACCESS_ONE_REG_FIELD_RET_ERR(name, val, mode)   \
{                                                       \
    ClearRegFieldQ(priv);                               \
    ADD_ONE_REG_FIELD_RET_ERR(name, val);               \
    ACCESS_REG_FIELD_RET_ERR(mode)                      \
}

#define MAX96712_IS_GMSL_LINK_SET(linkVar, linkNum) (((1U << (linkNum)) & (uint8_t) (linkVar)) != 0U)

typedef struct {
    struct i2c_client *i2c_client;
    struct regmap *i2cProgrammer;
    struct mutex lock;
    RegBitFieldQ regBitFieldQ;
    u8 csi_mode;
    u8 linkMask;
    u8 phylines[MAX96712_MAX_NUM_PHY];
    GMSLMode gmslMode[MAX96712_MAX_NUM_LINK];
    PHYMode phyMode;
    I2CPortMAX96712 i2cPort;
    MIPICSIController txPort;
    int reset_gpio;
    bool power_status;
} max96712;


void maxim96712_power_on(struct device *dev);
int maxim96712_dev_info(struct device *dev);
int maxim96712_set_i2c_ctl_port(max96712 *priv);
int maxim96712_get_enabled_links(struct device *dev, int *link_status);
int maxim96712_enable_link(struct device *dev, struct gmsl_link_ctx *g_ctx);
int maxim96712_set_link_mode(struct device *dev, struct gmsl_link_ctx *g_ctx);
int maxim96712_one_shot_reset(struct device *dev, struct gmsl_link_ctx *g_ctx);
int maxim96712_check_link_lock(struct device *dev, struct gmsl_link_ctx *g_ctx);
int maxim96712_video_pipe_selection(struct device *dev, struct gmsl_link_ctx *g_ctx);
int maxim96712_MIPI_phy_set(struct device *dev, u8 speed);
int maxim96712_video_map_to_mipi_ctl(struct device *dev, struct gmsl_link_ctx *g_ctx);

void test_default(struct device *dev);
#endif /* __MAXIM96712_H__ */