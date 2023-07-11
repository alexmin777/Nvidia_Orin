#ifndef GMSL_COMMON_H
#define GMSL_COMMON_H

/**
 * @brief The log switch is controlled globally
 * 
 */

/** Quick-log a message at debugging level*/
#define WRICRI_DBG

/** Quick-log a message at info level*/
#define WRICRI_INFO

/** Quick-log a message at error level*/
#define WRICRI_ERROR

/** Quick-log a message at warning level*/
#define WRICRI_WARN


#ifdef WRICRI_DBG
#define LOG_DBG(fmt, ...) \
	printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)

#else
#define LOG_DBG(format, args...)
#endif

#ifdef WRICRI_INFO
#define LOG_INFO(fmt, ...) \
	printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#else
#define LOG_INFO(format, args...)
#endif

#ifdef WRICRI_ERROR
#define LOG_ERROR(fmt, ...) \
	printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else
#define LOG_INFO(format, args...)
#endif

#ifdef WRICRI_WARN
#define LOG_WARN(fmt, ...) \
	printk(KERN_WARNING pr_fmt(fmt), ##__VA_ARGS__)
#else
#define LOG_WARN(format, args...)
#endif

#define I2C_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


typedef struct {
    /**
     * @brief i2c regist addr.
     * 
     */
	u16 address;
    /**
     * @brief i2c regist value.
     * 
     */
	u8 data;
    /**
     * @brief delay time us.
     * 
     */
    u16 delayUsec;
} DevBlkCDII2CReg;

typedef struct {
    /**
     * An array of DevBlkCDII2CReg structures, of length `numRegs`.
     *
     * The array should be declared with "const" so the values in the
     * array cannot be modified.
     */
    const DevBlkCDII2CReg *regs;
    /**
     * The number of registers in the `regs` array; Valid range: [0, UINT32_MAX].
     */
    uint32_t numRegs;
} DevBlkCDII2CRegList;

typedef enum {
    /** Specifies that the operation completed successfully
     (with no error). */
    WICRI_STATUS_OK = 0,
    /** Specifies that a bad parameter was passed. */
    WICRI_STATUS_BAD_PARAMETER = 1,
    /** Specifies that the operation has not finished yet. */
    WICRI_STATUS_PENDING = 2,
    /** Specifies that the operation timed out. */
    WICRI_STATUS_TIMED_OUT = 3,
    /** Specifies that the process is out of memory. */
    WICRI_STATUS_OUT_OF_MEMORY = 4,
    /** Specifies that a component requred by the function call is not
     initialized. */
    WICRI_STATUS_NOT_INITIALIZED = 5,
    /** Specifies that the requested operation is not supported. */
    WICRI_STATUS_NOT_SUPPORTED = 6,
    /** Specifies a catch-all error, used when no other error code applies. */
    WICRI_STATUS_ERROR = 7,
    /** Specifies that no operation is pending. */
    WICRI_STATUS_NONE_PENDING = 8,
    /** Specifies insufficient buffering. */
    WICRI_STATUS_INSUFFICIENT_BUFFERING = 9,
    /** Specifies that the size of an object passed to a function was
     invalid. */
    WICRI_STATUS_INVALID_SIZE = 10,
    /** Specifies that a library's version is incompatible with the
     application. */
    WICRI_STATUS_INCOMPATIBLE_VERSION = 11,
    /** Specifies that the operation entered an undefined state. */
    WICRI_STATUS_UNDEFINED_STATE = 13,
    /** Specifies an error from Permanent Fault Software Diagnostic. */
    WICRI_STATUS_PFSD_ERROR = 14,
} NvMediaStatus;
#endif