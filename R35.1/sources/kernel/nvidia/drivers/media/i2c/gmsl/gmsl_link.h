#ifndef __GMSL_LINK_H
#define __GMSL_LINK_H

typedef enum {
    GMSL_MODE_INVALID,
    GMSL_MODE_UNUSED,
    GMSL1_MODE,
    GMSL2_MODE_6GBPS,
    GMSL2_MODE_3GBPS,
    GMSL_MODE_MAX,
} GMSLMode;

typedef enum {
    LINK_LOCK_GMSL1,
    LINK_LOCK_GMSL2,
    LINK_LOCK_VIDEO,
    LINK_LOCK_MAX,
} LinkLockType;

typedef enum {
    LINK_UNLOCKED,
    LINK_LOCKED,
    LINK_LOCK_INVALID,
} LinkLockStatus;

#endif // __GMSL_LINK_H