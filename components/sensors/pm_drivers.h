#pragma once

/*
 * The concrete drivers pm.c binds to. Private to the sensors component.
 */

#include "pm.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const pm_driver_t sps30_pm_driver;
extern const pm_driver_t pms5003_pm_driver;

#ifdef __cplusplus
}
#endif
