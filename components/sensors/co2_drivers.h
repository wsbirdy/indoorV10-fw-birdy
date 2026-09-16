#pragma once

/*
 * The concrete drivers co2.c binds to. Private to the sensors component.
 */

#include "co2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const co2_driver_t senseair_s88_driver;
extern const co2_driver_t senseair_s12_driver;

#ifdef __cplusplus
}
#endif
