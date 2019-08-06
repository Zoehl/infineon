#include <stdint.h>
#ifndef _DPS310_HAL_H_
#define _DPS310_HAL_H_

int32_t dps310_hal_init(void);

int32_t dps310_hal_get_temp_press(double* temp, double* press);

#endif
