/*
 * water_level.c
 *
 *  Created on: Jun 12, 2019
 *      Author: Zoehl's Rig
 */

//#define water_temp 22
#define density 1000
#define gravity 9.8
#define offset_distance 3
<<<<<<< HEAD
#define atm_prs 100740
=======
#define atm_prs 100338
>>>>>>> 0c0a8a90b8ec990f9bc6cd970d62973dd75b1963

#include <stdio.h>
#include <math.h>
#include "string.h"
#include "dps310_hal.h"
#include "dps310.h"
#include "global.h"

double water_level (void);
void get_ref_pressure (void);

double water_level ()
{
	int ret;
	double temp, air_prs, water_level = 0;
	ret = dps310_hal_get_temp_press(&temp, &air_prs);
<<<<<<< HEAD
	water_level = (((air_prs - ref_pressure)/ (density * gravity)) * 100);
	if (air_prs >= ref_pressure)
		water_level = water_level + offset_distance;
=======
	water_level = (((air_prs - atm_prs)/ (density * gravity)) * 100) + offset_distance;
//	if (air_prs >= ref_pressure)
//		water_level = water_level + offset_distance;
>>>>>>> 0c0a8a90b8ec990f9bc6cd970d62973dd75b1963
	return water_level;
}

void get_ref_pressure (void)
{
	double temp;
	int ret;
	ret = dps310_hal_get_temp_press(&temp, &ref_pressure);
}
