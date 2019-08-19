#define density 1000
#define gravity 9.8
#define offset_distance 3
#define atm_prs 100740

#include <stdio.h>
#include <math.h>
#include "string.h"
#include "dps310_hal.h"
#include "dps310.h"
#include "global.h"

double water_level (void);
void get_ref_pressure (void);

/* Function to calculate water level from pressure
 * Principle used is Boyle's Law, cancel volume from both sides
 * Pressure 1 is value fetched from pressure sensor
 * Pressure 2 = density * gravity * height */
double water_level ()
{
	int ret;
	double temp, air_prs, water_level = 0;
	ret = dps310_hal_get_temp_press(&temp, &air_prs);
	water_level = (((air_prs - ref_pressure)/ (density * gravity)) * 100);
	if (air_prs >= ref_pressure)
		water_level = water_level + offset_distance;
	return water_level;
}

// ref_pressure from global.h
void get_ref_pressure (void)
{
	double temp;
	int ret;
	ret = dps310_hal_get_temp_press(&temp, &ref_pressure);
}
