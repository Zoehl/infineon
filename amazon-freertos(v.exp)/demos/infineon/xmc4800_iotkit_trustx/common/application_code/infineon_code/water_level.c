#define density 1000
#define gravity 9.8
#define offset_distance 3
#define atm_prs 100740

#include <stdio.h>
#include <math.h>
#include "string.h"
#include "dps310_hal.h"
#include "dps310.h"
#include "water_level.h"
#include "global.h"
//#include "global.c"

/* Function to calculate water level from pressure
 * Principle used is Boyle's Law, cancel volume from both sides
 * Pressure 1 is value fetched from pressure sensor
 * Pressure 2 = density * gravity * height */
double water_level (double temp, double air_prs, double ref_pressure)
{
	double water_level = 0;
	double pressure_diff = 0.0;

	/*
	 * Safety feature to make sure the readings are never negative
	 */
	if (air_prs >= 0.0) {
		last_good_pressure = air_prs;
	} else {
		air_prs = last_good_pressure;
	}

	/*
	 * Updates the ref_pressure to air_prs if air_prs is lower than ref_pressure
	 */
	pressure_diff = air_prs - ref_pressure;
	if (pressure_diff < 0) {
		ref_pressure = air_prs;
	}

	/*
	 * Boyle's Law
	 */
	water_level = (((air_prs - ref_pressure)/ (density * gravity)) * 100)  + offset_distance;
	if (water_level <= 5) {
		water_level = water_level - offset_distance;
	}

	/*
	* Safety feature to make sure the readings are never negative
	*/
	if (water_level < 0)
		water_level = -water_level;
	return water_level;
}

// ref_pressure from global.h
