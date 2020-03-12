/*
 * global.c
 *
 *  Created on: Sep 24, 2019
 *      Author: Zoehl's Rig
 */

double get_ref_pressure (void)
{
	double temp;
	double press;
	if (dps310_hal_get_temp_press(&temp, &press) == -1) {
		return 0;
	}
	return press;
}



