#include "Dave.h"
#include "sound_buzzer.h"
#include "stdio.h"

//toss this away
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "task.h"



void sound_buzzer(double water)
{
	const TickType_t xFiveSeconds = pdMS_TO_TICKS( 5000UL );
	const TickType_t xThreeSeconds = pdMS_TO_TICKS( 3000UL );
	const TickType_t xOneHalfSecond = pdMS_TO_TICKS( 1500UL );
	double warning = 15;
	double danger = 30;
	double emergency = 45;

	/*Arbitrary water levels for warning to sound*/
	if(water>warning)
	{
		if (water>danger)
		{
			if(water>emergency)
			{
				DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_0);
				configPRINTF( ( "water level: emergency" ) );
				return;
			}
			DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_0);
			configPRINTF( ( "water level: danger" ) );
			vTaskDelay (xThreeSeconds);
			DIGITAL_IO_SetOutputLow(&DIGITAL_IO_0);
			return;
		}
		DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_0);
		configPRINTF( ( "water level: warning" ) );
		vTaskDelay (xOneHalfSecond);
		DIGITAL_IO_SetOutputLow(&DIGITAL_IO_0);
		return;
	}
	else
	{
		DIGITAL_IO_SetOutputLow(&DIGITAL_IO_0);
		configPRINTF( ( "setting led pin low" ) );
		return;
	}
}

