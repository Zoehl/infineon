#include "Dave.h"
#include "sound_buzzer.h"
#include "stdio.h"

//toss this away
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
<<<<<<< HEAD
#include "task.h"
=======
>>>>>>> 0c0a8a90b8ec990f9bc6cd970d62973dd75b1963



void sound_buzzer(double water) {
<<<<<<< HEAD
	const TickType_t xFiveSeconds = pdMS_TO_TICKS( 5000UL );
	const TickType_t xThreeSeconds = pdMS_TO_TICKS( 3000UL );
	const TickType_t xOneHalfSecond = pdMS_TO_TICKS( 1500UL );
	double warning = 15;
	double danger = 30;
	double emergency = 45;
	//static uint32_t duty = (uint32_t)500;
	//static uint32_t time_interval = (uint32_t) 500000000U;
	//DAVE_Init();
=======
	double danger = 20;
	//static uint32_t duty = (uint32_t)500;
	//static uint32_t time_interval = (uint32_t) 500000000U;
	DAVE_Init();
>>>>>>> 0c0a8a90b8ec990f9bc6cd970d62973dd75b1963
//	DAVE_STATUS_t status;
//	status = DAVE_Init();
//	if(status != DAVE_STATUS_SUCCESS)
//		{
//			XMC_DEBUG("DAVE APPs initialization failed\n");
//		}

	//PWM_SetDutyCycle(&PWM_0,duty);
	//TIMER_SetTimeInterval(&TIMER_0,time_interval);
<<<<<<< HEAD
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
=======
	if(water>danger){
		DIGITAL_IO_SetOutputHigh(&DIGITAL_IO_0);
		configPRINTF( ( "setting led pin high" ) );
		return;
	}else{
>>>>>>> 0c0a8a90b8ec990f9bc6cd970d62973dd75b1963
		DIGITAL_IO_SetOutputLow(&DIGITAL_IO_0);
		configPRINTF( ( "setting led pin low" ) );
		return;
	}

	//return;
//	TIMER_Clear(&TIMER_0);
//	TIMER_ClearEvent(&TIMER_0);
//	TIMER_Start(&TIMER_0);
//	PWM_Start(&PWM_0);
}

//void time_interval_event(void)
//{
//	PWM_Stop(&PWM_0);
//	return;
//}
//

