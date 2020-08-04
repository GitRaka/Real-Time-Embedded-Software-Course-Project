/*
 * controlTask.c
 *
 *  Created on: Aug 1, 2020
 *      Author: shamanth
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adchal.h"
/* Private variables ---------------------------------------------------------*/
uint32_t fanSpeedCmd;

/**
 *
 * @brief controlTask()
 *             Convert temperature to speed
 *
 * @return     none
 */

void controlTask()
{
	float tempValue;

	//Parameters to record execution time
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;

	// record start time
	getCurrentTime(&timStart);

	tempValue = rtdTemp;
	// Clear the update flag after data read
	tempValueReady = 0;

// open loop fan speed control
	if (tempValue <= 15)
	{
		fanSpeedCmd = 0;
	}

	else if (tempValue > 15 && tempValue < 30)
	{
		fanSpeedCmd = 1000;  //rpm
	}

	else if (tempValue > 30 && tempValue < 45)
	{
		fanSpeedCmd = 2000;  //rpm
	}

	else if (tempValue > 45 && tempValue < 55)
	{
		fanSpeedCmd = 2650;  //rpm
	}

	else if (tempValue > 55 && tempValue < 65)
	{
		fanSpeedCmd = 3300;  //rpm
	}

	else if (tempValue > 65 && tempValue < 80)
	{
		fanSpeedCmd = 3500;  //rpm
	}

	else if (tempValue > 80)
	{
		fanSpeedCmd = 4000;  //rpm
	}
	// Indicate to consumer that the data is ready
	speedValueReady = 1;
	//measure time elapsed by the task
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);
	if (timDelay > 1000)
	{
		//Software breakpoint if the deadline is missed
		__asm__("BKPT");
	}

}

/**
 *
 * @brief outputTask()
 *             Convert speed to duty cycle to drive the fan
 *
 * @return     none
 */

void outputTask()
{
	uint32_t speedValue;

	//Parameters to record execution time
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;

	// record start time
	getCurrentTime(&timStart);

	speedValue = fanSpeedCmd;
	// Clear the update flag after data read
	speedValueReady = 0;

// open loop fan speed control:
	// refer data sheet:
	// https://media.digikey.com/pdf/Data%20Sheets/Comair%20Rotron%20PDFs/19041196A_Spec.pdf
	switch (speedValue)
	{
	case 0:
		TIM1->CCR1 = PULSE1_VALUE;
		break;
	case 1000:
		TIM1->CCR1 = PULSE2_VALUE;
		break;
	case 2000:
		TIM1->CCR1 = PULSE3_VALUE;
		break;
	case 2650:
		TIM1->CCR1 = PULSE4_VALUE;
		break;
	case 3300:
		TIM1->CCR1 = PULSE5_VALUE;
		break;
	case 3500:
		TIM1->CCR1 = PULSE6_VALUE;
		break;
	case 4000:
		TIM1->CCR1 = PULSE7_VALUE;
		break;
	default:
		TIM1->CCR1 = PULSE1_VALUE;  // zero speed
		break;
	}

	//measure time elapsed by the task
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);
	if (timDelay > 1000)
	{
		//Software breakpoint if the deadline is missed
		__asm__("BKPT");
	}
}
