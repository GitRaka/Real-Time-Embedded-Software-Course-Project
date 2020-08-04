/*
 * scheduler.c
 *
 *  Created on: 01-Jul-2020
 *      Author: Shamanth
 *       */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adchal.h"

/* Externs--------------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Scheduler function to implement RM policy.
  *
  */
void schedulerExecutive(void)
{

	uint32_t measuredTicks;
	measuredTicks = HAL_GetTick();

	if (measuredTicks % 50 == 0)
	{
		//ADC read counts and convert to resistance and then to temp
		processInput();  //Emulator implementation built-in just like the real hardware driver code
	}

	if (measuredTicks % 60 == 0)
	{
		if (tempValueReady)   //Can be replaced with arm semaphores if there are too many interrupts
		{
			controlTask(); // Convert to speed
		}
	}

	if (measuredTicks % 75 == 0)
	{
		if (speedValueReady)  //Can be replaced with arm semaphores if there are too many interrupts
		{
			outputTask(); // Convert to speed
		}
	}

}
