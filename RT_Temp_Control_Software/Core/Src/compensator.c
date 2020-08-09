/*
 * compensator.c
 *
 *  Created on: Aug 3, 2020
 *      Author: shama
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adchal.h"

/* Private variables ---------------------------------------------------------*/
uint32_t fanSpeed;
uint32_t errorPrev = 0;
uint32_t intgPrev = 0;
uint32_t Kp = 0;
uint32_t Ki = 0;
uint32_t Kd = 0;
uint32_t error, intg, derv;
uint32_t dt;
float d;
/**
  * @brief  PI compensator
  *
  */
void PI_Loop(void)
{

	error = fanSpeedRef - fanSpeed;
	intg = intgPrev + error*dt;
	derv = (error - errorPrev)/dt;
	d = Kp * error + Ki * intg + Kd * derv;

	if (d <= 0.5)
		d = 0.5;
	else if (d >= 0.9)
		d = 0.9;

	TIM1->CCR1 = (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * d);

	//store all previous values
	errorPrev = error;
	intgPrev = intg;
	dt = dt+1;
}
