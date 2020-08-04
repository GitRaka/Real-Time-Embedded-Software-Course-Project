/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g4xx_nucleo.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct timeSpecDef {

	uint32_t usTimerResetcount;
	uint32_t usElapsed;
} timeSpec;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void delay_us(uint32_t us);

void processInput();
void controlTask();
void outputTask();

void getCurrentTime (timeSpec *currTime);
uint32_t getTimeDiff (timeSpec *timStop, timeSpec *timStart);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* Size of buffer */
// #define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)
#define ONE 1
/**
  * @brief DRDY GPIO Pin
  */
#define DRDY_GPIO_PIN                  GPIO_PIN_8
#define DRDY_GPIO_PORT                 GPIOC
#define DRDY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()
#define DRDY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()
#define DRDY_EXTI_IRQn                 EXTI15_10_IRQn
#define DRDY_EXTI_LINE                 EXTI_LINE_13

/**
  * @brief ADC_RESET GPIO Pin
  */
#define ADC_RESET_GPIO_PIN             	   GPIO_PIN_10
#define ADC_RESET_GPIO_PORT           	   GPIOC
#define ADC_RESET_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_RESET_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOC_CLK_DISABLE()

/**
  * @brief ADC_RESET GPIO Pin
  */
#define ADC_START_GPIO_PIN             	   GPIO_PIN_12
#define ADC_START_GPIO_PORT           	   GPIOC
#define ADC_START_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_START_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOC_CLK_DISABLE()


enum
{
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};


/* Private defines for Timer PWM config -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* Compute the prescaler value to have TIM1 counter clock equal to 37500000 Hz */

#define PRESCALER_VALUE     (uint32_t)(((SystemCoreClock) / 1000000) - 1)

/* -----------------------------------------------------------------------
TIM1 Configuration: generate 4 PWM signals with 4 different duty cycles.

    In this example TIM1 input clock (TIM1CLK) is set to APB2 clock (PCLK2),
    since APB2 prescaler is equal to 1.
      TIM1CLK = PCLK2
      PCLK1 = HCLK
      => TIM1CLK = HCLK = SystemCoreClock

    To get TIM1 counter clock at 85 MHz, the prescaler is computed as follows:
       Prescaler = (TIM1CLK / TIM1 counter clock) - 1
       Prescaler = ((SystemCoreClock) /85 MHz) - 1

    To get TIM1 output clock at 85 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM1 counter clock / TIM1 output clock) - 1
           = 999

    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR + 1)* 100 = 50%
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR + 1)* 100 = 37.5%
    TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 25%
    TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 12.5%

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32g4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

/* Initialize TIMx peripheral as follows:
   + Prescaler = (SystemCoreClock / 85000000) - 1
   + Period = (1000 - 1)
   + ClockDivision = 0
   + Counter direction = Up
*/
#define  PERIOD_VALUE       			(uint32_t)(1000 - 1)              /* Period Value  */

#define  FREQ 							  4000
#define  CAL_PERIOD_REG_VALUE(Freq)       (((uint32_t)(SystemCoreClock/((PRESCALER_VALUE+1)*Freq))) - 1)

#define  PULSE1_VALUE       			  (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * 10.0 / 100)              /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       		      (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * 15.0 / 100)     /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       		      (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * 20.0 / 100)              /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       		      (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * 50.0 / 100)
#define  PULSE5_VALUE       		      (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * 75.0 / 100)
#define  PULSE6_VALUE       		      (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * 80.0 / 100)
#define  PULSE7_VALUE       		      (uint32_t)(CAL_PERIOD_REG_VALUE(FREQ) * 97.0 / 100)


/* Capture Compare 4 Value  */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
