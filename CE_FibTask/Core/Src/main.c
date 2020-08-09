/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FIB_LIMIT_FOR_32_BIT 47

#define FIB_TEST(seqCnt, iterCnt)      \
   for(idx=0; idx < iterCnt; idx++)    \
   {                                   \
      fib = fib0 + fib1;               \
      while(jdx < seqCnt)              \
      {                                \
         fib0 = fib1;                  \
         fib1 = fib;                   \
         fib = fib0 + fib1;            \
         jdx++;                        \
      }                                \
   }                                   \

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
volatile unsigned int idx = 0, jdx = 1;
volatile unsigned int fib = 0, fib0 = 0, fib1 = 1;
volatile unsigned int seqIter=1000, loopIter=1000;
volatile unsigned long long usTimerPeriodCounter = 0;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
extern void initialise_monitor_handles(void);
/* USER CODE BEGIN PFP */
uint32_t measuredTicks;

void fib200(void)
{
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;
	//uint32_t measuredTicks;
	uint32_t measuredTicks1;
	uint32_t time_diff = 0;

	getCurrentTime(&timStart);
	FIB_TEST(seqIter * 220, loopIter * 220);
	measuredTicks1 = HAL_GetTick();
	time_diff = measuredTicks1 -  measuredTicks;
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);
	if (timDelay >= 200000)
	{
		// Control task exceeded its deadline
		printf("Task-fib200 missed deadline at ticks:%ld.\n",uwTick);
	}
	else
	{
		printf("Completed the fib200 task in %ld\n\n", timDelay);
	}
}


void fib20(void)
{
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;
	//uint32_t measuredTicks;
	uint32_t measuredTicks1;
	uint32_t time_diff = 0;

	getCurrentTime(&timStart);
	FIB_TEST(seqIter * 8, loopIter * 8);
	measuredTicks1 = HAL_GetTick();
	time_diff = measuredTicks1 -  measuredTicks;
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);
	if (timDelay >= 20000)
	{
		// Control task exceeded its deadline
		printf("Task-fib20 missed deadline at ticks:%ld.\n",uwTick);
	}
	else
	{
		printf("Completed the fib01 task in %ld\n\n", timDelay);
	}
}

void fib4(void)
{
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;
	//uint32_t measuredTicks;
	uint32_t measuredTicks1;
	uint32_t time_diff = 0;

	getCurrentTime(&timStart);
	FIB_TEST(seqIter * 2, loopIter * 2);
	measuredTicks1 = HAL_GetTick();
	time_diff = measuredTicks1 -  measuredTicks;
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);
	if (timDelay >= 4000)
	{
		// Control task exceeded its deadline
		printf("Task-fib4 missed deadline at ticks:%ld.\n",uwTick);
	}
	else
	{
		printf("Completed the fib4 task in %ld\n\n", timDelay);
	}
}

void fib2()
{
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;
	//uint32_t measuredTicks;
	uint32_t measuredTicks1;
	uint32_t time_diff = 0;

	getCurrentTime(&timStart);
	FIB_TEST(seqIter+3, loopIter+3);
	measuredTicks1 = HAL_GetTick();
	time_diff = measuredTicks1 -  measuredTicks;
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);
	if (timDelay > 2500)
	{
		// Control task exceeded its deadline
		printf("Task-fib2 missed deadline at ticks:%ld.\n",uwTick);
	}
	else
	{
		printf("Completed the fib2 task in %ld\n\n", timDelay);
	}
}

void fib01()
{
	timeSpec timStart = {0, 0};
	timeSpec timStop = {0, 0};
	uint32_t timDelay = 0;
	//uint32_t measuredTicks;
	uint32_t measuredTicks1;
	uint32_t time_diff = 0;

	getCurrentTime(&timStart);
	FIB_TEST(seqIter-995, loopIter-995);
	measuredTicks1 = HAL_GetTick();
	time_diff = measuredTicks1 -  measuredTicks;
	getCurrentTime(&timStop);
	timDelay = getTimeDiff(&timStop, &timStart);
	if (timDelay >= 1000)
	{
		// Control task exceeded its deadline
		printf("Task-fib01 missed deadline at ticks:%ld.\n",uwTick);
	}
	else
	{
		printf("Completed the fib01 task in %ld\n\n", timDelay);
	}
}

/**
  * @brief  Scheduler function to implement RM policy.
  *
  */
void schedulerExecutive(void)
{

	uint32_t measuredTicks;
	measuredTicks = HAL_GetTick();

	/* USER CODE END WHILE */
	measuredTicks = HAL_GetTick();
	//printf("\nmeasuredTicks--->%ld\n", measuredTicks);
	//getCurrentTime(&timStart);
	if ((measuredTicks % 20) == 0)
	{
		fib01();  //sample testing for the fibonacci series
	}
	else if((measuredTicks % 25) == 0)
	{
		fib2();  //sample testing for the fibonacci series
	}
	else if((measuredTicks % 126) == 0)
	{
		fib4();  //sample testing for the fibonacci series
	}
	else if((measuredTicks % 251) == 0)
	{
		fib20();  //sample testing for the fibonacci series
	}
	else if((measuredTicks % 501) == 0)
	{
		fib200();  //sample testing for the fibonacci series
	}
}

static void MX_TIM7_Init(void);
//static void EXTI15_10_IRQHandler_Config(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  initialise_monitor_handles();
  /* USER CODE END SysInit */
  printf("Initializing...\n");
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  MX_TIM7_Init();
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_NVIC_SetPriority(TIM7_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//
//
//    //HAL_TIM_Base_Stop_IT(&htim7);
//	//printf("check\n");
//    /* USER CODE BEGIN 3 */
//  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	// RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	// HSE = 24Mhz, SYSCLK = PLL = (HSE/2)*(PLLN/PLLM)
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 25;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	  Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
	{
	  Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

	  __HAL_RCC_TIM7_CLK_ENABLE();
  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 150;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
	  //__HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief Error handler for timer period elapsed
  * @param None
  * @retval None
  */
void TIM7_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim7);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
	usTimerPeriodCounter++;

	//HAL_TIM_Base_Stop_IT(&htim7);
	//HAL_TIM_Base_Start_IT(&htim7);
}

/**
  * @brief  record current timer count value in us.
  * @param  timeSpec
  * @retval uint32_t
  */

void getCurrentTime (timeSpec *currTime)
{
	//__HAL_LOCK(&htim7);
	currTime->usElapsed = TIM7->CNT;
	currTime->usTimerResetcount = usTimerPeriodCounter;
	//__HAL_UNLOCK(&htim7);
}

/**
  * @brief  measure time diff between time stamps
  * @param  timeSpec,timeSpec
  * @retval uint32_t
  */

uint32_t getTimeDiff (timeSpec *timStop, timeSpec *timStart)
{
	uint32_t rollDiff = 0;
	uint32_t usdiff = 0;
	rollDiff = timStop->usTimerResetcount - timStart->usTimerResetcount;
	usdiff = timStop->usElapsed  - timStart->usElapsed;

	if (rollDiff == 0)
	{
		if (usdiff >= 0)
			return usdiff;
		else
			return ((65535 - timStart->usElapsed) + timStop->usElapsed);
	}
	else
		return ((65535 - timStart->usElapsed) + timStop->usElapsed + (65535*(rollDiff-1)));

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
