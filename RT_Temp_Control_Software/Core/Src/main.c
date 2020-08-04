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
#include "stdio.h"  //printfs using SWV
#include "adchal.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
/* transfer state */
TIM_HandleTypeDef htim1;
__IO uint32_t wTransferState = TRANSFER_WAIT;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
uint32_t cnt;
unsigned long long usTimerPeriodCounter = 0;
//uint32_t traceCount;
// unsigned long long usTimerPeriodCounter;
TIM_OC_InitTypeDef sConfigOC = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_Init();
static void MX_TIM1_Init(void);
static void MX_DMA_Init();
static void MX_SPI1_Init();
static void EXTI15_10_IRQHandler_Config(void);
// static void ADC_RESET_Pin_Init(void);
// static void ADC_START_Pin_Init(void);
void readRTDtemp();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Forward all printf calls to ITM.
  * @retval int
  */

//int _write(int file, char *ptr, int len)
//{
//  /* Implement your write code here, this is used by puts and printf for example */
//  int i=0;
//  for(i=0 ; i<len ; i++)
//    ITM_SendChar((*ptr++));
//  return len;
//}

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

  //ITM_SendChar('a');
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  /* Configure LED2 */
  BSP_LED_Init(LED2);

  /* Configure User push-button button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* Initialize all configured peripherals */
  MX_TIM7_Init();
  MX_TIM1_Init();
  //MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  //User code initialization
  //Set DRDY Interrupt to GPIO Pin 08
  EXTI15_10_IRQHandler_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_NVIC_SetPriority(TIM7_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);


  /*## Start PWM signals generation #######################################*/
  /* Start channel 1 */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
      /* PWM Generation Error */
      Error_Handler();
    }

  // Configure LED2
  //BSP_LED_Init(LED2);

  //Function to output any internal clock on PA08.

  //HAL_RCC_MCOConfig(RCC_MCO1, RCC_CFGR_MCOPRE_DIV16, RCC_MCO1SOURCE_SYSCLK);
  //BSP_LED_On(LED2);
    tempValueReady = 0;
    readRTDtemp();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		processInput();

		if (tempValueReady)
		{
			controlTask(); // Convert to speed
		}

		if (speedValueReady)
		{
			outputTask(); // Convert to speed
		}

//		if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_SET)
//		{
//			i++;
//			BSP_LED_Toggle(LED2);
//			HAL_Delay(100);
//			if (i == 1)
//			   TIM1->CCR1 = PULSE1_VALUE;
//			if (i == 2)
//			   TIM1->CCR1 = PULSE2_VALUE;
//			if (i == 3)
//			   TIM1->CCR1 = PULSE3_VALUE;
//			if (i == 4)
//			{
//			   TIM1->CCR1 = PULSE4_VALUE;
//			   i=0;
//			}
//		}


  }
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

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
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

/* USER CODE BEGIN 4 */
/* -----------------------------------------------------------------------------*/
/**
  * @brief delay in microseconds
  * @retval None
  */
void delay_us(uint32_t us)
{
	HAL_TIM_Base_Start(&htim7);
	//__TIM7_FORCE_RESET();
	while(TIM7->CNT < us);
	HAL_TIM_Base_Stop(&htim7);
}

/* -----------------------------------------------------------------------------*/
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
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
	usTimerPeriodCounter++;

	//HAL_TIM_Base_Stop_IT(&htim7);
	//HAL_TIM_Base_Start_IT(&htim7);
}
/* -----------------------------------------------------------------------------*/
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;  //Updated this to reflect CPHA = 1
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; //sets fpclk/4 = 24MHz/4 = 6MHz (TI is 5MHz)
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; // Check this
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
/* -----------------------------------------------------------------------------*/
/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  // DMA controller clock enable
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA interrupt init
  // DMA1_Channel2_IRQn interrupt configuration
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  // DMA1_Channel3_IRQn interrupt configuration
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}
/* -----------------------------------------------------------------------------*/
/**
  * @brief GPIO Initializations
  * @retval None
  */
void MX_GPIO_Init()
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	//SPI Ports clock enabled
	__HAL_RCC_GPIOB_CLK_ENABLE();

	// Enable GPIOC clock
	ADC_START_GPIO_CLK_ENABLE();
	//static void ADC_START_Pin_Init(void);
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin   = ADC_START_GPIO_PIN;   //Currently PC12
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	// GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(ADC_START_GPIO_PORT, &GPIO_InitStructure);

	// Enable GPIOC clock
	//ADC_RESET_GPIO_CLK_ENABLE();
	//static void ADC_RESET_Pin_Init(void);
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pin   = ADC_RESET_GPIO_PIN;   //Currently PC10
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	// GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(ADC_RESET_GPIO_PORT, &GPIO_InitStructure);

	// GPIO to test the timer
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/* -----------------------------------------------------------------------------*/

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = PRESCALER_VALUE;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = CAL_PERIOD_REG_VALUE(FREQ);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PULSE1_VALUE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PULSE2_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PULSE3_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PULSE4_VALUE;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  //GPIO Initialization for all the PWM outputs
  HAL_TIM_MspPostInit(&htim1);

}
/* -----------------------------------------------------------------------------*/

static void EXTI15_10_IRQHandler_Config(void)
/**
  * @brief  Configures EXTI lines 10 to 15 (connected to PC.13 pin) in interrupt mode
  * @param  None
  * @retval None
  */
{
  GPIO_InitTypeDef   GPIO_InitStructure;


  // Enable GPIOC clock
  DRDY_GPIO_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;


  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = DRDY_GPIO_PIN;
  HAL_GPIO_Init(DRDY_GPIO_PORT, &GPIO_InitStructure);


  /* Enable and set lines 10 to 15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(DRDY_EXTI_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DRDY_EXTI_IRQn);
}
///* -----------------------------------------------------------------------------*/
///**
//  * @brief  Configures ADC Reset GPIO Pin
//  * @param  None
//  * @retval None
//  */
//
//static void ADC_START_Pin_Init(void)
//{
//  GPIO_InitTypeDef   GPIO_InitStructure;
//
//
//  // Enable GPIOC clock
//  //ADC_START_GPIO_CLK_ENABLE();
//
//  /* Configure PC.13 pin as input floating */
//  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.Pin   = ADC_START_GPIO_PIN;   //Currently PC10
//  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.Pull  = GPIO_NOPULL;
//  // GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//
//  HAL_GPIO_Init(ADC_START_GPIO_PORT, &GPIO_InitStructure);
//}
//
///* -----------------------------------------------------------------------------*/
//
///**
//  * @brief  Configures ADC Reset GPIO Pin
//  * @param  None
//  * @retval None
//  */
//
//static void ADC_RESET_Pin_Init(void)
//{
//  GPIO_InitTypeDef   GPIO_InitStructure;
//
//  /* Configure PC.13 pin as input floating */
//  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.Pin   = ADC_RESET_GPIO_PIN;   //Currently PC10
//  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStructure.Pull  = GPIO_NOPULL;
//  // GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//
//  HAL_GPIO_Init(ADC_RESET_GPIO_PORT, &GPIO_InitStructure);
//}

/* -----------------------------------------------------------------------------*/
/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of DMA TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED2 on: Transfer in transmission/reception process is complete */
  BSP_LED_On(LED2);
  wTransferState = TRANSFER_COMPLETE;
}
/* -----------------------------------------------------------------------------*/
/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}

/* -----------------------------------------------------------------------------*/

/**
  * @brief  record current timer count value in us.
  * @param  timeSpec
  * @retval uint32_t
  */

void getCurrentTime (timeSpec *currTime)
{
	currTime->usTimerResetcount = usTimerPeriodCounter;
	currTime->usElapsed = TIM7->CNT;
}

/**
  * @brief  measure time diff between time stamps
  * @param  timeSpec,timeSpec
  * @retval uint32_t
  */

uint32_t getTimeDiff (timeSpec *timStop, timeSpec *timStart)
{
	uint32_t rollDiff = 0;
	rollDiff = timStop->usTimerResetcount - timStart->usTimerResetcount;

	return ((timStop->usElapsed  - timStart->usElapsed) + 65535 * rollDiff );

}

/*-----------------------------------------------------------------------------*/


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
