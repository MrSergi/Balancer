/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "conf.h"
#include "console.h"
#include "usb_device.h"
#include "sensors.h"

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint32_t periodBlink;
volatile uint32_t sysTickUptime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void User_PWM_SetPulseValue(uint16_t pulseValue)
{
	htim1.Instance->CCR2 = pulseValue;
}

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


void microrl_run(void *pvParameters)
{
	consoleInit();
	microrl_terminalInit();
	while(1)
	{
		microrl_terminalProcess();
	}
}

void vLedTask (void *pvParameters)
{
	static uint16_t pulseValue = 0;
	static int16_t step = 0;

    while(1)
    {
//    	vTaskDelay(periodBlink);

        vTaskDelay(10);
        if(pulseValue <= 0) step = 10;
        if(pulseValue >= 1000) step = -10;
        pulseValue += step;

      /** OC_Config->Pulse means TIMx->CCRx;
        * htim->Init->Period means TIMx->ARR;
        * Duty ratio equals (CCRx / (ARR + 1))
        * i.e. here equals (sConfigOC.Pulse / (htim4.Init.Period + 1))
        */
        User_PWM_SetPulseValue(pulseValue);
    }

    vTaskDelete(NULL);
}

void DebugTask (void *pvParameters)
{
	uint8_t TxData[50];
	uint8_t NByte;
	int16_t AccAngle;
	int16_t GyroAngle;
	int16_t Angle;

    while(1)
    {
    	NByte = 0;
    	Angle = (int16_t)pAngle;
    	AccAngle = (int16_t)AccSensor.Angle[Y];
    	GyroAngle = (int16_t)GyroAngleX;

    	TxData[NByte++] = 0x12;
    	TxData[NByte++] = Angle & 0xFF;
    	TxData[NByte++] = Angle >> 8;
    	TxData[NByte++] = 0x10;
    	TxData[NByte++] = AccAngle & 0xFF;
    	TxData[NByte++] = AccAngle >> 8;
    	TxData[NByte++] = 0x10;
    	TxData[NByte++] = GyroAngle & 0xFF;
    	TxData[NByte++] = GyroAngle >> 8;
    	TxData[NByte++] = 0x13;

    	CDC_Transmit_FS(TxData, NByte);

    	vTaskDelay(150);
    }

    vTaskDelete(NULL);
}

/* USER CODE END 0 */

portTASK_FUNCTION_PROTO(initTask, pvParameters)
{
	/* MCU Configuration----------------------------------------------------------*/
	uint32_t sysclock = 0;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	periodBlink = 100;

	/* Configure the system clock */
	SystemClock_Config();

	__HAL_RCC_I2C2_CLK_ENABLE();
	vTaskDelay(100);
	__HAL_RCC_I2C2_FORCE_RESET();
	vTaskDelay(100);
	__HAL_RCC_I2C2_RELEASE_RESET();
	vTaskDelay(100);

	vTaskDelay(1000);

	sysclock = HAL_RCC_GetSysClockFreq();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
//	MX_DMA_Init();
//	MX_ADC1_Init();
//	MX_TIM1_Init();
	MX_I2C2_Init();
	MX_USB_DEVICE_Init();

//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	SensInit();

	xTaskCreate(	SensorTask,"sensor",
					256,
					NULL,
					tskIDLE_PRIORITY + 1, // —амый низкий приоритет после 0
					NULL);

	xTaskCreate(	DebugTask,"debug",
					100,
					NULL,
					tskIDLE_PRIORITY + 2,
					NULL);

//	xTaskCreate(	vLedTask,"led",
//					100,
//					NULL,
//					tskIDLE_PRIORITY + 2,
//					NULL);

//	xTaskCreate(	microrl_run,"microrl",
//					500,
//					NULL,
//					tskIDLE_PRIORITY + 3,
//					NULL);


	/* Terminate initTask */
	vTaskDelete(NULL);
}

int main(void)
{
//	/* Disable all interrupts */
//	__set_PRIMASK(1);
//
//	/* Set the Vector Table base location at 0x4000 */
////	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
//	SCB->VTOR = (FLASH_BASE | 0x4000);
//
//	/* Enable all interrupts */
//	__set_PRIMASK(0);

	//		   Task_func		       Task_name   Stack	    Param  Prio			   Handler
	xTaskCreate(initTask, (signed char *) "Init",  128, (void *) NULL, 2, (xTaskHandle *) NULL);

	/* «апуск шедулера, после чего созданные задачи начнут выполн€тьс€. */
	vTaskStartScheduler();

	/* ≈сли все хорошо, то управление в main() никогда не дойдет до этой точки,
	     и теперь шедулер будет управл€ть задачами. ≈сли main() довела управление
	     до этого места, то это может означать, что не хватает пам€ти кучи (heap)
	     дл€ создани€ специальной задачи ожидани€ (idle task, об этой задаче
	     далее). „асть 5 предоставл€ет больше информации по управлению
	     пам€тью. */
	for( ;; );

	return 0;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  static uint32_t delay = 0;
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	  for(delay = 0; delay < 1000000; delay++)
		  continue;

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  /* USER CODE END Error_Handler */ 
}

/*************************************************************
*  Function:       micros
*------------------------------------------------------------
*  description:    —читывает врем€ безотказной работы системы в
*                  микросекундах (70 минут)
*  parameters:     void
*  on return:      uint32_t - системное врем€ в микросекундах
*************************************************************/
uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;
    do
    {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);

    return ((float)ms * 1000.0) + (72000.0 - (float)cycle_cnt) / 72.0;
}

/*******************************************************************************
 —екци€ обработчиков системных событий:
  ошибка MallocFailed - не удалось выделить пам€ть, скорее всего закончилась свободна€ пам€ть;
  ошибка StackOverflow - переполнение стека вызовов. Ѕесконечна€ рекурси€ или выделение слишком большой переменной;
  событие Idle - вызываетс€ каждый тик €дра RTOS.

ќшибки "обрабатываютс€" падением в бесконечный цикл. ќчевидно, это лучше чем пытатьс€ продолжать
программу с неверными данными. Ќа самом деле, лучше вызывать стандартный обработчик HARD_FAULT.

—обытие Idle обрабатываетс€ пустой процедурой. ¬ этом примере нам не нужно дополнительно что-то делать каждый тик.
*******************************************************************************/
//void vApplicationMallocFailedHook( void )
//{
//	HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
//	for( ;; );
//}
//
//void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
//{
//	HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
//	for( ;; );
//}
//
//void vApplicationIdleHook( void )
//{
//	HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
//	for( ;; );
//}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
