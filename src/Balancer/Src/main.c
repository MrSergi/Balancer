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
#include "hw.h"
#include "console.h"
#include "sensors.h"
#include "motor.h"


//******************************************************************************
//  Секция определения переменных, используемых в модуле
//******************************************************************************

//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------

void SystemClock_Config(void);

//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------

volatile uint32_t periodBlink;
volatile uint32_t sysTickUptime = 0;

//******************************************************************************
//  Секция прототипов локальных функций
//******************************************************************************

static void MX_DMA_Init(void);

//******************************************************************************
//  Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************

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
//	static uint16_t pulseValue = 0;
//	static int16_t step = 0;

    while(1)
    {
    	vTaskDelay(300);

    	gpioToggle(GPIO_LED_STA);

//        vTaskDelay(10);
//        if(pulseValue <= 0) step = 10;
//        if(pulseValue >= 1000) step = -10;
//        pulseValue += step;

      /** OC_Config->Pulse means TIMx->CCRx;
        * htim->Init->Period means TIMx->ARR;
        * Duty ratio equals (CCRx / (ARR + 1))
        * i.e. here equals (sConfigOC.Pulse / (htim4.Init.Period + 1))
        */
//        User_PWM_SetPulseValue(pulseValue);
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
    	AccAngle = direction;//(int16_t)AccSensor.Angle[Y];
    	GyroAngle = (int16_t)(DriveSpeed * 10.0);//(int16_t)GyroAngleX;

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

//    	CDC_Transmit_FS(TxData, NByte);

    	vTaskDelay(150);
    }

    vTaskDelete(NULL);
}


portTASK_FUNCTION_PROTO(initTask, pvParameters)
{
	hwInit();                                     /* Initialize all configured peripherals */
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();

	UART_Init();

	SensInit();

	MotorCtrlDrive(0.0);                          // Отключаем управление моторами на 5 сек.
	vTaskDelay(1000);


#ifdef DEBUG
	xTaskCreate(	DebugTask,"debug",
					128,
					NULL,
					tskIDLE_PRIORITY + 2,
					NULL);
#endif

	xTaskCreate(	vLedTask,"led",
					256,
					NULL,
					tskIDLE_PRIORITY + 1,         // Самый низкий приоритет после 0
					NULL);

	xTaskCreate(	microrl_run,"microrl",
					1024,
					NULL,
					tskIDLE_PRIORITY + 2,
					NULL);

	xTaskCreate(	SensorTask,"sensor",
					256,
					NULL,
					tskIDLE_PRIORITY + 3,
					NULL);


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
	xTaskCreate(initTask, (const char *) "Init",  256, (void *) NULL, 2, (xTaskHandle *) NULL);

	/* Запуск шедулера, после чего созданные задачи начнут выполняться. */
	vTaskStartScheduler();

	/* Если все хорошо, то управление в main() никогда не дойдет до этой точки,
	     и теперь шедулер будет управлять задачами. Если main() довела управление
	     до этого места, то это может означать, что не хватает памяти кучи (heap)
	     для создания специальной задачи ожидания (idle task, об этой задаче
	     далее). Часть 5 предоставляет больше информации по управлению
	     памятью. */
	for( ;; );

	return 0;
}


/*************************************************************
*  Function:       MX_DMA_Init
*------------------------------------------------------------
*  description:    Enable DMA controller clock
*  parameters:     void
*  on return:      uint32_t - системное время в микросекундах
*************************************************************/
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/*************************************************************
*  Function:       micros
*------------------------------------------------------------
*  description:    Считывает время безотказной работы системы в
*                  микросекундах (70 минут)
*  parameters:     void
*  on return:      uint32_t - системное время в микросекундах
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
 Секция обработчиков системных событий:
  ошибка MallocFailed - не удалось выделить память, скорее всего закончилась свободная память;
  ошибка StackOverflow - переполнение стека вызовов. Бесконечная рекурсия или выделение слишком большой переменной;
  событие Idle - вызывается каждый тик ядра RTOS.

Ошибки "обрабатываются" падением в бесконечный цикл. Очевидно, это лучше чем пытаться продолжать
программу с неверными данными. На самом деле, лучше вызывать стандартный обработчик HARD_FAULT.

Событие Idle обрабатывается пустой процедурой. В этом примере нам не нужно дополнительно что-то делать каждый тик.
*******************************************************************************/
//void vApplicationMallocFailedHook( void )
//{
//	gpioToggle(GPIO_LED_ERR);
//	for( ;; );
//}
//
//void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
//{
//	gpioToggle(GPIO_LED_ERR);
//	for( ;; );
//}
//
//void vApplicationIdleHook( void )
//{
//	gpioToggle(GPIO_LED_ERR);
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
