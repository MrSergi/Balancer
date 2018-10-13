//******************************************************************************
//  Секция include: здесь подключается заголовочный файл к модулю
//******************************************************************************

#include "hw.h"
#include "console.h"

//******************************************************************************
//  Секция определения переменных, используемых в модуле
//******************************************************************************

//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------

const gpio_config_t gpio_cfg[] =
{
	{ GPIOC, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW },	/* Led red     */
	{ GPIOC, GPIO_PIN_14, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW },	/* Led yellow  */
	{ GPIOB, GPIO_PIN_7,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW },	/* Direction of right motor */
	{ GPIOB, GPIO_PIN_9,  GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW },	/* Direction of left motor  */
};

const i2c_cfg_t i2c_cfg[] =
{
	{
		.regs 			   = I2C2,
		.scl_gpio_port 	   = GPIOB,
		.scl_gpio_pin 	   = GPIO_PIN_10,
		.sda_gpio_port     = GPIOB,
		.sda_gpio_pin      = GPIO_PIN_11,
        .ClkSpeed          = 100000,
	},
};

//const uart_cfg_t uart_cfg[] =
//{
//	{
//		.regs 			   = USART1,
//		.tx_gpio_port 	   = GPIOA,
//		.tx_gpio_pin 	   = GPIO_PIN_9,
//		.rx_gpio_port 	   = GPIOA,
//		.rx_gpio_pin 	   = GPIO_PIN_10,
//		.NVIC_IRQChannel   = USART1_IRQn,
//		.txBufferSize	   = 128,
//	},
//};

//******************************************************************************
//  Секция прототипов локальных функций
//******************************************************************************

void SystemClock_Config(void);

//******************************************************************************
//  Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************

/*******************************************************************************
*  Function:       hwInit
*-------------------------------------------------------------------------------
*  description:
*  parameters:     void
*  on return:      void
*******************************************************************************/
void hwInit(void)
{
	                                   /* MCU Configuration----------------------------------------------------------*/
	HAL_Init();                        /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

	SystemClock_Config();              /* Configure the system clock */

	i2cClockEnable(I2C2);

	vTaskDelay(100);

//	sysclock = HAL_RCC_GetSysClockFreq();

__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();
__HAL_RCC_GPIOD_CLK_ENABLE();

	gpioInit(gpio_cfg, HW_GPIO_COUNT);

	i2cConfig(i2c_cfg, 1);
	i2cInit(0);

//	uartConfig(uart_cfg, 1);
//	uartInit(0, 9600, "8N1", consoleInput);
	//	uartInit(0, 115200, "8N1", NULL);
}


/*******************************************************************************
*  Function:       SystemClock_Config
*-------------------------------------------------------------------------------
*  description:    System Clock Configuration
*  parameters:     void
*  on return:      void
*******************************************************************************/
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

/*******************************************************************************
*  Function:       Error_Handler
*-------------------------------------------------------------------------------
*  description:    This function is executed in case of error occurrence.
*  parameters:     void
*  on return:      void
*******************************************************************************/
void Error_Handler(void)
{
  static uint32_t delay = 0;
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  for(delay = 0; delay < 1000000; delay++)
		  continue;

	  gpioToggle(GPIO_LED_ERR);
  }
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
