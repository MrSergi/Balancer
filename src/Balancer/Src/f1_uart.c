//******************************************************************************
//  Секция include: здесь подключается заголовочный файл к модулю
//******************************************************************************

#include "f1_gpio.h"
#include "f1_uart.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#endif

//******************************************************************************
//  Секция определения переменных, используемых в модуле
//******************************************************************************

//------------------------------------------------------------------------------
// Глобальные
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Локальные
//------------------------------------------------------------------------------


#ifdef USE_FREERTOS
xSemaphoreHandle xRxSemaphore;
xSemaphoreHandle xTxSemaphore;
#endif

uart_drv_t uart_drv[SERIAL_UART_NUM];
static uart_cfg_t * uart_p = NULL;
static uint8_t uart_num = 0;

//******************************************************************************
//  Секция прототипов локальных функций
//******************************************************************************

static void uartOpen(uint8_t uart_id, uint32_t speed);
static void uartClockEnable(USART_TypeDef *UARTx);

//******************************************************************************
//  Секция описания функций (сначала глобальных, потом локальных)
//******************************************************************************

/*******************************************************************************
*  Function:       uartInit
*-------------------------------------------------------------------------------
*  description:    Конфигурирует модуль UART
*  parameters:     void
*  on return:      void
*******************************************************************************/
int32_t uartInit(uint8_t uart_id, uint32_t speed, char *mode, uartRxCallbackPtr func)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	if (uart_id >= uart_num)
		return -1;	                                  // Не допустимый индекс порта

	if (uart_p == NULL)
		return -2;	                                  // Драйвер не инициализирован. Нет вызова uartConfig().

	strncpy(uart_drv[uart_id].uart_mode, mode, 3);

	uart_drv[uart_id].txBuffer = malloc(uart_p[uart_id].txBufferSize);

	uart_drv[uart_id].txBufferHead = 0;
	uart_drv[uart_id].txBufferTail = 0;
	uart_drv[uart_id].uartCallback = func;

	gpioClockEnable(uart_p[uart_id].rx_gpio_port);
	gpioClockEnable(uart_p[uart_id].tx_gpio_port);

	GPIO_InitStruct.Pin   = uart_p[uart_id].tx_gpio_pin;             /* Configure gpio pin */
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(uart_p[uart_id].tx_gpio_port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin   = uart_p[uart_id].rx_gpio_pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	HAL_GPIO_Init(uart_p[uart_id].rx_gpio_port, &GPIO_InitStruct);

	uartClockEnable(uart_p[uart_id].regs);

	uartOpen(uart_id, speed);

    HAL_NVIC_SetPriority(uart_p[uart_id].NVIC_IRQChannel, 5, 0);     /* USART1 interrupt Init */
    HAL_NVIC_EnableIRQ(uart_p[uart_id].NVIC_IRQChannel);

	__HAL_UART_ENABLE_IT(&uart_drv[uart_id].huart, UART_IT_RXNE);

#ifdef USE_FREERTOS
	xRxSemaphore = xSemaphoreCreateCounting(10,0);
	xTxSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xTxSemaphore);
#endif

	return 0;
}


/*******************************************************************************
*  Function:       uartConfig
*-------------------------------------------------------------------------------
*  description:    Конфигурирует модуль UART
*  parameters:     void
*  on return:      void
*******************************************************************************/
void uartConfig(const uart_cfg_t * cfg, uint8_t num)
{
	uart_p = cfg;
	uart_num = num;
}

/*******************************************************************************
*  Function:       uartOpen
*-------------------------------------------------------------------------------
*  description:
*  parameters:     void
*  on return:      void
*******************************************************************************/
static void uartOpen(uint8_t uart_id, uint32_t speed)
{
	uart_drv[uart_id].huart.Instance              = uart_p[uart_id].regs;
	uart_drv[uart_id].huart.Init.BaudRate         = speed;
	uart_drv[uart_id].huart.Init.Mode             = UART_MODE_TX_RX;
	uart_drv[uart_id].huart.Init.HwFlowCtl        = UART_HWCONTROL_NONE;
	uart_drv[uart_id].huart.Init.OverSampling     = UART_OVERSAMPLING_16;

    if (uart_drv[uart_id].uart_mode[0] == '9')
    	uart_drv[uart_id].huart.Init.WordLength   = UART_WORDLENGTH_9B;
    else
    	uart_drv[uart_id].huart.Init.WordLength   = UART_WORDLENGTH_8B;

    if (uart_drv[uart_id].uart_mode[1] == 'E')
    	uart_drv[uart_id].huart.Init.Parity 	  = UART_PARITY_EVEN;
	else if (uart_drv[uart_id].uart_mode[1] == 'O')
		uart_drv[uart_id].huart.Init.Parity 	  = UART_PARITY_ODD;
	else
		uart_drv[uart_id].huart.Init.Parity 	  = UART_PARITY_NONE;

    if (uart_drv[uart_id].uart_mode[2] == '2')
    	uart_drv[uart_id].huart.Init.StopBits 	  = UART_STOPBITS_2;
    else
    	uart_drv[uart_id].huart.Init.StopBits 	  = UART_STOPBITS_1;

    HAL_UART_Init(&uart_drv[uart_id].huart);
}

/*******************************************************************************
*  Function:       uartWrite
*-------------------------------------------------------------------------------
*  description:    Отправка байта
*  parameters:     void
*  on return:      void
*******************************************************************************/
void uartWrite(uint8_t uart_id, uint8_t ch)
{
	if (uart_id >= uart_num)
		return;	                                  // Не допустимый индекс порта

	uart_drv[uart_id].txBuffer[uart_drv[uart_id].txBufferHead] = ch;

	uart_drv[uart_id].txBufferHead = (uart_drv[uart_id].txBufferHead + 1) % uart_p[uart_id].txBufferSize;

	__HAL_UART_ENABLE_IT(&uart_drv[uart_id].huart, UART_IT_TXE);
}

/*******************************************************************************
*  Function:       uartWriteStr
*-------------------------------------------------------------------------------
*  description:    Отправка байта
*  parameters:     void
*  on return:      void
*******************************************************************************/
void uartWriteStr(uint8_t uart_id, char *str)
{
    while (*str)
    {
        uartWrite(uart_id, *(str++));
    }
}

/*******************************************************************************
*  Function:       uartWriteBin
*-------------------------------------------------------------------------------
*  description:    Отправка байта
*  parameters:     void
*  on return:      void
*******************************************************************************/
void uartWriteBin(uint8_t uart_id, const uint8_t * buff, uint32_t len)
{
	xSemaphoreTake(xTxSemaphore, portMAX_DELAY);

	uint32_t  i = 0;

	while (i < len)
	{
		uartWrite(uart_id, buff[i++]);
	}

	xSemaphoreGive(xTxSemaphore);
}

/*******************************************************************************
*  Function:       uartClockEnable
*-------------------------------------------------------------------------------
*  description:
*  parameters:     void
*  on return:      void
*******************************************************************************/
static void uartClockEnable(USART_TypeDef *UARTx)
{
		 if (UARTx == USART1)	__HAL_RCC_USART1_CLK_ENABLE();
	else if (UARTx == USART2)	__HAL_RCC_USART2_CLK_ENABLE();
	else if (UARTx == USART3)	__HAL_RCC_USART3_CLK_ENABLE();
	else if (UARTx == UART4)	__HAL_RCC_UART4_CLK_ENABLE();
	else if (UARTx == UART5)	__HAL_RCC_UART5_CLK_ENABLE();
}


/*******************************************************************************
*  Function:       USARTx_IRQHandler
*-------------------------------------------------------------------------------
*  description:    Прерывания модуля UART
*  parameters:     void
*  on return:      void
*******************************************************************************/
void USARTx_IRQHandler(uint8_t uart_id)
{
	uint8_t data = 0;

	if(__HAL_UART_GET_FLAG(&uart_drv[uart_id].huart, UART_FLAG_RXNE) != RESET)
	{
		data = (uint8_t)(uart_drv[uart_id].huart.Instance->DR & 0x00FF);            // принимаем байт
		if (uart_drv[uart_id].uartCallback)
			uart_drv[uart_id].uartCallback(data);
		__HAL_UART_CLEAR_FLAG(&uart_drv[uart_id].huart, UART_FLAG_RXNE);
#ifdef USE_FREERTOS
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xRxSemaphore, &xHigherPriorityTaskWoken);
		if( xHigherPriorityTaskWoken != pdFALSE )
		{
			portYIELD();
		}
#endif
	}
	else if (__HAL_UART_GET_IT_SOURCE(&uart_drv[uart_id].huart, UART_IT_TXE) != RESET)
	{
		if (uart_drv[uart_id].txBufferTail != uart_drv[uart_id].txBufferHead)
		{
			uart_drv[uart_id].huart.Instance->DR = uart_drv[uart_id].txBuffer[uart_drv[uart_id].txBufferTail]; // передаём байт
			uart_drv[uart_id].txBufferTail = (uart_drv[uart_id].txBufferTail + 1) % uart_p[uart_id].txBufferSize;

			if (uart_drv[uart_id].txBufferTail == uart_drv[uart_id].txBufferHead)
			{
				__HAL_UART_DISABLE_IT(&uart_drv[uart_id].huart, UART_IT_TXE);
//	            __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
			}
		}
		else
		{
			__HAL_UART_DISABLE_IT(&uart_drv[uart_id].huart, UART_IT_TXE);
//	        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
		}
	}

//	HAL_UART_IRQHandler(&huart1);
}

void USART1_IRQHandler(void)
{
	USARTx_IRQHandler(SERIAL_UART1);
}

void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(SERIAL_UART2);
}

void USART3_IRQHandler(void)
{
	USARTx_IRQHandler(SERIAL_UART3);
}

void UART4_IRQHandler(void)
{
	USARTx_IRQHandler(SERIAL_UART4);
}

void UART5_IRQHandler(void)
{
	USARTx_IRQHandler(SERIAL_UART5);
}

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
