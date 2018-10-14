#ifndef UART_H                              // Блокируем повторное включение этого модуля
#define UART_H

//******************************************************************************
//  Секция include: здесь подключаются заголовочные файлы используемых модулей
//******************************************************************************

#include "conf.h"

//******************************************************************************
//  Секция определения констант
//******************************************************************************

#define USE_FREERTOS				//uncomment this for using freeRTOS semaphore

//******************************************************************************
//  Секция определения типов
//******************************************************************************

typedef void (* uartRxCallbackPtr)(uint8_t data);   // used by uart driver to return recieved data to app

typedef enum serial_uart_t                          // Serial port device ID
{
	SERIAL_UART_NOTSET = -1,
    SERIAL_UART1 = 0,
    SERIAL_UART2 = 1,
    SERIAL_UART3 = 2,
    SERIAL_UART4 = 3,
    SERIAL_UART5 = 4,
    SERIAL_UART_NUM
} serial_uart_t;

typedef struct uart_drv_t
{
	UART_HandleTypeDef  huart;
	USART_TypeDef 	   *regs;				        /* UART port */
	uint8_t 		   *txBuffer;
	uint32_t 			txBufferTail;
	uint32_t 			txBufferHead;
	uartRxCallbackPtr 	uartCallback;
	char 				uart_mode[4];	            /* Режим порта в формате "8N1" */
} uart_drv_t;

typedef struct uart_cfg_t                           /* UART config structure */
{
	USART_TypeDef 	   *regs;				        /* UART port */
	GPIO_TypeDef 	   *rx_gpio_port;
	uint16_t			rx_gpio_pin;
	GPIO_TypeDef 	   *tx_gpio_port;
	uint16_t			tx_gpio_pin;
	uint8_t 			NVIC_IRQChannel;
	uint16_t 			txBufferSize;
} uart_cfg_t;

//******************************************************************************
//  Секция определения глобальных переменных
//******************************************************************************


//******************************************************************************
//  Секция прототипов глобальных функций
//******************************************************************************

int32_t uartInit(uint8_t uart_id, uint32_t speed, char *mode, uartRxCallbackPtr func);
void uartConfig(const uart_cfg_t * cfg, uint8_t num);
void UART_SendChar(uint8_t uart_id, uint8_t data);
void UART_SendString(uint8_t uart_id, const char *str, int len);
int UART_GetChar();

void uartWriteBin(uint8_t uart_id, const uint8_t * buff, uint32_t len);

#ifdef USE_FREERTOS
int UART_GetCharBlocking();
#endif

//******************************************************************************
//  Секция определения макросов
//******************************************************************************


//******************************************************************************

#if (uartSIZE_OF_RING_BUFFER > 256)
#error "uartSIZE_OF_RING_BUFFER must be less then 256"
#endif

#endif                       // Закрывающий #endif к блокировке повторного включения

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
